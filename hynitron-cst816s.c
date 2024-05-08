// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for I2C connected CST816S touchsreen
 *
 * Copyright (C) 2024 Oleh Kuzhylnyi <kuzhylol@gmail.com>
 */
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of_irq.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>

#define CST816S_MAX_X 240
#define CST816S_MAX_Y CST816S_MAX_X

enum cst816s_commands {
        CST816S_GET_RAW_CMD = 0x01,
        CST816S_GET_VERSION_CMD = 0x15,
        CST816S_GET_VERSION_INFO_CMD = 0xA7,
};

enum cst816_gesture_id {
    CST816S_NONE = 0x00,
    CST816S_SWIPE_UP = 0x01,
    CST816S_SWIPE_DOWN = 0x02,
    CST816S_SWIPE_LEFT = 0x03,
    CST816S_SWIPE_RIGHT = 0x04,
    CST816S_SINGLE_CLICK = 0x05,
    CST816S_DOUBLE_CLICK = 0x0B,
    CST816S_LONG_PRESS = 0x0C,
};

struct cst816s_info {
        uint8_t version;
        uint8_t version_info[3];

        uint8_t gesture;
        uint8_t event;
        uint8_t point;

        int x;
        int y;
};

struct cst816s_priv {
        struct device *dev;
        struct i2c_client *client;
        struct gpio_desc *reset;
        struct workqueue_struct *wq;
        struct input_dev *input;
        struct mutex lock;
        struct work_struct work;
        struct touchscreen_properties prop;
        struct cst816s_info info;

        u8 rxtx[8];

        int irq;
};

struct cst816s_gesture_mapping {
    enum cst816_gesture_id gesture_id;
    int event_code;
};

static const struct cst816s_gesture_mapping cst816s_gesture_map[] = {
    {CST816S_SWIPE_UP, KEY_UP},
    {CST816S_SWIPE_DOWN, KEY_DOWN},
    {CST816S_SWIPE_LEFT, KEY_LEFT},
    {CST816S_SWIPE_RIGHT, KEY_RIGHT},
    {CST816S_SINGLE_CLICK, BTN_TOUCH},
    {CST816S_DOUBLE_CLICK, BTN_TOOL_DOUBLETAP},
    {CST816S_LONG_PRESS, BTN_TOOL_TRIPLETAP}
};

static const struct i2c_device_id cst816s_id[] = {
    { "cst816s", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, cst816s_id);

static int cst816s_i2c_read_register(struct cst816s_priv *priv, u8 register_addr)
{
        struct i2c_client *client;
        struct i2c_msg xfer[2];
        int rc;

        client = priv->client;

        xfer[0].addr = client->addr;
        xfer[0].flags = 0;
        xfer[0].len = 1;
        xfer[0].buf = &register_addr;

        xfer[1].addr = client->addr;
        xfer[1].flags = I2C_M_RD;
        xfer[1].len = sizeof(priv->rxtx);
        xfer[1].buf = priv->rxtx;

        rc = i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer));
        if (rc < 0) {
                dev_err(&client->dev, "i2c xfer err: %d\n", rc);
                rc = -EIO;
        }

        return rc == ARRAY_SIZE(xfer) ? 0 : -EIO;
}

static void report_gesture_event(struct cst816s_priv *priv,
                                 enum cst816_gesture_id gesture_id) {
        const struct cst816s_gesture_mapping *mapping;

        mapping = NULL;

        for (uint8_t i = 0; i < ARRAY_SIZE(cst816s_gesture_map); i++) {
                if (cst816s_gesture_map[i].gesture_id == gesture_id) {
                        mapping = &cst816s_gesture_map[i];
                        break;
                }
        }

        if (mapping) {
                input_report_key(priv->input, mapping->event_code, 1);
                input_sync(priv->input);
                input_report_key(priv->input, mapping->event_code, 0);
        } else {
                dev_warn(priv->dev, "unknown gesture: %d\n", gesture_id);
        }
}

static int cst816s_process_touch(struct cst816s_priv *priv)
{
        uint8_t *raw;
        int rc;

        rc = cst816s_i2c_read_register(priv, CST816S_GET_RAW_CMD);
        if (!rc) {
                raw = priv->rxtx;

                priv->info.gesture = raw[0];
                priv->info.point = raw[1];
                priv->info.event = raw[2] >> 6;
                priv->info.x = ((raw[2] & 0x0f) << 8) + raw[3];
                priv->info.y = ((raw[4] & 0x0f) << 8) + raw[5];

                dev_warn(priv->dev, "x: %d, y: %d, event: %d point %d, gesture: 0x%x",
                         priv->info.x, priv->info.y,
                         priv->info.event, priv->info.point, priv->info.gesture);
        } else {
                dev_warn(priv->dev, "request was dropped\n");
        }

        return rc;
}

static int cst816s_register_input(struct cst816s_priv *priv)
{
        int rc;

        priv->input = devm_input_allocate_device(priv->dev);
        if (!priv->input) {
                dev_err(priv->dev, "input device alloc err: %d\n", rc);
                rc = -ENOMEM;

                goto err;
        }

        priv->input->name = "CST816S Touchscreen";
        priv->input->phys = "input/ts";
        priv->input->id.bustype = BUS_I2C;
        input_set_drvdata(priv->input, priv);

        for (uint8_t i = 0; i < ARRAY_SIZE(cst816s_gesture_map); i++) {
                input_set_capability(priv->input, EV_KEY,
                                     cst816s_gesture_map[i].event_code);
        }

        input_set_abs_params(priv->input, ABS_MT_POSITION_X,
                             0, CST816S_MAX_X, 0, 0);
        input_set_abs_params(priv->input, ABS_MT_POSITION_Y,
                             0, CST816S_MAX_Y, 0, 0);

        rc = input_mt_init_slots(priv->input, 1, INPUT_MT_DIRECT);
        if (rc) {
                dev_err(priv->dev, "failed to init input slots: %d\n", rc);

                goto err;
        }

        rc = input_register_device(priv->input);
        if (rc) {
                dev_err(priv->dev, "input registration err: %d\n", rc);

                goto err;
        }

err:
        return rc;
}

static void wq_cb(struct work_struct *work)
{
        struct cst816s_priv *priv = container_of(work, struct cst816s_priv, work);

        mutex_lock(&priv->lock);

        if (!cst816s_process_touch(priv)) {
                if (priv->info.gesture == CST816S_NONE) {
                        touchscreen_report_pos(priv->input, &priv->prop,
                                               priv->info.x, priv->info.y,
                                               true);
                } else {
                        report_gesture_event(priv, priv->info.gesture);
                }
        }

        mutex_unlock(&priv->lock);
}

static irqreturn_t cst815s_irq_cb(int irq, void *cookie)
{
        struct cst816s_priv *priv = (struct cst816s_priv *)cookie;

        queue_work(priv->wq, &priv->work);

        return IRQ_HANDLED;
}

static int cst816s_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
        struct cst816s_priv *priv;
        struct device *dev = &client->dev;
        int rc;

        priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
        if (!priv) {
                rc = -ENOMEM;
                dev_err(dev, "devm alloc failed: %d\n", rc);

                goto err;
        }

        priv->wq = create_workqueue("cst816s-wq");
        if (!priv->wq) {
                rc = -ENOMEM;
                dev_err(dev, "workqueue alloc failed: %d\n", rc);

                goto err;
        }

        mutex_init(&priv->lock);
        INIT_WORK(&priv->work, wq_cb);

        priv->dev = dev;
        priv->client = client;

        priv->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
        if (priv->reset == NULL) {
                dev_err(dev, "reset GPIO request failed\n");
                rc = -EIO;

                goto destroy_wq;
        }

        if (priv->reset) {
                gpiod_set_value_cansleep(priv->reset, GPIOD_OUT_LOW);
                msleep(20);
                gpiod_set_value_cansleep(priv->reset, GPIOD_OUT_HIGH);
        }

        rc = cst816s_register_input(priv);
        if (rc) {
                goto destroy_wq;
        }

        client->irq = of_irq_get(dev->of_node, 0);
        if (client->irq <= 0) {
                rc = -EINVAL;
                dev_err(dev, "get parent IRQ err: %d\n", rc);

                goto destroy_wq;
        }

        if (client->irq > 0) {
                rc = devm_request_irq(dev, client->irq,
                                      cst815s_irq_cb,
                                      IRQF_TRIGGER_RISING,
                                      dev->driver->name, priv);
                if (rc) {
                        dev_err(dev, "IRQ probe err: %d\n", client->irq);

                        goto free_input;
                }

                priv->irq = client->irq;
        } else {
                dev_warn(dev, "no IRQ will use for cst816s\n");
        }

        if (cst816s_i2c_read_register(priv, CST816S_GET_VERSION_CMD) == 0) {
                memcpy(&priv->info.version, priv->rxtx, sizeof(priv->info.version));
        } else {
                goto free_input;
        }

        if (cst816s_i2c_read_register(priv, CST816S_GET_VERSION_INFO_CMD) == 0) {
                memcpy(priv->info.version_info, priv->rxtx, ARRAY_SIZE(priv->info.version_info));
        } else {
                goto free_input;
        }

        dev_info(dev, "ts attached, version: %u, version info: %u.%u.%u",
                 priv->info.version,
                 priv->info.version_info[2],
                 priv->info.version_info[1],
                 priv->info.version_info[0]);

free_input:
        if (rc) {
                input_free_device(priv->input);
        }

destroy_wq:
        if (rc) {
                destroy_workqueue(priv->wq);
        }
err:
        return rc;
}

static const struct of_device_id cst816s_of_match[] = {
    { .compatible = "cst,cst816s", },
    { }
};
MODULE_DEVICE_TABLE(of, cst816s_of_match);

static struct i2c_driver cst816s_driver = {
        .driver = {
                .name = "cst816s",
                .of_match_table = cst816s_of_match,
        },
        .id_table = cst816s_id,
        .probe = cst816s_probe
};

module_i2c_driver(cst816s_driver);

MODULE_AUTHOR("Oleh Kuzhylnyi <kuzhylol@gmail.com>");
MODULE_DESCRIPTION("CST816S Touchscreen Driver");
MODULE_LICENSE("GPL v2");
