// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for I2C connected CST816X touchsreen
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
#include <linux/timer.h>

#define CST816X_MAX_X 240
#define CST816X_MAX_Y CST816X_MAX_X
#define CST816X_EVENT_TIMEOUT_MS 50

enum cst816x_commands {
        CST816X_SET_DOUBLE_TAP = 0x01,
        CST816X_SET_UP_DOWN_SWIPE = 0x02,
        CST816X_SET_LEFT_RIGHT_SWIPE = 0x04,
        CST816X_SET_STANDBY_MODE = 0x03,
        CST816X_SET_GESTURE_MODE = 0x41,
        CST816X_SET_POINT_MODE = 0x71,
        CST816X_SET_MIXED_MODE = 0x11,
        CST816X_SET_CYCLE_UNIT = 0x01, // 0.1ms, range: 1 - 30ms
        CST816X_SET_PULSE_WIDTH_UNIT = 0x0A, // 10ms, range: 0 - 200ms
};

enum cst816x_registers {
        CST816X_FRAME = 0x01,
        CST816X_VERSION = 0xA7,
        CST816X_MOTION = 0xEC,
        CST816X_STANDBY = 0xA5,
        CST816X_IRQCTL = 0xFA,
        CST816X_CYCLE = 0xEE,
        CST816X_PULSE_WIDTH = 0xED,
};

enum cst816_gesture_id {
        CST816X_NONE = 0x00,
        CST816X_SWIPE_UP = 0x01,
        CST816X_SWIPE_DOWN = 0x02,
        CST816X_SWIPE_LEFT = 0x03,
        CST816X_SWIPE_RIGHT = 0x04,
        CST816X_SINGLE_TAP = 0x05,
        CST816X_DOUBLE_TAP = 0x0B,
        CST816X_LONG_PRESS = 0x0C,
};

struct cst816x_info {
        uint8_t version[3];

        uint8_t gesture;
        uint8_t x;
        uint8_t y;
};

struct cst816x_priv {
        struct device *dev;
        struct i2c_client *client;
        struct gpio_desc *reset;
        struct workqueue_struct *wq;
        struct input_dev *input;
        struct mutex lock;
        struct timer_list timer;
        struct work_struct work;
        struct cst816x_info info;

        u8 rxtx[8];

        int irq;
};

struct cst816x_gesture_mapping {
        enum cst816_gesture_id gesture_id;
        int event_code;
};

static const struct cst816x_gesture_mapping cst816x_gesture_map[] = {
        {CST816X_SWIPE_UP, KEY_UP},
        {CST816X_SWIPE_DOWN, KEY_DOWN},
        {CST816X_SWIPE_LEFT, KEY_LEFT},
        {CST816X_SWIPE_RIGHT, KEY_RIGHT},
        {CST816X_SINGLE_TAP, BTN_TOUCH},
        {CST816X_DOUBLE_TAP, BTN_TOOL_DOUBLETAP},
        {CST816X_LONG_PRESS, BTN_TOOL_TRIPLETAP}
};

static const struct i2c_device_id cst816x_id[] = {
        { "cst816s", 0 },
        { }
};
MODULE_DEVICE_TABLE(i2c, cst816x_id);

static int cst816x_i2c_reg_write(struct cst816x_priv *priv, u8 reg, u8 cmd)
{
        struct i2c_client *client;
        struct i2c_msg xfer;
        int rc;

        client = priv->client;

        priv->rxtx[0] = reg;
        priv->rxtx[1] = cmd;

        xfer.addr = client->addr;
        xfer.flags = 0;
        xfer.len = 2;
        xfer.buf = priv->rxtx;

        rc = i2c_transfer(client->adapter, &xfer , 1);
        if (rc < 0) {
                dev_err(&client->dev, "i2c tx err: %d\n", rc);
                rc = -EIO;
        }

        return rc;
}

static int cst816x_i2c_reg_read(struct cst816x_priv *priv, u8 reg)
{
        struct i2c_client *client;
        struct i2c_msg xfer[2];
        int rc;

        client = priv->client;

        xfer[0].addr = client->addr;
        xfer[0].flags = 0;
        xfer[0].len = sizeof(reg);
        xfer[0].buf = &reg;

        xfer[1].addr = client->addr;
        xfer[1].flags = I2C_M_RD;
        xfer[1].len = sizeof(priv->rxtx);
        xfer[1].buf = priv->rxtx;

        rc = i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer));
        if (rc < 0) {
                dev_err(&client->dev, "i2c rx err: %d\n", rc);
                rc = -EIO;
        }

        return rc == ARRAY_SIZE(xfer) ? 0 : -EIO;
}

static int cst816x_setup_regs(struct cst816x_priv *priv)
{
        int rc;

        rc = cst816x_i2c_reg_write(priv, CST816X_IRQCTL, CST816X_SET_MIXED_MODE);
        if (rc < 0) {
                goto err;
        }

        rc = cst816x_i2c_reg_write(priv, CST816X_CYCLE, CST816X_SET_CYCLE_UNIT);
        if (rc < 0) {
                goto err;
        }

        rc = cst816x_i2c_reg_write(priv, CST816X_PULSE_WIDTH,
                                   CST816X_SET_PULSE_WIDTH_UNIT);
        if (rc < 0) {
                goto err;
        }

        rc = cst816x_i2c_reg_write(priv, CST816X_MOTION, CST816X_DOUBLE_TAP);
        if (rc < 0) {
                goto err;
        }

err:
        if (rc < 0) {
                dev_err(priv->dev, "register setup err: %d\n", rc);
        } else {
                rc = 0;
        }

        return rc;
}

static void report_gesture_event(struct cst816x_priv *priv,
                                 enum cst816_gesture_id gesture_id,
                                 bool state) {
        const struct cst816x_gesture_mapping *mapping;

        mapping = NULL;

        for (uint8_t i = 0; i < ARRAY_SIZE(cst816x_gesture_map); i++) {
                if (cst816x_gesture_map[i].gesture_id == gesture_id) {
                        mapping = &cst816x_gesture_map[i];
                        break;
                }
        }

        if (mapping) {
                input_report_key(priv->input, mapping->event_code, state);
        } else {
                dev_warn(priv->dev, "unknown gesture: %d\n", gesture_id);
        }
}

static int cst816x_process_touch(struct cst816x_priv *priv)
{
        uint8_t *raw;
        int rc;

        rc = cst816x_i2c_reg_read(priv, CST816X_FRAME);
        if (!rc) {
                raw = priv->rxtx;

                priv->info.gesture = raw[0];
                priv->info.x = ((raw[2] & 0x0f) << 8) + raw[3];
                priv->info.y = ((raw[4] & 0x0f) << 8) + raw[5];

                dev_dbg(priv->dev, "x: %d, y: %d, gesture: 0x%x\n",
                        priv->info.x, priv->info.y, priv->info.gesture);
        } else {
                dev_warn(priv->dev, "request was dropped\n");
        }

        return rc;
}

static int cst816x_register_input(struct cst816x_priv *priv)
{
        int rc;

        priv->input = devm_input_allocate_device(priv->dev);
        if (!priv->input) {
                dev_err(priv->dev, "input device alloc err: %d\n", rc);
                rc = -ENOMEM;

                goto err;
        }

        priv->input->name = "CST816X Touchscreen";
        priv->input->phys = "input/ts";
        priv->input->id.bustype = BUS_I2C;
        input_set_drvdata(priv->input, priv);

        for (uint8_t i = 0; i < ARRAY_SIZE(cst816x_gesture_map); i++) {
                input_set_capability(priv->input, EV_KEY,
                                     cst816x_gesture_map[i].event_code);
        }

        input_set_abs_params(priv->input, ABS_X, 0, CST816X_MAX_X, 0, 0);
        input_set_abs_params(priv->input, ABS_Y, 0, CST816X_MAX_Y, 0, 0);
        input_set_capability(priv->input, EV_ABS, ABS_X);
        input_set_capability(priv->input, EV_ABS, ABS_Y);

        rc = input_register_device(priv->input);
        if (rc) {
                dev_err(priv->dev, "input registration err: %d\n", rc);

                goto err;
        }
err:
        return rc;
}

static void cst816x_reset(struct cst816x_priv *priv)
{
        gpiod_set_value_cansleep(priv->reset, 0);
        msleep(20);
        gpiod_set_value_cansleep(priv->reset, 1);
        msleep(20);
}

static void cst816x_event_release(struct timer_list *t)
{
        struct cst816x_priv *priv = from_timer(priv, t, timer);

        mutex_lock(&priv->lock);

        report_gesture_event(priv, priv->info.gesture, false);
        input_sync(priv->input);

        mutex_unlock(&priv->lock);
}

static void wq_cb(struct work_struct *work)
{
        struct cst816x_priv *priv =
                container_of(work, struct cst816x_priv, work);

        mutex_lock(&priv->lock);

        if (!cst816x_process_touch(priv)) {
                input_report_abs(priv->input, ABS_X, priv->info.x);
                input_report_abs(priv->input, ABS_Y, priv->info.y);

                if (priv->info.gesture != CST816X_NONE)
                        report_gesture_event(priv, priv->info.gesture, true);

                input_sync(priv->input);
        }

        mod_timer(&priv->timer,
                  jiffies + msecs_to_jiffies(CST816X_EVENT_TIMEOUT_MS));

        mutex_unlock(&priv->lock);
}

static irqreturn_t cst815s_irq_cb(int irq, void *cookie)
{
        struct cst816x_priv *priv = (struct cst816x_priv *)cookie;

        queue_work(priv->wq, &priv->work);

        return IRQ_HANDLED;
}

static int cst816x_suspend(struct device *dev)
{
        struct cst816x_priv *priv = i2c_get_clientdata(to_i2c_client(dev));

        flush_work(&priv->work);
        disable_irq(priv->irq);
        del_timer_sync(&priv->timer);

        return cst816x_i2c_reg_write(priv, CST816X_STANDBY,
                                     CST816X_SET_STANDBY_MODE);
}

static int cst816x_resume(struct device *dev)
{
        struct cst816x_priv *priv = i2c_get_clientdata(to_i2c_client(dev));

        cst816x_reset(priv);
        cst816x_setup_regs(priv);

        enable_irq(priv->irq);

        return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(cst816x_pm_ops, cst816x_suspend, cst816x_resume);

static int cst816x_probe(struct i2c_client *client,
                         const struct i2c_device_id *id)
{
        struct cst816x_priv *priv;
        struct device *dev = &client->dev;
        int rc;

        priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
        if (!priv) {
                rc = -ENOMEM;
                dev_err(dev, "devm alloc failed: %d\n", rc);

                goto err;
        }

        priv->wq = create_workqueue("cst816x-wq");
        if (!priv->wq) {
                rc = -ENOMEM;
                dev_err(dev, "workqueue alloc failed: %d\n", rc);

                goto err;
        }

        mutex_init(&priv->lock);
        INIT_WORK(&priv->work, wq_cb);
        timer_setup(&priv->timer, cst816x_event_release, 0);

        priv->dev = dev;
        priv->client = client;

        priv->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
        if (priv->reset == NULL) {
                dev_err(dev, "reset GPIO request failed\n");
                rc = -EIO;

                goto destroy_wq;
        }

        if (priv->reset) {
                cst816x_reset(priv);
        }

        rc = cst816x_setup_regs(priv);
        if (rc < 0) {
                goto destroy_wq;
        }

        rc = cst816x_register_input(priv);
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
                rc = devm_request_threaded_irq(dev, client->irq, NULL,
                                               cst815s_irq_cb,
                                               IRQF_ONESHOT | IRQF_NO_AUTOEN,
                                               dev->driver->name, priv);
                if (rc) {
                        dev_err(dev, "IRQ probe err: %d\n", client->irq);

                        goto free_input;
                }

                priv->irq = client->irq;
        } else {
                dev_warn(dev, "no IRQ will use for cst816x\n");
        }

        if (cst816x_i2c_reg_read(priv, CST816X_VERSION) == 0) {
                memcpy(priv->info.version, priv->rxtx,
                       ARRAY_SIZE(priv->info.version));
        } else {
                goto free_input;
        }

        enable_irq(priv->irq);

        dev_info(dev, "touchscreen attached, version: %u.%u.%u",
                 priv->info.version[2], priv->info.version[1],
                 priv->info.version[0]);

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

static const struct of_device_id cst816x_of_match[] = {
        { .compatible = "cst,cst816s", },
        { }
};
MODULE_DEVICE_TABLE(of, cst816x_of_match);

static struct i2c_driver cst816x_driver = {
        .driver = {
                .name = "cst816x",
                .of_match_table = cst816x_of_match,
                .pm = pm_sleep_ptr(&cst816x_pm_ops),
        },
        .id_table = cst816x_id,
        .probe = cst816x_probe
};

module_i2c_driver(cst816x_driver);

MODULE_AUTHOR("Oleh Kuzhylnyi <kuzhylol@gmail.com>");
MODULE_DESCRIPTION("CST816X Touchscreen Driver");
MODULE_LICENSE("GPL v2");
