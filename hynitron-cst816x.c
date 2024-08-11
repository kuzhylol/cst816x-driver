// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for I2C connected Hynitron CST816X Touchscreen
 *
 * Copyright (C) 2024 Oleh Kuzhylnyi <kuzhylol@gmail.com>
 */
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>

enum cst816x_registers {
	CST816X_FRAME = 0x01,
	CST816X_MOTION = 0xEC,
};

enum cst816x_gestures {
	CST816X_SWIPE_UP = 0x01,
	CST816X_SWIPE_DOWN = 0x02,
	CST816X_SWIPE_LEFT = 0x03,
	CST816X_SWIPE_RIGHT = 0x04,
	CST816X_SINGLE_TAP = 0x05,
	CST816X_LONG_PRESS = 0x0C,
};

struct cst816x_touch_info {
	u8 gesture;
	u8 touch;
	u16 abs_x;
	u16 abs_y;
};

struct cst816x_priv {
	struct device *dev;
	struct i2c_client *client;
	struct gpio_desc *reset;
	struct input_dev *input;
};

struct cst816x_event_mapping {
	enum cst816x_gestures gesture;
	u16 code;
};

static const struct cst816x_event_mapping event_map[16] = {
	{CST816X_SWIPE_UP, BTN_FORWARD},
        {CST816X_SWIPE_DOWN, BTN_BACK},
        {CST816X_SWIPE_LEFT, BTN_LEFT},
        {CST816X_SWIPE_RIGHT, BTN_RIGHT},
        {CST816X_SINGLE_TAP, BTN_TOUCH},
        {CST816X_LONG_PRESS, BTN_TOOL_TRIPLETAP},
        {KEY_RESERVED, KEY_RESERVED},
        {KEY_RESERVED, KEY_RESERVED},
        {KEY_RESERVED, KEY_RESERVED},
        {KEY_RESERVED, KEY_RESERVED},
        {KEY_RESERVED, KEY_RESERVED},
        {KEY_RESERVED, KEY_RESERVED},
        {KEY_RESERVED, KEY_RESERVED},
        {KEY_RESERVED, KEY_RESERVED},
        {KEY_RESERVED, KEY_RESERVED},
        {KEY_RESERVED, KEY_RESERVED}
};

static int cst816x_i2c_read_register(struct cst816x_priv *priv, u8 reg,
				     void *buf, size_t len)
{
	struct i2c_client *client;
	struct i2c_msg xfer[2];
	int rc;

	client = priv->client;

	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].buf = &reg;
	xfer[0].len = sizeof(reg);

	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].buf = buf;
	xfer[1].len = len;

	rc = i2c_transfer(client->adapter, xfer, ARRAY_SIZE(xfer));
	if (rc != ARRAY_SIZE(xfer)) {
		if (rc >= 0)
			rc = -EIO;
	} else {
		rc = 0;
	}

	if (rc < 0)
		dev_err(&client->dev, "i2c rx err: %d\n", rc);

	return rc;
}

static int cst816x_process_touch(struct cst816x_priv *priv,
				 struct cst816x_touch_info *info)
{
	u8 raw[8];
	int rc;

	rc = cst816x_i2c_read_register(priv, CST816X_FRAME, raw, sizeof(raw));
	if (!rc) {
		info->gesture = raw[0];
		info->touch = raw[1];
		info->abs_x = ((raw[2] & 0x0F) << 8) | raw[3];
		info->abs_y = ((raw[4] & 0x0F) << 8) | raw[5];

		dev_dbg(priv->dev, "x: %d, y: %d, t: %d, g: 0x%x\n",
			info->abs_x, info->abs_y, info->touch, info->gesture);
	}

	return rc;
}

static int cst816x_register_input(struct cst816x_priv *priv)
{
	priv->input = devm_input_allocate_device(priv->dev);
	if (!priv->input)
		return -ENOMEM;

	priv->input->name = "Hynitron CST816X Touchscreen";
	priv->input->phys = "input/ts";
	priv->input->id.bustype = BUS_I2C;
	input_set_drvdata(priv->input, priv);

	for (unsigned int i = 0; i < ARRAY_SIZE(event_map); i++) {
		input_set_capability(priv->input, EV_KEY, event_map[i].code);
	}

	input_set_abs_params(priv->input, ABS_X, 0, 240, 0, 0);
	input_set_abs_params(priv->input, ABS_Y, 0, 240, 0, 0);

	return input_register_device(priv->input);
}

static void cst816x_reset(struct cst816x_priv *priv)
{
	gpiod_set_value_cansleep(priv->reset, 1);
	msleep(50);
	gpiod_set_value_cansleep(priv->reset, 0);
	msleep(100);
}

static void report_gesture_event(const struct cst816x_priv *priv,
				 enum cst816x_gestures gesture, bool touch)
{
	u16 key = event_map[gesture & 0x0F].code;
	if (key != KEY_RESERVED)
		input_report_key(priv->input, key, touch);

	if (!touch)
		input_report_key(priv->input, BTN_TOUCH, 0);
}

/*
 * Supports five gestures: TOUCH, LEFT, RIGHT, FORWARD, BACK, and LONG_PRESS.
 * Reports surface interaction, sliding coordinates and finger detachment.
 *
 * 1. TOUCH Gesture Scenario:
 *
 * [x/y] [touch] [gesture] [Action] [Report ABS] [Report Key]
 *  x y   true    0x00      Touch    ABS_X_Y      BTN_TOUCH
 *  x y   true    0x00      Slide    ABS_X_Y
 *  x y   false   0x05      Gesture               BTN_TOUCH
 *
 * 2. LEFT, RIGHT, FORWARD, BACK, and LONG_PRESS Gestures Scenario:
 *
 * [x/y] [touch] [gesture] [Action] [Report ABS] [Report Key]
 *  x y   true    0x00      Touch    ABS_X_Y      BTN_TOUCH
 *  x y   true    0x01      Gesture  ABS_X_Y      BTN_FORWARD
 *  x y   true    0x01      Slide    ABS_X_Y
 *  x y   false   0x01      Detach                BTN_FORWARD | BTN_TOUCH
 */
static irqreturn_t cst816x_irq_cb(int irq, void *cookie)
{
	struct cst816x_priv *priv = (struct cst816x_priv *)cookie;
	struct cst816x_touch_info info;

	if (!cst816x_process_touch(priv, &info)) {
		if (info.touch) {
			input_report_abs(priv->input, ABS_X, info.abs_x);
			input_report_abs(priv->input, ABS_Y, info.abs_y);
			input_report_key(priv->input, BTN_TOUCH, 1);
		}

		if (info.gesture)
			report_gesture_event(priv, info.gesture, info.touch);

		input_sync(priv->input);
	}

	return IRQ_HANDLED;
}

static int cst816x_probe(struct i2c_client *client)
{
	struct cst816x_priv *priv;
	struct device *dev = &client->dev;
	int rc;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->client = client;

	priv->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->reset))
		return dev_err_probe(dev, PTR_ERR(priv->reset),
				     "reset gpio not found\n");

	cst816x_reset(priv);

	rc = cst816x_register_input(priv);
	if (rc)
		return dev_err_probe(dev, rc, "input register failed\n");

	rc = devm_request_threaded_irq(dev, client->irq, NULL, cst816x_irq_cb,
				       IRQF_ONESHOT, dev->driver->name, priv);
	if (rc)
		return dev_err_probe(dev, rc, "irq request failed\n");

	return 0;
}

static const struct i2c_device_id cst816x_id[] = {
	{ .name = "cst816s", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, cst816x_id);

static const struct of_device_id cst816x_of_match[] = {
	{ .compatible = "hynitron,cst816s", },
	{ }
};
MODULE_DEVICE_TABLE(of, cst816x_of_match);

static struct i2c_driver cst816x_driver = {
	.driver = {
		.name = "cst816x",
		.of_match_table = cst816x_of_match,
	},
	.id_table = cst816x_id,
	.probe = cst816x_probe,
};

module_i2c_driver(cst816x_driver);

MODULE_AUTHOR("Oleh Kuzhylnyi <kuzhylol@gmail.com>");
MODULE_DESCRIPTION("Hynitron CST816X Touchscreen Driver");
MODULE_LICENSE("GPL");
