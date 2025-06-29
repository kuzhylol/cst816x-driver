// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for I2C connected Hynitron CST816x Touchscreen
 *
 * Copyright (C) 2025 Oleh Kuzhylnyi <kuzhylol@gmail.com>
 */
#include <asm/unaligned.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>

#define CST816X_NUM_KEYS 16
#define CST816X_RD_REG 0x01
#define CST816X_TOUCH 0x00

struct cst816x_touch_desc {
	u8 gesture;
	u8 touch;
	__be16 abs_x;
	__be16 abs_y;
} __packed;

struct cst816x_priv {
	struct i2c_client *client;
	struct gpio_desc *reset;
	struct input_dev *input;
	unsigned int keycode[CST816X_NUM_KEYS];
	unsigned int keycodemax;
};

static int cst816x_parse_keycodes(struct device *dev, struct cst816x_priv *priv)
{
	int ret;

	ret = device_property_count_u32(dev, "linux,keycodes");
	if (ret > ARRAY_SIZE(priv->keycode)) {
		dev_err(dev, "Too many keycodes: %d\n", ret);
		return -EINVAL;
	}
	priv->keycodemax = ret;

	ret = device_property_read_u32_array(dev, "linux,keycodes",
					     priv->keycode, priv->keycodemax);
	if (ret) {
		dev_err(dev, "Failed to read keycodes: %d\n", ret);
		return ret;
	}

	return 0;
}

static int cst816x_i2c_read_register(struct cst816x_priv *priv, u8 reg,
				     void *buf, size_t len)
{
	int ret;
	struct i2c_msg xfer[] = {
		{
			.addr = priv->client->addr,
			.flags = 0,
			.buf = &reg,
			.len = sizeof(reg),
		},
		{
			.addr = priv->client->addr,
			.flags = I2C_M_RD,
			.buf = buf,
			.len = len,
		},
	};

	ret = i2c_transfer(priv->client->adapter, xfer, ARRAY_SIZE(xfer));
	if (ret != ARRAY_SIZE(xfer)) {
		ret = ret < 0 ? ret : -EIO;
		dev_err(&priv->client->dev, "i2c rx err: %d\n", ret);
		return ret;
	}

	return 0;
}

static bool cst816x_process_touch(struct cst816x_priv *priv,
				  struct cst816x_touch_desc *desc)
{
	if (cst816x_i2c_read_register(priv, CST816X_RD_REG, desc, sizeof(*desc)))
		return false;

	desc->abs_x = get_unaligned_be16(&desc->abs_x) & GENMASK(11, 0);
	desc->abs_y = get_unaligned_be16(&desc->abs_y) & GENMASK(11, 0);

	dev_dbg(&priv->client->dev, "x: %u, y: %u, t: %u, g: 0x%x\n",
		desc->abs_x, desc->abs_y, desc->touch, desc->gesture);

	return true;
}

static int cst816x_register_input(struct cst816x_priv *priv)
{
	priv->input = devm_input_allocate_device(&priv->client->dev);
	if (!priv->input)
		return -ENOMEM;

	priv->input->name = "Hynitron CST816x Series Touchscreen";
	priv->input->phys = "input/ts";
	priv->input->id.bustype = BUS_I2C;
	input_set_drvdata(priv->input, priv);

	input_set_abs_params(priv->input, ABS_X, 0, 240, 0, 0);
	input_set_abs_params(priv->input, ABS_Y, 0, 240, 0, 0);

	for (int i = 0; i < priv->keycodemax; i++) {
		if (priv->keycode[i] == KEY_RESERVED)
			continue;

		input_set_capability(priv->input, EV_KEY, priv->keycode[i]);
	}

	return input_register_device(priv->input);
}

static void cst816x_reset(struct cst816x_priv *priv)
{
	gpiod_set_value_cansleep(priv->reset, 1);
	msleep(50);
	gpiod_set_value_cansleep(priv->reset, 0);
	msleep(100);
}

static irqreturn_t cst816x_irq_cb(int irq, void *cookie)
{
	struct cst816x_priv *priv = cookie;
	struct cst816x_touch_desc desc;

	if (!cst816x_process_touch(priv, &desc))
		return IRQ_HANDLED;

	if (desc.touch) {
		input_report_key(priv->input, priv->keycode[CST816X_TOUCH], 1);
		input_report_abs(priv->input, ABS_X, desc.abs_x);
		input_report_abs(priv->input, ABS_Y, desc.abs_y);
	}

	if (desc.gesture) {
		input_report_key(priv->input, priv->keycode[desc.gesture & 0x0F],
				 desc.touch);

		if (!desc.touch)
			input_report_key(priv->input,
					 priv->keycode[CST816X_TOUCH], 0);
	}

	input_sync(priv->input);

	return IRQ_HANDLED;
}

static int cst816x_probe(struct i2c_client *client)
{
	struct cst816x_priv *priv;
	struct device *dev = &client->dev;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->client = client;

	priv->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->reset))
		return dev_err_probe(dev, PTR_ERR(priv->reset),
				     "gpio reset request failed\n");

	if (priv->reset)
		cst816x_reset(priv);

	ret = cst816x_parse_keycodes(dev, priv);
	if (ret)
		dev_warn(dev, "no gestures found in dt\n");

	ret = cst816x_register_input(priv);
	if (ret)
		return dev_err_probe(dev, ret, "input register failed\n");

	ret = devm_request_threaded_irq(dev, client->irq, NULL, cst816x_irq_cb,
					IRQF_ONESHOT, dev->driver->name, priv);
	if (ret)
		return dev_err_probe(dev, ret, "irq request failed\n");

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
MODULE_DESCRIPTION("Hynitron CST816x Series Touchscreen Driver");
MODULE_LICENSE("GPL");
