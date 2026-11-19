// SPDX-License-Identifier: GPL-2.0-only
/*
 * Backlight driver for SG Micro SGM37604A 4-channel WLED driver.
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>

#define SGM37604A_REG_LED_EN		0x10
#define SGM37604A_REG_MODE		0x11
#define SGM37604A_REG_BRT_MSB		0x19
#define SGM37604A_REG_BRT_LSB		0x1a

#define SGM37604A_LED_ALL_EN		0x1f
#define SGM37604A_MODE_I2C_ONLY		0x05
#define SGM37604A_MAX_BRIGHTNESS	4095
#define SGM37604A_DEF_BRIGHTNESS	3072

struct sgm37604a_bl {
	struct i2c_client *client;
	struct gpio_desc *enable_gpio;
	struct backlight_device *bl;
};

static int sgm37604a_write(struct i2c_client *client, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(client, reg, val);
}

static int sgm37604a_update_status(struct backlight_device *bl)
{
	struct sgm37604a_bl *sgm = bl_get_data(bl);
	int brightness = backlight_get_brightness(bl);
	int ret;

	if (brightness <= 0) {
		ret = sgm37604a_write(sgm->client, SGM37604A_REG_BRT_MSB, 0);
		if (ret < 0)
			return ret;
		ret = sgm37604a_write(sgm->client, SGM37604A_REG_BRT_LSB, 0);
		if (ret < 0)
			return ret;
		return sgm37604a_write(sgm->client, SGM37604A_REG_LED_EN, 0x00);
	}

	if (sgm->enable_gpio) {
		gpiod_set_value_cansleep(sgm->enable_gpio, 1);
		usleep_range(1000, 2000);
	}

	ret = sgm37604a_write(sgm->client, SGM37604A_REG_MODE,
			      SGM37604A_MODE_I2C_ONLY);
	if (ret < 0)
		return ret;

	ret = sgm37604a_write(sgm->client, SGM37604A_REG_LED_EN,
			      SGM37604A_LED_ALL_EN);
	if (ret < 0)
		return ret;

	ret = sgm37604a_write(sgm->client, SGM37604A_REG_BRT_MSB,
			      (brightness >> 4) & 0xff);
	if (ret < 0)
		return ret;

	return sgm37604a_write(sgm->client, SGM37604A_REG_BRT_LSB,
			       brightness & 0x0f);
}

static const struct backlight_ops sgm37604a_bl_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = sgm37604a_update_status,
};

static int sgm37604a_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct backlight_properties props = {};
	struct sgm37604a_bl *sgm;
	u32 def_brt = SGM37604A_DEF_BRIGHTNESS;

	sgm = devm_kzalloc(dev, sizeof(*sgm), GFP_KERNEL);
	if (!sgm)
		return -ENOMEM;

	sgm->client = client;
	sgm->enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(sgm->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(sgm->enable_gpio),
				     "failed to get enable gpio\n");

	of_property_read_u32(dev->of_node, "default-brightness-level", &def_brt);
	if (def_brt > SGM37604A_MAX_BRIGHTNESS)
		def_brt = SGM37604A_MAX_BRIGHTNESS;

	props.type = BACKLIGHT_RAW;
	props.max_brightness = SGM37604A_MAX_BRIGHTNESS;
	props.brightness = def_brt;

	sgm->bl = devm_backlight_device_register(dev, dev_name(dev), dev, sgm,
						 &sgm37604a_bl_ops, &props);
	if (IS_ERR(sgm->bl))
		return dev_err_probe(dev, PTR_ERR(sgm->bl),
				     "failed to register backlight\n");

	i2c_set_clientdata(client, sgm);
	return backlight_update_status(sgm->bl);
}

static const struct of_device_id sgm37604a_of_match[] = {
	{ .compatible = "sgmicro,sgm37604a" },
	{ }
};
MODULE_DEVICE_TABLE(of, sgm37604a_of_match);

static const struct i2c_device_id sgm37604a_id[] = {
	{ "sgm37604a" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sgm37604a_id);

static struct i2c_driver sgm37604a_driver = {
	.driver = {
		.name = "sgm37604a",
		.of_match_table = sgm37604a_of_match,
	},
	.probe = sgm37604a_probe,
	.id_table = sgm37604a_id,
};
module_i2c_driver(sgm37604a_driver);

MODULE_DESCRIPTION("SG Micro SGM37604A Backlight Driver");
MODULE_LICENSE("GPL");
