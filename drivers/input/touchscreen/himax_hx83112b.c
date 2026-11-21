// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for Himax hx83112b touchscreens
 *
 * Copyright (C) 2022 Job Noorman <job@noorman.info>
 *
 * HX83100A support
 * Copyright (C) 2024 Felix Kaechele <felix@kaechele.ca>
 *
 * This code is based on "Himax Android Driver Sample Code for QCT platform":
 *
 * Copyright (C) 2017 Himax Corporation.
 */

#include <drm/drm_panel.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>

#define HIMAX_MAX_POINTS		10

#define HIMAX_AHB_ADDR_BYTE_0			0x00
#define HIMAX_AHB_ADDR_RDATA_BYTE_0		0x08
#define HIMAX_AHB_ADDR_ACCESS_DIRECTION		0x0c
#define HIMAX_AHB_ADDR_INCR4			0x0d
#define HIMAX_AHB_ADDR_CONTI			0x13
#define HIMAX_AHB_ADDR_EVENT_STACK		0x30

#define HIMAX_AHB_CMD_ACCESS_DIRECTION_READ	0x00
#define HIMAX_AHB_CMD_INCR4			0x10
#define HIMAX_AHB_CMD_CONTI			0x31

#define HIMAX_REG_ADDR_FW_MODE_STATUS		0x10007088
#define HIMAX_REG_ADDR_FLASH_RELOAD2		0x100072c0
#define HIMAX_REG_ADDR_FLASH_RELOAD		0x10007f00
#define HIMAX_REG_ADDR_ICID			0x900000d0

#define HIMAX_FW_DATA_IDLE_DIS_PWD		0x17

#define HX83100A_REG_FW_EVENT_STACK		0x90060000

#define HIMAX_INVALID_COORD		0xffff

struct himax_event_point {
	__be16 x;
	__be16 y;
} __packed;

struct himax_event {
	struct himax_event_point points[HIMAX_MAX_POINTS];
	u8 majors[HIMAX_MAX_POINTS];
	u8 pad0[2];
	u8 num_points;
	u8 pad1[2];
	u8 checksum_fix;
} __packed;

static_assert(sizeof(struct himax_event) == 56);

struct himax_ts_data;
struct himax_chip {
	u32 id;
	unsigned int post_reset_ms;
	int (*check_id)(struct himax_ts_data *ts);
	int (*init)(struct himax_ts_data *ts);
	int (*read_events)(struct himax_ts_data *ts, struct himax_event *event,
			   size_t length);
};

struct himax_ts_data {
	const struct himax_chip *chip;
	struct gpio_desc *gpiod_rst;
	struct input_dev *input_dev;
	struct i2c_client *client;
	struct regmap *regmap;
	struct regulator *vdd;
	struct drm_panel_follower panel_follower;
	struct touchscreen_properties props;
	bool is_panel_follower;
	bool enabled;
};

static const struct regmap_config himax_regmap_config = {
	.reg_bits = 8,
	.val_bits = 32,
	.val_format_endian = REGMAP_ENDIAN_LITTLE,
};

static int himax_write_reg8(struct himax_ts_data *ts, u8 reg, u8 val)
{
	u8 buf[2] = { reg, val };
	int ret;

	ret = i2c_master_send(ts->client, buf, sizeof(buf));
	if (ret < 0)
		return ret;

	return ret == sizeof(buf) ? 0 : -EIO;
}

static int himax_bus_enable_burst(struct himax_ts_data *ts)
{
	int error;

	error = himax_write_reg8(ts, HIMAX_AHB_ADDR_CONTI,
				 HIMAX_AHB_CMD_CONTI);
	if (error)
		return error;

	error = himax_write_reg8(ts, HIMAX_AHB_ADDR_INCR4,
				 HIMAX_AHB_CMD_INCR4);
	if (error)
		return error;

	return 0;
}

static int himax_bus_read(struct himax_ts_data *ts, u32 address, void *dst,
			  size_t length)
{
	int error;

	if (length > 4) {
		error = himax_bus_enable_burst(ts);
		if (error)
			return error;
	}

	error = regmap_write(ts->regmap, HIMAX_AHB_ADDR_BYTE_0, address);
	if (error)
		return error;

	error = himax_write_reg8(ts, HIMAX_AHB_ADDR_ACCESS_DIRECTION,
				 HIMAX_AHB_CMD_ACCESS_DIRECTION_READ);
	if (error)
		return error;

	if (length > 4)
		error = regmap_noinc_read(ts->regmap, HIMAX_AHB_ADDR_RDATA_BYTE_0,
					  dst, length);
	else
		error = regmap_read(ts->regmap, HIMAX_AHB_ADDR_RDATA_BYTE_0,
				    dst);
	if (error)
		return error;

	return 0;
}

static int himax_bus_write(struct himax_ts_data *ts, u32 address, u32 value)
{
	__le32 data[2] = {
		cpu_to_le32(address),
		cpu_to_le32(value),
	};

	return regmap_raw_write(ts->regmap, HIMAX_AHB_ADDR_BYTE_0, data,
				sizeof(data));
}

static void himax_reset(struct himax_ts_data *ts)
{
	gpiod_set_value_cansleep(ts->gpiod_rst, 1);

	/* Delay copied from downstream driver */
	msleep(20);
	gpiod_set_value_cansleep(ts->gpiod_rst, 0);

	if (ts->chip->post_reset_ms)
		msleep(ts->chip->post_reset_ms);
	else
		usleep_range(1000, 1100);
}

static int himax_read_product_id(struct himax_ts_data *ts, u32 *product_id)
{
	int error;

	error = himax_bus_read(ts, HIMAX_REG_ADDR_ICID, product_id,
			       sizeof(*product_id));
	if (error)
		return error;

	*product_id >>= 8;
	return 0;
}

static int himax_check_product_id(struct himax_ts_data *ts)
{
	int error;
	u32 product_id;

	error = himax_read_product_id(ts, &product_id);
	if (error)
		return error;

	dev_info_once(&ts->client->dev, "Product id: %x\n", product_id);

	if (product_id == ts->chip->id)
		return 0;

	dev_err(&ts->client->dev, "Unknown product id: %x\n",
		product_id);
	return -EINVAL;
}

static int hx83102e_init(struct himax_ts_data *ts)
{
	u32 val = 0, reload = 0, reload2 = 0;
	int error;
	int i;

	error = himax_bus_enable_burst(ts);
	if (error)
		return error;

	for (i = 0; i < 20; i++) {
		error = himax_bus_read(ts, HIMAX_REG_ADDR_FLASH_RELOAD,
				       &reload, sizeof(reload));
		if (error)
			return error;

		error = himax_bus_read(ts, HIMAX_REG_ADDR_FLASH_RELOAD2,
				       &reload2, sizeof(reload2));
		if (error)
			return error;

		if ((reload & 0xffff) == 0x3aa3 || (reload2 & 0xffff) == 0x72c0)
			break;

		usleep_range(10000, 11000);
	}
	if (i == 20)
		dev_warn(&ts->client->dev,
			 "Flash reload timeout (%#010x, %#010x)\n",
			 reload, reload2);

	/*
	 * HX83102E firmware drops into an unwakeable idle state after ~5.5s
	 * when left in default idle-enabled mode; keep idle disabled while
	 * the panel is enabled.
	 */
	for (i = 0; i < 20; i++) {
		error = himax_bus_read(ts, HIMAX_REG_ADDR_FW_MODE_STATUS,
				       &val, sizeof(val));
		if (error)
			return error;

		if ((val & 0xff) == HIMAX_FW_DATA_IDLE_DIS_PWD)
			return 0;

		val = (val & ~0xffU) | HIMAX_FW_DATA_IDLE_DIS_PWD;
		error = himax_bus_write(ts, HIMAX_REG_ADDR_FW_MODE_STATUS, val);
		if (error)
			return error;

		usleep_range(10000, 11000);
	}

	dev_err(&ts->client->dev, "Failed to disable idle mode (%#010x)\n", val);
	return -ETIMEDOUT;
}

static int himax_input_register(struct himax_ts_data *ts)
{
	int error;

	ts->input_dev = devm_input_allocate_device(&ts->client->dev);
	if (!ts->input_dev) {
		dev_err(&ts->client->dev, "Failed to allocate input device\n");
		return -ENOMEM;
	}

	ts->input_dev->name = "Himax Touchscreen";

	input_set_capability(ts->input_dev, EV_ABS, ABS_MT_POSITION_X);
	input_set_capability(ts->input_dev, EV_ABS, ABS_MT_POSITION_Y);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 200, 0, 0);

	touchscreen_parse_properties(ts->input_dev, true, &ts->props);

	error = input_mt_init_slots(ts->input_dev, HIMAX_MAX_POINTS,
				    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (error) {
		dev_err(&ts->client->dev,
			"Failed to initialize MT slots: %d\n", error);
		return error;
	}

	error = input_register_device(ts->input_dev);
	if (error) {
		dev_err(&ts->client->dev,
			"Failed to register input device: %d\n", error);
		return error;
	}

	return 0;
}

static u8 himax_event_get_num_points(const struct himax_event *event)
{
	if (event->num_points == 0xff)
		return 0;
	else
		return event->num_points & 0x0f;
}

static bool himax_process_event_point(struct himax_ts_data *ts,
				      const struct himax_event *event,
				      int point_index)
{
	const struct himax_event_point *point = &event->points[point_index];
	u16 x = be16_to_cpu(point->x);
	u16 y = be16_to_cpu(point->y);
	u8 w = event->majors[point_index];

	if (x == HIMAX_INVALID_COORD || y == HIMAX_INVALID_COORD)
		return false;

	input_mt_slot(ts->input_dev, point_index);
	input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
	touchscreen_report_pos(ts->input_dev, &ts->props, x, y, true);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
	return true;
}

static void himax_process_event(struct himax_ts_data *ts,
				const struct himax_event *event)
{
	int i;
	int num_points_left = himax_event_get_num_points(event);

	for (i = 0; i < HIMAX_MAX_POINTS && num_points_left > 0; i++) {
		if (himax_process_event_point(ts, event, i))
			num_points_left--;
	}

	input_mt_sync_frame(ts->input_dev);
	input_sync(ts->input_dev);
}

static bool himax_verify_checksum(struct himax_ts_data *ts,
				  const struct himax_event *event)
{
	u8 *data = (u8 *)event;
	int i;
	u16 checksum = 0;

	if (!memchr_inv(data, 0, sizeof(*event)))
		return false;

	for (i = 0; i < sizeof(*event); i++)
		checksum += data[i];

	if ((checksum & 0x00ff) != 0) {
		dev_err(&ts->client->dev, "Wrong event checksum: %04x\n",
			checksum);
		return false;
	}

	return true;
}

static int himax_read_events(struct himax_ts_data *ts,
			     struct himax_event *event, size_t length)
{
	return regmap_raw_read(ts->regmap, HIMAX_AHB_ADDR_EVENT_STACK, event,
			       length);
}

/*
 * HX83102E/P firmware with pen support appends a 12-byte stylus block after
 * the 56-byte touch event in the event stack (68 bytes total). Reading only
 * 56 bytes leaves the 12-byte pen block in the FIFO so the level-low IRQ
 * immediately re-fires and parses the 12-byte tail (+ 44 zero bytes, whose
 * own checksum sums to 0) as a bogus 0-finger lift event.
 */
#define HX83102E_PEN_INFO_SIZE		12

static int hx83102e_read_events(struct himax_ts_data *ts,
				struct himax_event *event, size_t length)
{
	u8 buf[sizeof(*event) + HX83102E_PEN_INFO_SIZE];
	int error;

	error = regmap_raw_read(ts->regmap, HIMAX_AHB_ADDR_EVENT_STACK, buf,
				sizeof(buf));
	if (error)
		return error;

	memcpy(event, buf, length);
	return 0;
}

static int hx83100a_read_events(struct himax_ts_data *ts,
				struct himax_event *event, size_t length)
{
	return himax_bus_read(ts, HX83100A_REG_FW_EVENT_STACK, event, length);
};

static int himax_handle_input(struct himax_ts_data *ts)
{
	int error;
	struct himax_event event;

	error = ts->chip->read_events(ts, &event, sizeof(event));
	if (error) {
		dev_err(&ts->client->dev, "Failed to read input event: %d\n",
			error);
		return error;
	}

	/*
	 * Only process the current event when it has a valid checksum but
	 * don't consider it a fatal error when it doesn't.
	 */
	if (himax_verify_checksum(ts, &event))
		himax_process_event(ts, &event);

	return 0;
}

static irqreturn_t himax_irq_handler(int irq, void *dev_id)
{
	int error;
	struct himax_ts_data *ts = dev_id;

	error = himax_handle_input(ts);
	if (error)
		return IRQ_NONE;

	return IRQ_HANDLED;
}

static int himax_power_on(struct himax_ts_data *ts)
{
	int error;

	error = regulator_enable(ts->vdd);
	if (error) {
		dev_err(&ts->client->dev, "Failed to enable vdd: %d\n", error);
		return error;
	}

	usleep_range(5000, 6000);
	himax_reset(ts);

	if (ts->chip->check_id) {
		error = ts->chip->check_id(ts);
		if (error)
			goto err_power_off;
	}

	if (ts->chip->init) {
		error = ts->chip->init(ts);
		if (error)
			goto err_power_off;
	}

	return 0;

err_power_off:
	gpiod_set_value_cansleep(ts->gpiod_rst, 1);
	regulator_disable(ts->vdd);
	return error;
}

static void himax_power_off(struct himax_ts_data *ts)
{
	gpiod_set_value_cansleep(ts->gpiod_rst, 1);
	regulator_disable(ts->vdd);
}

static void himax_power_off_action(void *data)
{
	himax_power_off(data);
}

static int himax_panel_enabled(struct drm_panel_follower *follower)
{
	struct himax_ts_data *ts = container_of(follower, struct himax_ts_data,
						panel_follower);
	int error;

	if (ts->enabled)
		return 0;

	error = himax_power_on(ts);
	if (error)
		return error;

	ts->enabled = true;
	enable_irq(ts->client->irq);
	return 0;
}

static int himax_panel_disabling(struct drm_panel_follower *follower)
{
	struct himax_ts_data *ts = container_of(follower, struct himax_ts_data,
						panel_follower);

	if (!ts->enabled)
		return 0;

	disable_irq(ts->client->irq);
	himax_power_off(ts);
	ts->enabled = false;
	return 0;
}

static const struct drm_panel_follower_funcs himax_panel_follower_funcs = {
	.panel_enabled = himax_panel_enabled,
	.panel_disabling = himax_panel_disabling,
};

static int himax_probe(struct i2c_client *client)
{
	int error;
	struct device *dev = &client->dev;
	struct himax_ts_data *ts;
	unsigned long irq_flags = IRQF_ONESHOT;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(dev, "I2C check functionality failed\n");
		return -ENXIO;
	}

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	i2c_set_clientdata(client, ts);
	ts->client = client;
	ts->chip = i2c_get_match_data(client);
	ts->is_panel_follower = drm_is_panel_follower(dev);

	ts->regmap = devm_regmap_init_i2c(client, &himax_regmap_config);
	error = PTR_ERR_OR_ZERO(ts->regmap);
	if (error) {
		dev_err(dev, "Failed to initialize regmap: %d\n", error);
		return error;
	}

	ts->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(ts->vdd))
		return dev_err_probe(dev, PTR_ERR(ts->vdd),
				     "Failed to get vdd regulator\n");

	ts->gpiod_rst = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	error = PTR_ERR_OR_ZERO(ts->gpiod_rst);
	if (error) {
		dev_err(dev, "Failed to get reset GPIO: %d\n", error);
		return error;
	}

	if (!ts->is_panel_follower) {
		error = himax_power_on(ts);
		if (error)
			return error;

		error = devm_add_action_or_reset(dev, himax_power_off_action, ts);
		if (error)
			return error;
	} else {
		irq_flags |= IRQF_NO_AUTOEN;
	}

	error = himax_input_register(ts);
	if (error)
		return error;

	error = devm_request_threaded_irq(dev, client->irq, NULL,
					  himax_irq_handler, irq_flags,
					  client->name, ts);
	if (error)
		return error;

	if (ts->is_panel_follower) {
		ts->panel_follower.funcs = &himax_panel_follower_funcs;
		error = devm_drm_panel_add_follower(dev, &ts->panel_follower);
		if (error)
			return dev_err_probe(dev, error,
					     "Failed to add panel follower\n");
	}

	return 0;
}

static int himax_suspend(struct device *dev)
{
	struct himax_ts_data *ts = dev_get_drvdata(dev);

	if (ts->is_panel_follower)
		return 0;

	disable_irq(ts->client->irq);
	return 0;
}

static int himax_resume(struct device *dev)
{
	struct himax_ts_data *ts = dev_get_drvdata(dev);

	if (ts->is_panel_follower)
		return 0;

	enable_irq(ts->client->irq);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(himax_pm_ops, himax_suspend, himax_resume);

static const struct himax_chip hx83100a_chip = {
	.read_events = hx83100a_read_events,
};

static const struct himax_chip hx83102e_chip = {
	.id = 0x83102e,
	.post_reset_ms = 50,
	.check_id = himax_check_product_id,
	.init = hx83102e_init,
	.read_events = hx83102e_read_events,
};

static const struct himax_chip hx83112b_chip = {
	.id = 0x83112b,
	.check_id = himax_check_product_id,
	.read_events = himax_read_events,
};

static const struct i2c_device_id himax_ts_id[] = {
	{ .name = "hx83100a", .driver_data = (kernel_ulong_t)&hx83100a_chip },
	{ .name = "hx83102e", .driver_data = (kernel_ulong_t)&hx83102e_chip },
	{ .name = "hx83112b", .driver_data = (kernel_ulong_t)&hx83112b_chip },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, himax_ts_id);

#ifdef CONFIG_OF
static const struct of_device_id himax_of_match[] = {
	{ .compatible = "himax,hx83100a", .data = &hx83100a_chip },
	{ .compatible = "himax,hx83102e", .data = &hx83102e_chip },
	{ .compatible = "himax,hx83112b", .data = &hx83112b_chip },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, himax_of_match);
#endif

static struct i2c_driver himax_ts_driver = {
	.probe = himax_probe,
	.id_table = himax_ts_id,
	.driver = {
		.name = "Himax-hx83112b-TS",
		.of_match_table = of_match_ptr(himax_of_match),
		.pm = pm_sleep_ptr(&himax_pm_ops),
	},
};
module_i2c_driver(himax_ts_driver);

MODULE_AUTHOR("Job Noorman <job@noorman.info>");
MODULE_DESCRIPTION("Himax hx83112b touchscreen driver");
MODULE_LICENSE("GPL");
