/*
 *  Driver for Goodix Touchscreens
 *
 *  Copyright (c) 2014 Red Hat Inc.
 *
 *  This code is based on gt9xx.c authored by andrew@hycon.com:
 *
 *  2010 - 2012 Goodix Technology.
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; version 2 of the License.
 */

#include <linux/kernel.h>
#include <linux/dmi.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <linux/of.h>
#include <asm/unaligned.h>

struct hycon_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	int abs_x_max;
	int abs_y_max;
	unsigned int max_touch_num;
	unsigned int int_trigger_type;
	bool rotated_screen;
};

#define HYCON_MAX_HEIGHT		4096
#define HYCON_MAX_WIDTH		4096
#define HYCON_INT_TRIGGER		1
#define HYCON_CONTACT_SIZE		8
#define HYCON_MAX_CONTACTS		10

#define HYCON_CONFIG_MAX_LENGTH	240

/* Register defines */
#define HYCON_READ_COOR_ADDR		0x814E
#define HYCON_REG_CONFIG_DATA		0x8047
#define HYCON_REG_ID			0x8140

#define RESOLUTION_LOC		1
#define MAX_CONTACTS_LOC	5
#define TRIGGER_LOC		6

static const unsigned long hycon_irq_flags[] = {
	IRQ_TYPE_EDGE_RISING,
	IRQ_TYPE_EDGE_FALLING,
	IRQ_TYPE_LEVEL_LOW,
	IRQ_TYPE_LEVEL_HIGH,
};


/**
 * hycon_i2c_read - read data from a register of the i2c slave device.
 *
 * @client: i2c device.
 * @reg: the register to read from.
 * @buf: raw write data buffer.
 * @len: length of the buffer to write
 */
static int hycon_i2c_read(struct i2c_client *client,
			   u16 reg, u8 *buf, int len)
{
	struct i2c_msg msgs[2];
	u16 wbuf = cpu_to_be16(reg);
	int ret;

	msgs[0].flags = 0;
	msgs[0].addr  = client->addr;
	msgs[0].len   = 2;
	msgs[0].buf   = (u8 *)&wbuf;

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = client->addr;
	msgs[1].len   = len;
	msgs[1].buf   = buf;

	ret = i2c_transfer(client->adapter, msgs, 2);
	return ret < 0 ? ret : (ret != ARRAY_SIZE(msgs) ? -EIO : 0);
}

static int hycon_ts_read_input_report(struct hycon_ts_data *ts, u8 *data)
{
	int touch_num;
	int error;

	error = hycon_i2c_read(ts->client, HYCON_READ_COOR_ADDR, data,
				HYCON_CONTACT_SIZE + 1);
	if (error) {
		dev_err(&ts->client->dev, "I2C transfer error: %d\n", error);
		return error;
	}

	if (!(data[0] & 0x80))
		return -EAGAIN;

	touch_num = data[0] & 0x0f;
	if (touch_num > ts->max_touch_num)
		return -EPROTO;

	if (touch_num > 1) {
		data += 1 + HYCON_CONTACT_SIZE;
		error = hycon_i2c_read(ts->client,
					HYCON_READ_COOR_ADDR +
						1 + HYCON_CONTACT_SIZE,
					data,
					HYCON_CONTACT_SIZE * (touch_num - 1));
		if (error)
			return error;
	}

	return touch_num;
}

static void hycon_ts_report_touch(struct hycon_ts_data *ts, u8 *coor_data)
{
	int id = coor_data[0] & 0x0F;
	int input_x = get_unaligned_le16(&coor_data[1]);
	int input_y = get_unaligned_le16(&coor_data[3]);
	int input_w = get_unaligned_le16(&coor_data[5]);

	if (ts->rotated_screen) {
		input_x = ts->abs_x_max - input_x;
		input_y = ts->abs_y_max - input_y;
	}

	input_mt_slot(ts->input_dev, id);
	input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
	input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
	input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);
	input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, input_w);
}

/**
 * hycon_process_events - Process incoming events
 *
 * @ts: our hycon_ts_data pointer
 *
 * Called when the IRQ is triggered. Read the current device state, and push
 * the input events to the user space.
 */
static void hycon_process_events(struct hycon_ts_data *ts)
{
	u8  point_data[1 + HYCON_CONTACT_SIZE * HYCON_MAX_CONTACTS];
	int touch_num;
	int i;

	touch_num = hycon_ts_read_input_report(ts, point_data);
	if (touch_num < 0)
		return;

	for (i = 0; i < touch_num; i++)
		hycon_ts_report_touch(ts,
				&point_data[1 + HYCON_CONTACT_SIZE * i]);

	input_mt_sync_frame(ts->input_dev);
	input_sync(ts->input_dev);
}

/**
 * hycon_ts_irq_handler - The IRQ handler
 *
 * @irq: interrupt number.
 * @dev_id: private data pointer.
 */
static irqreturn_t hycon_ts_irq_handler(int irq, void *dev_id)
{
	static const u8 end_cmd[] = {
		HYCON_READ_COOR_ADDR >> 8,
		HYCON_READ_COOR_ADDR & 0xff,
		0
	};
	struct hycon_ts_data *ts = dev_id;

	hycon_process_events(ts);

	if (i2c_master_send(ts->client, end_cmd, sizeof(end_cmd)) < 0)
		dev_err(&ts->client->dev, "I2C write end_cmd error\n");

	return IRQ_HANDLED;
}

/**
 * hycon_read_config - Read the embedded configuration of the panel
 *
 * @ts: our hycon_ts_data pointer
 *
 * Must be called during probe
 */
static void hycon_read_config(struct hycon_ts_data *ts)
{
	u8 config[HYCON_CONFIG_MAX_LENGTH];
	int error;

	error = hycon_i2c_read(ts->client, HYCON_REG_CONFIG_DATA,
				config,
				HYCON_CONFIG_MAX_LENGTH);
	if (error) {
		dev_warn(&ts->client->dev,
			 "Error reading config (%d), using defaults\n",
			 error);
		ts->abs_x_max = HYCON_MAX_WIDTH;
		ts->abs_y_max = HYCON_MAX_HEIGHT;
		ts->int_trigger_type = HYCON_INT_TRIGGER;
		ts->max_touch_num = HYCON_MAX_CONTACTS;
		return;
	}

	ts->abs_x_max = get_unaligned_le16(&config[RESOLUTION_LOC]);
	ts->abs_y_max = get_unaligned_le16(&config[RESOLUTION_LOC + 2]);
	ts->int_trigger_type = config[TRIGGER_LOC] & 0x03;
	ts->max_touch_num = config[MAX_CONTACTS_LOC] & 0x0f;
	if (!ts->abs_x_max || !ts->abs_y_max || !ts->max_touch_num) {
		dev_err(&ts->client->dev,
			"Invalid config, using defaults\n");
		ts->abs_x_max = HYCON_MAX_WIDTH;
		ts->abs_y_max = HYCON_MAX_HEIGHT;
		ts->max_touch_num = HYCON_MAX_CONTACTS;
	}

	ts->rotated_screen = dmi_check_system(rotated_screen);
	if (ts->rotated_screen)
		dev_dbg(&ts->client->dev,
			 "Applying '180 degrees rotated screen' quirk\n");
}

/**
 * hycon_read_version - Read hycon touchscreen version
 *
 * @client: the i2c client
 * @version: output buffer containing the version on success
 * @id: output buffer containing the id on success
 */
static int hycon_read_version(struct i2c_client *client, u16 *version, u16 *id)
{
	int error;
	u8 buf[6];
	char id_str[5];

	error = hycon_i2c_read(client, HYCON_REG_ID, buf, sizeof(buf));
	if (error) {
		dev_err(&client->dev, "read version failed: %d\n", error);
		return error;
	}

	memcpy(id_str, buf, 4);
	id_str[4] = 0;
	if (kstrtou16(id_str, 10, id))
		*id = 0x1001;

	*version = get_unaligned_le16(&buf[4]);

	dev_info(&client->dev, "ID %d, version: %04x\n", *id, *version);

	return 0;
}

/**
 * hycon_i2c_test - I2C test function to check if the device answers.
 *
 * @client: the i2c client
 */
static int hycon_i2c_test(struct i2c_client *client)
{
	int retry = 0;
	int error;
	u8 test;

	while (retry++ < 2) {
		error = hycon_i2c_read(client, HYCON_REG_CONFIG_DATA,
					&test, 1);
		if (!error)
			return 0;

		dev_err(&client->dev, "i2c test failed attempt %d: %d\n",
			retry, error);
		msleep(20);
	}

	return error;
}

/**
 * hycon_request_input_dev - Allocate, populate and register the input device
 *
 * @ts: our hycon_ts_data pointer
 * @version: device firmware version
 * @id: device ID
 *
 * Must be called during probe
 */
static int hycon_request_input_dev(struct hycon_ts_data *ts, u16 version,
				    u16 id)
{
	int error;

	ts->input_dev = devm_input_allocate_device(&ts->client->dev);
	if (!ts->input_dev) {
		dev_err(&ts->client->dev, "Failed to allocate input device.");
		return -ENOMEM;
	}

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,
			     0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,
			     0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);

	input_mt_init_slots(ts->input_dev, ts->max_touch_num,
			    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);

	ts->input_dev->name = "Goodix Capacitive TouchScreen";
	ts->input_dev->phys = "input/ts";
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0x0416;
	ts->input_dev->id.product = id;
	ts->input_dev->id.version = version;

	error = input_register_device(ts->input_dev);
	if (error) {
		dev_err(&ts->client->dev,
			"Failed to register input device: %d", error);
		return error;
	}

	return 0;
}

static int hycon_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct hycon_ts_data *ts;
	unsigned long irq_flags;
	int error;
	u16 version_info, id_info;

	dev_dbg(&client->dev, "I2C Address: 0x%02x\n", client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C check functionality failed.\n");
		return -ENXIO;
	}

	ts = devm_kzalloc(&client->dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ts->client = client;
	i2c_set_clientdata(client, ts);

	error = hycon_i2c_test(client);
	if (error) {
		dev_err(&client->dev, "I2C communication failure: %d\n", error);
		return error;
	}

	error = hycon_read_version(client, &version_info, &id_info);
	if (error) {
		dev_err(&client->dev, "Read version failed.\n");
		return error;
	}

	hycon_read_config(ts);

	error = hycon_request_input_dev(ts, version_info, id_info);
	if (error)
		return error;

	irq_flags = hycon_irq_flags[ts->int_trigger_type] | IRQF_ONESHOT;
	error = devm_request_threaded_irq(&ts->client->dev, client->irq,
					  NULL, hycon_ts_irq_handler,
					  irq_flags, client->name, ts);
	if (error) {
		dev_err(&client->dev, "request IRQ failed: %d\n", error);
		return error;
	}

	return 0;
}

static const struct i2c_device_id hycon_ts_id[] = {
	{ "HYCONXX:00", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, hycon_ts_id);


#ifdef CONFIG_OF
static const struct of_device_id hycon_of_match[] = {
	{ .compatible = "hycon,hy4633" },
	{ }
};
MODULE_DEVICE_TABLE(of, hycon_of_match);
#endif

static struct i2c_driver hycon_ts_driver = {
	.probe = hycon_ts_probe,
	.id_table = hycon_ts_id,
	.driver = {
		.name = "hycon-TS",
		.acpi_match_table = ACPI_PTR(hycon_acpi_match),
		.of_match_table = of_match_ptr(hycon_of_match),
	},
};
module_i2c_driver(hycon_ts_driver);

MODULE_AUTHOR("Bill Lindblad <bill@threefivedisplays.com>");
MODULE_DESCRIPTION("Hycon touchscreen driver");
MODULE_LICENSE("GPL v2");
