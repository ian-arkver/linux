/*
 * Copyright (c) 2016 Ian Jamison, Arkver
 *
 * based loosely on the OV5642 driver which is...
 * Copyright (c) 2014 Philipp Zabel, Pengutronix
 * Copyright (c) 2012-2014 Mentor Graphics Inc.
 * Copyright (C) 2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

/* Fixed resolution (non-I2C) camera */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/ctype.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-of.h>

#define DEFAULT_FPS 	30
#define DEFAULT_WIDTH	720
#define DEFAULT_HEIGHT	480

#define MAX_FPS		100
#define MIN_FPS		1

#define FIXEDCAM_XCLK_MIN   1000000
#define FIXEDCAM_XCLK_MAX 100000000

struct fixedcam_dev {
	struct i2c_client *i2c_client;
	struct device *dev;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_hdl;
	struct v4l2_of_endpoint ep; /* the parsed DT endpoint info */
	struct v4l2_mbus_framefmt fmt;
	struct v4l2_rect crop;
	struct v4l2_captureparm streamcap;

	bool on;

	/*
	 * These are not directly used but are included to
	 * allow DT to call up clocks, GPIOs or regulators
	 * as needed. Names are from ov5642 driver and have
	 * no meaning here.
	 */
	struct clk *xclk; /* An internal clock */
	int xclk_freq;    /* requested xclk freq from devicetree */

	int reset_gpio;
	int pwdn_gpio;
	int gp_gpio;

	struct regulator *io_regulator;
	struct regulator *core_regulator;
	struct regulator *analog_regulator;
	struct regulator *gpo_regulator;
};

static inline struct fixedcam_dev *to_fixedcam_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct fixedcam_dev, sd);
}

static void fixedcam_power(struct fixedcam_dev *sensor, bool enable);

static int fixedcam_probe(struct i2c_client *adapter,
				const struct i2c_device_id *device_id);
static int fixedcam_remove(struct i2c_client *client);

static void fixedcam_dump_format(struct fixedcam_dev *sensor)
{
	pr_debug("Image size %u x %x:\n", sensor->fmt.width, sensor->fmt.height);
}

static int fixedcam_regulators_on(struct fixedcam_dev *sensor)
{
	int ret;

	if (sensor->io_regulator) {
		ret = regulator_enable(sensor->io_regulator);
		if (ret) {
			v4l2_err(&sensor->sd, "io reg enable failed\n");
			return ret;
		}
	}
	if (sensor->core_regulator) {
		ret = regulator_enable(sensor->core_regulator);
		if (ret) {
			v4l2_err(&sensor->sd, "core reg enable failed\n");
			return ret;
		}
	}
	if (sensor->gpo_regulator) {
		ret = regulator_enable(sensor->gpo_regulator);
		if (ret) {
			v4l2_err(&sensor->sd, "gpo reg enable failed\n");
			return ret;
		}
	}
	if (sensor->analog_regulator) {
		ret = regulator_enable(sensor->analog_regulator);
		if (ret) {
			v4l2_err(&sensor->sd, "analog reg enable failed\n");
			return ret;
		}
	}

	return 0;
}

static void fixedcam_regulators_off(struct fixedcam_dev *sensor)
{
	if (sensor->analog_regulator)
		regulator_disable(sensor->analog_regulator);
	if (sensor->core_regulator)
		regulator_disable(sensor->core_regulator);
	if (sensor->io_regulator)
		regulator_disable(sensor->io_regulator);
	if (sensor->gpo_regulator)
		regulator_disable(sensor->gpo_regulator);
}

/* --------------- Subdev Operations --------------- */

static int fixedcam_s_power(struct v4l2_subdev *sd, int on)
{
	struct fixedcam_dev *sensor = to_fixedcam_dev(sd);

	if (on && !sensor->on) {
		if (sensor->xclk)
			clk_prepare_enable(sensor->xclk);

		fixedcam_regulators_on(sensor);

		fixedcam_power(sensor, true);

		fixedcam_restore_mode(sensor);
	} else if (!on && sensor->on) {
		fixedcam_power(sensor, false);

		fixedcam_regulators_off(sensor);

		if (sensor->xclk)
			clk_disable_unprepare(sensor->xclk);
	}

	sensor->on = on;

	return 0;
}

static int fixedcam_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct fixedcam_dev *sensor = to_fixedcam_dev(sd);

	fi->interval = sensor->streamcap.timeperframe;

	return 0;
}

static int fixedcam_s_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *fi)
{
	struct fixedcam_dev *sensor = to_fixedcam_dev(sd);
	struct v4l2_fract *timeperframe = &fi->interval;
	enum fixedcam_frame_rate new_frame_rate;
	u32 tgt_fps;
	int ret;

	/* Check that the new frame rate is allowed. */
	if ((timeperframe->numerator == 0) ||
	    (timeperframe->denominator == 0)) {
		timeperframe->denominator = DEFAULT_FPS;
		timeperframe->numerator = 1;
	}

	tgt_fps = timeperframe->denominator / timeperframe->numerator;

	if (tgt_fps > MAX_FPS) {
		timeperframe->denominator = MAX_FPS;
		timeperframe->numerator = 1;
	} else if (tgt_fps < MIN_FPS) {
		timeperframe->denominator = MIN_FPS;
		timeperframe->numerator = 1;
	}

	sensor->streamcap.timeperframe = *timeperframe;

	return 0;
}


static int fixedcam_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static int fixedcam_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct fixedcam_dev *sensor = to_fixedcam_dev(sd);

	if (code->index > 0)
		return -EINVAL;

	code->code = sensor->fmt.code;
	return 0;
}

static int fixedcam_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct fixedcam_dev *sensor = to_fixedcam_dev(sd);

	fse->min_width = sensor->fmt.width;
	fse->max_width = sensor->fmt.width;
	fse->min_height = sensor->fmt.height;
	fse->max_height = sensor->fmt.height;
	return 0;
}

static struct v4l2_mbus_framefmt *
__fixedcam_get_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg,
			unsigned int pad, enum v4l2_subdev_format_whence which)
{
	struct fixedcam_dev *sensor = to_fixedcam_dev(sd);

	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(sd, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &sensor->fmt;
	default:
		return NULL;
	}
}

static struct v4l2_rect *
__fixedcam_get_pad_crop(struct v4l2_subdev *sd, struct v4l2_subdev_pad_config *cfg,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	struct fixedcam_dev *sensor = to_fixedcam_dev(sd);

	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(sd, cfg, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &sensor->crop;
	default:
		return NULL;
	}
}

static int fixedcam_get_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *format)
{
	format->format = *__fixedcam_get_pad_format(sd, cfg, format->pad,
						  format->which);
	return 0;
}

static int fixedcam_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_pad_config *cfg,
			     struct v4l2_subdev_format *format)
{
	struct fixedcam_dev *sensor = to_fixedcam_dev(sd);
	struct v4l2_mbus_framefmt *__format;
	struct v4l2_rect *__crop;
	int ret;

	__crop = __fixedcam_get_pad_crop(sd, cfg, format->pad, format->which);

	/* ToDo: Check crop limits for sanity? */
	/* ToDo: Handle changes of interlace/progressive? */
	/* ToDo: Handle changes of input format? */

	sensor->fmt.width =	__crop->width = format->format.width;
	sensor->fmt.height = __crop->height = format->format.height;

	__format = __fixedcam_get_pad_format(sd, cfg, format->pad,
					   format->which);
	__format->width = __crop->width;
	__format->height = __crop->height;

	fixedcam_dump_format(sensor);
	return 0;
}

static int fixedcam_get_selection(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_selection *sel)
{
	if (sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	sel->r = *__fixedcam_get_pad_crop(sd, cfg, sel->pad,
					    sel->which);
	return 0;
}

static struct v4l2_subdev_core_ops fixedcam_core_ops = {
	.s_power = fixedcam_s_power,
	.g_ext_ctrls = v4l2_subdev_g_ext_ctrls,
	.try_ext_ctrls = v4l2_subdev_try_ext_ctrls,
	.s_ext_ctrls = v4l2_subdev_s_ext_ctrls,
	.g_ctrl = v4l2_subdev_g_ctrl,
	.s_ctrl = v4l2_subdev_s_ctrl,
	.queryctrl = v4l2_subdev_queryctrl,
	.querymenu = v4l2_subdev_querymenu,
};

static struct v4l2_subdev_video_ops fixedcam_video_ops = {
	.s_frame_interval = fixedcam_s_frame_interval,
	.g_frame_interval = fixedcam_g_frame_interval,
	.s_stream = fixedcam_s_stream,
};

static struct v4l2_subdev_pad_ops fixedcam_subdev_pad_ops = {
	.enum_mbus_code = fixedcam_enum_mbus_code,
	.enum_frame_size = fixedcam_enum_frame_size,
	.get_fmt = fixedcam_get_format,
	.set_fmt = fixedcam_set_format,
	.get_selection = fixedcam_get_selection,
};

static struct v4l2_subdev_ops fixedcam_subdev_ops = {
	.core = &fixedcam_core_ops,
	.video = &fixedcam_video_ops,
	.pad = &fixedcam_subdev_pad_ops,
};

static void fixedcam_power(struct fixedcam_dev *sensor, bool enable)
{
	gpio_set_value(sensor->pwdn_gpio, enable ? 0 : 1);
}

static void fixedcam_get_regulators(struct fixedcam_dev *sensor)
{
	sensor->io_regulator = devm_regulator_get(sensor->dev, "DOVDD");
	if (IS_ERR(sensor->io_regulator))
		sensor->io_regulator = NULL;

	sensor->core_regulator = devm_regulator_get(sensor->dev, "DVDD");
	if (IS_ERR(sensor->core_regulator))
		sensor->core_regulator = NULL;

	sensor->analog_regulator = devm_regulator_get(sensor->dev, "AVDD");
	if (IS_ERR(sensor->analog_regulator))
		sensor->analog_regulator = NULL;
}

/*!
 * fixedcam I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int fixedcam_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *endpoint;
	struct fixedcam_dev *sensor;
	int i, xclk, ret;

	sensor = devm_kzalloc(dev, sizeof(struct fixedcam_dev),
			      GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;

	sensor->i2c_client = client;
	sensor->dev = dev;
	sensor->fmt.code = MEDIA_BUS_FMT_YUYV8_2X8;
	sensor->fmt.width = DEFAULT_WIDTH;
	sensor->fmt.height = DEFAULT_HEIGHT;
	sensor->fmt.field = V4L2_FIELD_NONE;
	sensor->streamcap.capability = V4L2_MODE_HIGHQUALITY |
					   V4L2_CAP_TIMEPERFRAME;
	sensor->streamcap.capturemode = 0;
	sensor->streamcap.timeperframe.denominator = DEFAULT_FPS;
	sensor->streamcap.timeperframe.numerator = 1;

	endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	v4l2_of_parse_endpoint(endpoint, &sensor->ep);
	if (sensor->ep.bus_type != V4L2_MBUS_PARALLEL) {
		dev_err(dev, "invalid bus type, must be parallel\n");
		return -EINVAL;
	}
	of_node_put(endpoint);

	/* Override defaults with DT values if they exist */
	/* No errors if they don't */
	(void)of_property_read_u32(dev->of_node, "width", &sensor->fmt.width);
	(void)of_property_read_u32(dev->of_node, "height", &sensor->fmt.height);
	(void)of_property_read_u32(dev->of_node, "frametime_n", &sensor->streamcap.timeperframe.numerator);
	(void)of_property_read_u32(dev->of_node, "frametime_d", &sensor->streamcap.timeperframe.denominator);

	/* get system clock (xclk) frequency */
	ret = of_property_read_u32(dev->of_node, "xclk", &xclk);
	if (!ret) {
		if (xclk < FIXEDCAM_XCLK_MIN || xclk > FIXEDCAM_XCLK_MAX) {
			dev_err(dev, "invalid xclk frequency\n");
			return -EINVAL;
		}
		sensor->xclk_freq = xclk;
	}

	/* get system clock (xclk) */
	sensor->xclk = devm_clk_get(dev, "xclk");
	if (!IS_ERR(sensor->xclk)) {
		if (!sensor->xclk_freq) {
			dev_err(dev,
				"xclk requires xclk frequency!\n");
			return -EINVAL;
		}
		clk_set_rate(sensor->xclk, sensor->xclk_freq);
	} else
		sensor->xclk = NULL;

	/* Set high while sensor is on */
	sensor->pwdn_gpio = of_get_named_gpio(dev->of_node, "pwdn-gpios", 0);
	if (gpio_is_valid(sensor->pwdn_gpio)) {
		ret = devm_gpio_request_one(dev, sensor->pwdn_gpio,
						GPIOF_OUT_INIT_HIGH, "fixedcam_pwdn");
		if (ret < 0) {
			dev_err(dev, "request for power down gpio failed\n");
			return ret;
		}
	}

	/* Set low for 1ms, 5ms after power is enabled */
	sensor->reset_gpio = of_get_named_gpio(dev->of_node, "reset-gpios", 0);
	if (gpio_is_valid(sensor->reset_gpio)) {
		ret = devm_gpio_request_one(dev, sensor->reset_gpio,
						GPIOF_OUT_INIT_HIGH, "fixedcam_reset");
		if (ret < 0) {
			dev_err(dev, "request for reset gpio failed\n");
			return ret;
		}
	}

	/*
	 * General Purpose GPIO?
	 * Set high permanently
	 */
	sensor->gp_gpio = of_get_named_gpio(dev->of_node, "gp-gpios", 0);
	if (gpio_is_valid(sensor->gp_gpio)) {
		ret = devm_gpio_request_one(dev, sensor->gp_gpio,
					    GPIOF_OUT_INIT_HIGH, "fixedcam_gp");
		if (ret < 0) {
			dev_err(dev, "request for gp gpio failed\n");
			return ret;
		}
		gpio_set_value(sensor->gp_gpio, 1);
	}

	dev_info(dev, "gpios: reset %d, power-down %d, gp %d\n",
		 sensor->reset_gpio, sensor->pwdn_gpio, sensor->gp_gpio);

	v4l2_i2c_subdev_init(&sensor->sd, client, &fixedcam_subdev_ops);
	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret < 0)
		goto err;

	sensor->sd.dev = &client->dev;
	ret = v4l2_async_register_subdev(&sensor->sd);
	if (ret < 0)
		goto err;

	fixedcam_get_regulators(sensor);

	fixedcam_s_power(&sensor->sd, 1);
	msleep(5);
	gpio_set_value(sensor->reset_gpio, 0);
	msleep(1);
	gpio_set_value(sensor->reset_gpio, 1);
	msleep(20);

	ret = fixedcam_init_controls(sensor);

	fixedcam_s_power(&sensor->sd, 0);

	if (ret < 0)
		goto err;

	return 0;

err:
	media_entity_cleanup(&sensor->sd.entity);
	return ret;
}

/*!
 * fixedcam I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int fixedcam_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct fixedcam_dev *sensor = to_fixedcam_dev(sd);

	fixedcam_regulators_off(sensor);

	v4l2_async_unregister_subdev(sd);
	v4l2_ctrl_handler_free(&sensor->ctrl_hdl);

	return 0;
}

static const struct i2c_device_id fixedcam_id[] = {
	{ "fixedcam", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, fixedcam_id);

static struct of_device_id fixedcam_dt_ids[] = {
	{ .compatible = "ark,fixedcam" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, fixedcam_dt_ids);

static struct i2c_driver fixedcam_driver = {
	.driver = {
		.name	= "fixedcam",
		.owner	= THIS_MODULE,
		.of_match_table	= fixedcam_dt_ids,
	},
	.id_table	= fixedcam_id,
	.probe		= fixedcam_probe,
	.remove		= fixedcam_remove,
};

module_i2c_driver(fixedcam_driver);

MODULE_AUTHOR("Arkver Ltd.");
MODULE_DESCRIPTION("Fixed Resolution Camera Subdev Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
