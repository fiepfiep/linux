// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for ams MIRA130 cameras.
 * Copyright (C) 2022, ams-OSRAM
 *
 * Based on Sony IMX219 camera driver
 * Copyright (C) 2019, Raspberry Pi (Trading) Ltd
 */

#include "mira130.inl"

static const struct of_device_id mira130_dt_ids[] = {
	{ .compatible = "ams,mira130" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mira130_dt_ids);

static const struct i2c_device_id mira130_ids[] = {
	{ "mira130", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mira130_ids);

static struct i2c_driver mira130_i2c_driver = {
	.driver = {
		.name = "mira130",
		.of_match_table	= mira130_dt_ids,
		.pm = &mira130_pm_ops,
	},
	.probe = mira130_probe,
	.remove = mira130_remove,
	.id_table = mira130_ids,
};

module_i2c_driver(mira130_i2c_driver);

MODULE_AUTHOR("Zhenyu Ye <zhenyu.ye@ams-osram.com>");
MODULE_DESCRIPTION("ams MIRA130 sensor driver");
MODULE_LICENSE("GPL v2");
