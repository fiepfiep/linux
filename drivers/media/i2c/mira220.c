// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for ams MIRA220 cameras.
 * Copyright (C) 2022, ams-OSRAM
 *
 * Based on Sony IMX219 camera driver
 * Copyright (C) 2019, Raspberry Pi (Trading) Ltd
 */

#include "mira220.inl"

static const struct of_device_id mira220_dt_ids[] = {
	{ .compatible = "ams,mira220" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mira220_dt_ids);

static const struct i2c_device_id mira220_ids[] = {
	{ "mira220", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mira220_ids);

static struct i2c_driver mira220_i2c_driver = {
	.driver = {
		.name = "mira220",
		.of_match_table	= mira220_dt_ids,
		.pm = &mira220_pm_ops,
	},
	.probe = mira220_probe,
	.remove = mira220_remove,
	.id_table = mira220_ids,
};

module_i2c_driver(mira220_i2c_driver);

MODULE_AUTHOR("Zhenyu Ye <zhenyu.ye@ams-osram.com>");
MODULE_DESCRIPTION("ams MIRA220 sensor driver");
MODULE_LICENSE("GPL v2");
