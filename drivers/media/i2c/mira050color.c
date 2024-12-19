// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for ams MIRA050 cameras.
 * Copyright (C) 2022, ams-OSRAM
 *
 * Based on Sony IMX219 camera driver
 * Copyright (C) 2019, Raspberry Pi (Trading) Ltd
 */

#include "mira050.inl"

static const struct of_device_id mira050_dt_ids[] = {
	{ .compatible = "ams,mira050color" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mira050_dt_ids);

static const struct i2c_device_id mira050_ids[] = {
	{ "mira050color", 1 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mira050_ids);

static struct i2c_driver mira050_i2c_driver = {
	.driver = {
		.name = "mira050color",
		.of_match_table	= mira050_dt_ids,
		.pm = &mira050_pm_ops,
	},
	
	.probe = mira050_probe,
	.remove = mira050_remove,
	.id_table = mira050_ids,
};

module_i2c_driver(mira050_i2c_driver);

MODULE_AUTHOR("Zhenyu Ye <zhenyu.ye@ams-osram.com>");
MODULE_DESCRIPTION("ams MIRA050 sensor driver");
MODULE_LICENSE("GPL v2");

