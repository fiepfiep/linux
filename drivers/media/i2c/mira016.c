// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for ams MIRA016 cameras.
 * Copyright (C) 2022, ams-OSRAM
 *
 * Based on Sony IMX219 camera driver
 * Copyright (C) 2019, Raspberry Pi (Trading) Ltd
 */

#include "mira016.inl"

static const struct of_device_id mira016_dt_ids[] = {
	{ .compatible = "ams,mira016" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mira016_dt_ids);

static const struct i2c_device_id mira016_ids[] = {
	{ "mira016", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mira016_ids);

static struct i2c_driver mira016_i2c_driver = {
	.driver = {
		.name = "mira016",
		.of_match_table	= mira016_dt_ids,
		.pm = &mira016_pm_ops,
	},
	.probe = mira016_probe,
	.remove = mira016_remove,
	.id_table = mira016_ids,
};

module_i2c_driver(mira016_i2c_driver);

MODULE_AUTHOR("Zhenyu Ye <zhenyu.ye@ams-osram.com>");
MODULE_DESCRIPTION("ams MIRA016 sensor driver");
MODULE_LICENSE("GPL v2");

