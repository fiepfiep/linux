// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for ams MIRA016 cameras.
 * Copyright (C) 2022, ams-OSRAM
 *
 * Based on Sony IMX219 camera driver
 * Copyright (C) 2019, Raspberry Pi (Trading) Ltd
 */

// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for ams MIRA016 cameras.
 * Copyright (C) 2022, ams-OSRAM
 *
 * Based on Sony IMX219 camera driver
 * Copyright (C) 2019, Raspberry Pi (Trading) Ltd
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-cci.h>
#include <linux/unaligned.h>

/*
 * Introduce new v4l2 control
 */
#include <linux/v4l2-controls.h>


struct mira016_reg
{
	u16 address;
	u8 val;
};

struct mira016_fine_gain_lut_new
{
	u32 analog_gain;
	u8 gdig_preamp;
	u8 rg_adcgain;
	u8 rg_mult;
};

struct mira016_reg_list
{
	unsigned int num_of_regs;
	const struct mira016_reg *regs;
};

struct mira016_v4l2_reg
{
	u32 val;
};

// converted_Draco_i2c_configuration_sequence_hex_10bit_1x_360fps_Version3
static const struct mira016_reg full_400_400_100fps_10b_1lane_reg_pre_soft_reset[] = {

	//"Mira016_register_sequence_10b_1-4x_60fps_1000M.txt"
	{0xE000, 0x0},	// None
	{0x01E4, 0x0},	// None
	{0x01E5, 0x13}, // None
	{0x01E2, 0x17}, // None
	{0x01E3, 0xA8}, // None
	{0x01E6, 0x0},	// None
	{0x01E7, 0xCA}, // None
	{0x016C, 0x1},	// None
	{0x016B, 0x1},	// None
	{0x0208, 0x1},	// None
	{0x0209, 0xF0}, // None
	{0x020A, 0x3},	// None
	{0x020B, 0x4D}, // None
	{0x020C, 0x2},	// None
	{0x020D, 0x10}, // None
	{0x020E, 0x3},	// None
	{0x020F, 0x1},	// None
	{0x0210, 0x0},	// None
	{0x0211, 0x13}, // None
	{0x0212, 0x0},	// None
	{0x0213, 0x3},	// None
	{0x0214, 0x3},	// None
	{0x0215, 0xEF}, // None
	{0x0216, 0x3},	// None
	{0x0217, 0xF3}, // None
	{0x0218, 0x3},	// None
	{0x0219, 0xF4}, // None
	{0x021A, 0x0},	// None
	{0x021B, 0x1},	// None
	{0x021C, 0x3},	// None
	{0x021D, 0xF8}, // None
	{0x021E, 0x0},	// None
	{0x021F, 0x2},	// None
	{0x0220, 0x1},	// None
	{0x0221, 0xF2}, // None
	{0x0222, 0x3},	// None
	{0x0223, 0x1B}, // None
	{0x0224, 0x0},	// None
	{0x0225, 0x21}, // None
	{0x0226, 0x3},	// None
	{0x0227, 0xF0}, // None
	{0x0228, 0x3},	// None
	{0x0229, 0xF1}, // None
	{0x022A, 0x3},	// None
	{0x022B, 0xF2}, // None
	{0x022C, 0x3},	// None
	{0x022D, 0xF5}, // None
	{0x022E, 0x3},	// None
	{0x022F, 0xF6}, // None
	{0x0230, 0x0},	// None
	{0x0231, 0xC1}, // None
	{0x0232, 0x0},	// None
	{0x0233, 0x2},	// None
	{0x0234, 0x1},	// None
	{0x0235, 0xF2}, // None
	{0x0236, 0x3},	// None
	{0x0237, 0x6B}, // None
	{0x0238, 0x3},	// None
	{0x0239, 0xFF}, // None
	{0x023A, 0x3},	// None
	{0x023B, 0x31}, // None
	{0x023C, 0x1},	// None
	{0x023D, 0xF0}, // None
	{0x023E, 0x3},	// None
	{0x023F, 0x87}, // None
	{0x0240, 0x0},	// None
	{0x0241, 0xA},	// None
	{0x0242, 0x0},	// None
	{0x0243, 0xB},	// None
	{0x0244, 0x1},	// None
	{0x0245, 0xF9}, // None
	{0x0246, 0x3},	// None
	{0x0247, 0xD},	// None
	{0x0248, 0x0},	// None
	{0x0249, 0x7},	// None
	{0x024A, 0x3},	// None
	{0x024B, 0xEF}, // None
	{0x024C, 0x3},	// None
	{0x024D, 0xF3}, // None
	{0x024E, 0x3},	// None
	{0x024F, 0xF4}, // None
	{0x0250, 0x3},	// None
	{0x0251, 0x0},	// None
	{0x0252, 0x0},	// None
	{0x0253, 0x7},	// None
	{0x0254, 0x0},	// None
	{0x0255, 0xC},	// None
	{0x0256, 0x1},	// None
	{0x0257, 0xF1}, // None
	{0x0258, 0x3},	// None
	{0x0259, 0x43}, // None
	{0x025A, 0x1},	// None
	{0x025B, 0xF8}, // None
	{0x025C, 0x3},	// None
	{0x025D, 0x10}, // None
	{0x025E, 0x0},	// None
	{0x025F, 0x7},	// None
	{0x0260, 0x3},	// None
	{0x0261, 0xF0}, // None
	{0x0262, 0x3},	// None
	{0x0263, 0xF1}, // None
	{0x0264, 0x3},	// None
	{0x0265, 0xF2}, // None
	{0x0266, 0x3},	// None
	{0x0267, 0xF5}, // None
	{0x0268, 0x3},	// None
	{0x0269, 0xF6}, // None
	{0x026A, 0x3},	// None
	{0x026B, 0x0},	// None
	{0x026C, 0x2},	// None
	{0x026D, 0x87}, // None
	{0x026E, 0x0},	// None
	{0x026F, 0x1},	// None
	{0x0270, 0x3},	// None
	{0x0271, 0xFF}, // None
	{0x0272, 0x3},	// None
	{0x0273, 0x0},	// None
	{0x0274, 0x3},	// None
	{0x0275, 0xFF}, // None
	{0x0276, 0x2},	// None
	{0x0277, 0x87}, // None
	{0x0278, 0x3},	// None
	{0x0279, 0x2},	// None
	{0x027A, 0x3},	// None
	{0x027B, 0xF},	// None
	{0x027C, 0x3},	// None
	{0x027D, 0xF7}, // None
	{0x027E, 0x0},	// None
	{0x027F, 0x16}, // None
	{0x0280, 0x0},	// None
	{0x0281, 0x33}, // None
	{0x0282, 0x0},	// None
	{0x0283, 0x4},	// None
	{0x0284, 0x0},	// None
	{0x0285, 0x11}, // None
	{0x0286, 0x3},	// None
	{0x0287, 0x9},	// None
	{0x0288, 0x0},	// None
	{0x0289, 0x2},	// None
	{0x028A, 0x0},	// None
	{0x028B, 0x20}, // None
	{0x028C, 0x0},	// None
	{0x028D, 0xB5}, // None
	{0x028E, 0x0},	// None
	{0x028F, 0xE5}, // None
	{0x0290, 0x0},	// None
	{0x0291, 0x12}, // None
	{0x0292, 0x0},	// None
	{0x0293, 0xB5}, // None
	{0x0294, 0x0},	// None
	{0x0295, 0xE5}, // None
	{0x0296, 0x0},	// None
	{0x0297, 0x10}, // None
	{0x0298, 0x0},	// None
	{0x0299, 0x2},	// None
	{0x029A, 0x0},	// None
	{0x029B, 0x20}, // None
	{0x029C, 0x0},	// None
	{0x029D, 0xB5}, // None
	{0x029E, 0x0},	// None
	{0x029F, 0xE5}, // None
	{0x02A0, 0x0},	// None
	{0x02A1, 0x12}, // None
	{0x02A2, 0x0},	// None
	{0x02A3, 0xB5}, // None
	{0x02A4, 0x0},	// None
	{0x02A5, 0xE5}, // None
	{0x02A6, 0x0},	// None
	{0x02A7, 0x0},	// None
	{0x02A8, 0x0},	// None
	{0x02A9, 0x12}, // None
	{0x02AA, 0x0},	// None
	{0x02AB, 0x12}, // None
	{0x02AC, 0x0},	// None
	{0x02AD, 0x20}, // None
	{0x02AE, 0x0},	// None
	{0x02AF, 0xB5}, // None
	{0x02B0, 0x0},	// None
	{0x02B1, 0xE5}, // None
	{0x02B2, 0x0},	// None
	{0x02B3, 0x0},	// None
	{0x02B4, 0x0},	// None
	{0x02B5, 0x12}, // None
	{0x02B6, 0x0},	// None
	{0x02B7, 0x12}, // None
	{0x02B8, 0x0},	// None
	{0x02B9, 0x20}, // None
	{0x02BA, 0x0},	// None
	{0x02BB, 0x47}, // None
	{0x02BC, 0x0},	// None
	{0x02BD, 0x27}, // None
	{0x02BE, 0x0},	// None
	{0x02BF, 0xB5}, // None
	{0x02C0, 0x0},	// None
	{0x02C1, 0xE5}, // None
	{0x02C2, 0x0},	// None
	{0x02C3, 0x0},	// None
	{0x02C4, 0x0},	// None
	{0x02C5, 0x4},	// None
	{0x02C6, 0x0},	// None
	{0x02C7, 0x43}, // None
	{0x02C8, 0x0},	// None
	{0x02C9, 0x1},	// None
	{0x02CA, 0x3},	// None
	{0x02CB, 0x2},	// None
	{0x02CC, 0x0},	// None
	{0x02CD, 0x8},	// None
	{0x02CE, 0x3},	// None
	{0x02CF, 0xFF}, // None
	{0x02D0, 0x2},	// None
	{0x02D1, 0x87}, // None
	{0x02D2, 0x3},	// None
	{0x02D3, 0xC7}, // None
	{0x02D4, 0x3},	// None
	{0x02D5, 0xF7}, // None
	{0x02D6, 0x0},	// None
	{0x02D7, 0x77}, // None
	{0x02D8, 0x0},	// None
	{0x02D9, 0x17}, // None
	{0x02DA, 0x0},	// None
	{0x02DB, 0x8},	// None
	{0x02DC, 0x3},	// None
	{0x02DD, 0xFF}, // None
	{0x02DE, 0x0},	// None
	{0x02DF, 0x38}, // None
	{0x02E0, 0x0},	// None
	{0x02E1, 0x17}, // None
	{0x02E2, 0x0},	// None
	{0x02E3, 0x8},	// None
	{0x02E4, 0x3},	// None
	{0x02E5, 0xFF}, // None
	{0x02E6, 0x3},	// None
	{0x02E7, 0xFF}, // None
	{0x02E8, 0x3},	// None
	{0x02E9, 0xFF}, // None
	{0x02EA, 0x3},	// None
	{0x02EB, 0xFF}, // None
	{0x02EC, 0x3},	// None
	{0x02ED, 0xFF}, // None
	{0x02EE, 0x3},	// None
	{0x02EF, 0xFF}, // None
	{0x02F0, 0x3},	// None
	{0x02F1, 0xFF}, // None
	{0x02F2, 0x3},	// None
	{0x02F3, 0xFF}, // None
	{0x02F4, 0x3},	// None
	{0x02F5, 0xFF}, // None
	{0x02F6, 0x3},	// None
	{0x02F7, 0xFF}, // None
	{0x02F8, 0x3},	// None
	{0x02F9, 0xFF}, // None
	{0x02FA, 0x3},	// None
	{0x02FB, 0xFF}, // None
	{0x02FC, 0x3},	// None
	{0x02FD, 0xFF}, // None
	{0x02FE, 0x3},	// None
	{0x02FF, 0xFF}, // None
	{0x0300, 0x3},	// None
	{0x0301, 0xFF}, // None
	{0x0302, 0x3},	// None
	{0x0303, 0xFF}, // None
	{0x01E9, 0x0},	// None
	{0x01E8, 0x19}, // None
	{0x01EA, 0x35}, // None
	{0x01EB, 0x37}, // None
	{0x01EC, 0x64}, // None
	{0x01ED, 0x6B}, // None
	{0x01F8, 0xF},	// None
	{0x01D8, 0x1},	// None
	{0x01DC, 0x1},	// None
	{0x01DE, 0x1},	// None
	{0x0189, 0x1},	// None
	{0x01B7, 0x1},	// None
	{0x01C1, 0x7},	// None
	{0x01C2, 0xF6}, // None
	{0x01C3, 0xFF}, // None
	{0x01C9, 0x7},	// None
	{0x0325, 0x0},	// None
	{0xE159, 0x0},	// None
	{0x033A, 0x0},	// None
	{0x01B8, 0x1},	// None
	{0x01BA, 0x33}, // None
	{0x01BE, 0x74}, // None
	{0x01BF, 0x36}, // None
	{0x01C0, 0x53}, // None
	{0x00EF, 0x0},	// None
	{0x0326, 0x0},	// None
	{0x00F0, 0x0},	// None
	{0x00F1, 0x0},	// None
	{0x0327, 0x0},	// None
	{0x00F2, 0x0},	// None
	{0x0071, 0x1},	// None
	{0x01B4, 0x1},	// None
	{0x01B5, 0x1},	// None
	{0x01F1, 0x1},	// None
	{0x01F4, 0x1},	// None
	{0x01F5, 0x1},	// None
	{0x0314, 0x1},	// None
	{0x0315, 0x1},	// None
	{0x0316, 0x1},	// None
	{0x0207, 0x0},	// None
	{0x4207, 0x2},	// None
	{0x2207, 0x2},	// None
	{0xE088, 0x1},	// None
	{0xE08D, 0x0},	// None
	{0xE08E, 0x30}, // None
	{0xE08F, 0xD4}, // None
	{0xE089, 0x56}, // None
	{0xE08A, 0x10}, // None
	{0xE08B, 0x1F}, // None
	{0xE08C, 0xF},	// None
	{0xE0A6, 0x0},	// None
	{0xE0A9, 0x10}, // None
	{0xE0AA, 0x0},	// None
	{0xE0AD, 0xA},	// None
	{0xE0A8, 0x30}, // None
	{0xE0A7, 0xF},	// None
	{0xE0AC, 0x10}, // None
	{0xE0AB, 0xF},	// None
	{0x209D, 0x0},	// None
	{0x0328, 0x0},	// None
	{0x0063, 0x1},	// None
	{0x01F7, 0xF},	// None
	{0xE0E3, 0x1},	// None
	{0xE0E7, 0x3},	// None
	{0xE33B, 0x0},	// None
	{0xE336, 0x0},	// None
	{0xE337, 0x0},	// None
	{0xE338, 0x0},	// None
	{0xE339, 0x0},	// None
	{0x00E9, 0x1},	// None
	{0x00EA, 0xFE}, // None
	{0x0309, 0x3},	// None
	{0x030A, 0x2},	// None
	{0x030B, 0x2},	// None
	{0x030C, 0x5},	// None
	{0x030E, 0x15}, // None
	{0x030D, 0x14}, // None
	{0x030F, 0x1},	// None
	{0x0310, 0xD},	// None
	{0x01D0, 0x1F}, // None
	{0x01D1, 0x1F}, // None
	{0x01CD, 0x11}, // None
	{0x0016, 0x0},	// None
	{0x0017, 0x5},	// None
	{0x01F2, 0x0},	// None
	{0x016A, 0x1},	// None
	{0xE0C0, 0x0},	// None
	{0xE0C1, 0x10}, // None
	{0xE0C2, 0x0},	// None
	{0xE0C3, 0x10}, // None
	{0x0168, 0x2B}, // None
	{0xE000, 0x0},	// None
	{0xE0A2, 0x0},	// None
	{0xE07B, 0x0},	// None
	{0xE078, 0x0},	// None
	{0xE079, 0x1},	// None
	{0x2077, 0x0},	// None
	{0x2076, 0x93}, // None
	{0x00CE, 0x1},	// None
	{0x0070, 0x6},	// None
	{0x016D, 0x22}, // None
	{0x0176, 0x42}, // None
	{0xE136, 0x0},	// None
	{0xE0C6, 0x0},	// None
	{0xE0C7, 0x0},	// None
	{0xE0C8, 0x1},	// None
	{0xE0C9, 0x0},	// None
	{0xE0CA, 0x0},	// None
	{0xE0CB, 0x1},	// None
	{0xE0CC, 0x80}, // None
	{0xE0CD, 0x80}, // None
	{0xE0BE, 0x2},	// None
	{0xE0BF, 0x2},	// None
	{0xE0C4, 0x8},	// None
	{0xE0C5, 0x8},	// None
	{0x2075, 0x0},	// None
	{0xE000, 0x0},	// None
	{0x001E, 0x1},	// None
	{0xE000, 0x0},	// None
	{0x207E, 0x0},	// None
	{0xE084, 0x0},	// None
	{0xE085, 0x0},	// None
	{0xE086, 0x1},	// None
	{0xE087, 0x0},	// None
	{0x207F, 0x0},	// None
	{0x2080, 0x0},	// None
	{0x2081, 0x3},	// None
	{0x2082, 0x0},	// None
	{0x2083, 0x2},	// None
	{0x0090, 0x0},	// None
	{0x2097, 0x0},	// None
	{0xE000, 0x0},	// None
	{0x0011, 0x3},	// None
	{0x011D, 0x0},	// None
	{0xE000, 0x0},	// None
	{0x0012, 0x0},	// None
	{0x0013, 0x18}, // None
	{0x015A, 0x0},	// None
	{0x015B, 0x57}, // None
	{0x015C, 0x0},	// None
	{0x015D, 0x57}, // None
	{0x015E, 0x0},	// None
	{0x015F, 0x57}, // None
	{0x0162, 0x0},	// None
	{0x0163, 0x5},	// None
	{0x0164, 0x4},	// None
	{0x0165, 0x79}, // None
	{0x0166, 0x4},	// None
	{0x0167, 0x79}, // None
	{0xE000, 0x0},	// None
	{0x01BB, 0xB4}, // None
	{0x01BC, 0xAC}, // None
	{0x00D0, 0x0},	// None
	{0x016E, 0xFD}, // None
	{0x0172, 0x0},	// None
	{0x0173, 0x0},	// None
	{0x016F, 0x7E}, // None
	{0x0170, 0x0},	// None
	{0x0171, 0xFD}, // None
	{0x0174, 0x0},	// None
	{0x0175, 0x0},	// None
	{0x0177, 0x78}, // None
	{0x018B, 0x4},	// None
	{0x018C, 0x4},	// None
	{0x018D, 0x2},	// None
	{0x018E, 0x60}, // None
	{0x018F, 0x6},	// None
	{0x0190, 0x4},	// None
	{0x00E8, 0x8},	// None
	{0xE000, 0x1},	// None
	{0xE000, 0x1},	// None
	{0xE024, 0xF},	// None
	{0xE000, 0x0},	// None
	{0xE000, 0x0},	// None
	{0x005C, 0x0},	// None
	{0x005D, 0x18}, // None
	{0x01EE, 0x1B}, // None
	{0x01EF, 0x46}, // None
	{0x01A2, 0x0},	// None
	{0x01A3, 0x1},	// None
	{0x031F, 0x0},	// None
	{0x0320, 0xA},	// None
	{0x01A6, 0x0},	// None
	{0x01A7, 0x98}, // None
	{0x01A4, 0x4},	// None
	{0x01A5, 0x27}, // None
	{0x0321, 0x4},	// None
	{0x0322, 0x30}, // None
	{0x01A8, 0x4},	// None
	{0x01A9, 0xBE}, // None
	{0x01A0, 0x0},	// None
	{0x01A1, 0xFB}, // None
	{0x01B2, 0x1},	// None
	{0x01B3, 0x15}, // None
	{0x01B0, 0x1},	// None
	{0x01B1, 0xE},	// None
	{0x01AC, 0x1},	// None
	{0x01AD, 0x19}, // None
	{0x01F0, 0x24}, // None
	{0x01F3, 0x2},	// None
	{0xE000, 0x0},	// None
	{0x0193, 0x8},	// None
	{0x0194, 0x4},	// None
	{0xE32C, 0x1},	// None
	{0xE32D, 0x0},	// None
	{0xE14E, 0x0},	// None
	{0xE312, 0x7F}, // None
	{0xE329, 0x1},	// None
	{0xE32B, 0x0},	// None
	{0xE331, 0xF},	// None
	{0xE332, 0x3},	// None
	{0xE32A, 0x0},	// None
	{0xE11E, 0x1},	// None
	{0xE000, 0x0},	// None
	{0xE009, 0x1},	// None
	{0x212F, 0x1},	// None
	{0x2130, 0x1},	// None
	{0x2131, 0x1},	// None
	{0x2132, 0x1},	// None
	{0x2133, 0x1},	// None
	{0x2134, 0x1},	// None
	{0x2135, 0x1},	// None
	{0xE0E1, 0x1},	// None
	{0x018A, 0x1},	// None
	{0x00E0, 0x1},	// None
	{0xE32E, 0x1},	// None
	{0xE340, 0x1},	// None
	{0xE004, 0x0},	// None
	{0xE000, 0x1},	// None
	{0xE02C, 0x0},	// None
	{0xE02D, 0xE},	// None
	{0xE02E, 0x1},	// None
	{0xE02F, 0x9D}, // None
	{0xE030, 0x0},	// None
	{0xE025, 0x0},	// None
	{0xE02A, 0x0},	// None
	{0x2029, 0x28}, // None
	{0x0034, 0x0},	// None
	{0x0035, 0xC8}, // None
	{0xE004, 0x1},	// None
	{0xE02C, 0x0},	// None
	{0xE02D, 0xE},	// None
	{0xE02E, 0x1},	// None
	{0xE02F, 0x9D}, // None
	{0xE030, 0x0},	// None
	{0xE025, 0x0},	// None
	{0xE02A, 0x0},	// None
	{0x2029, 0x28}, // None
	{0x0034, 0x0},	// None
	{0x0035, 0xC8}, // None
	{0xE000, 0x1},	// None
	{0xE004, 0x0},	// None
	{0xE000, 0x1},	// None
	{0x001E, 0x0},	// None
	{0x001F, 0x2},	// None
	{0x002B, 0x0},	// None
	{0xE004, 0x1},	// None
	{0x001E, 0x0},	// None
	{0x001F, 0x2},	// None
	{0x002B, 0x0},	// None
	{0xE000, 0x0},	// None
	{0xE000, 0x0},	// None
	{0x001F, 0x0},	// None
	{0x0020, 0x0},	// None
	{0x0023, 0x0},	// None
	{0x0024, 0x1},	// None
	{0x0025, 0x90}, // None
	{0x0026, 0x0},	// None
	{0x0027, 0xE},	// None
	{0x0028, 0x0},	// None
	{0x0029, 0x1},	// None
	{0x002A, 0x90}, // None
	{0x002B, 0x0},	// None
	{0x002C, 0xE},	// None
	{0x002D, 0x0},	// None
	{0x002E, 0x0},	// None
	{0x002F, 0x0},	// None
	{0x0030, 0x0},	// None
	{0x0031, 0x0},	// None
	{0x0032, 0x0},	// None
	{0x0033, 0x0},	// None
	{0x0034, 0x0},	// None
	{0x0035, 0x0},	// None
	{0x0036, 0x0},	// None
	{0x0037, 0x0},	// None
	{0x0038, 0x0},	// None
	{0x0039, 0x0},	// None
	{0x003A, 0x0},	// None
	{0x003B, 0x0},	// None
	{0x003C, 0x0},	// None
	{0x003D, 0x0},	// None
	{0x003E, 0x0},	// None
	{0x003F, 0x0},	// None
	{0x0040, 0x0},	// None
	{0x0041, 0x0},	// None
	{0x0042, 0x0},	// None
	{0x0043, 0x0},	// None
	{0x0044, 0x0},	// None
	{0x0045, 0x0},	// None
	{0x0046, 0x0},	// None
	{0x0047, 0x0},	// None
	{0x0048, 0x0},	// None
	{0x0049, 0x0},	// None
	{0x004A, 0x0},	// None
	{0x004B, 0x0},	// None
	{0x004C, 0x0},	// None
	{0x004D, 0x0},	// None
	{0x004E, 0x0},	// None
	{0x004F, 0x0},	// None
	{0x0050, 0x0},	// None
	{0x0051, 0x0},	// None
	{0x0052, 0x0},	// None
	{0x0053, 0x0},	// None
	{0x0054, 0x0},	// None
	{0x0055, 0x0},	// None
	{0xE004, 0x0},	// None
	{0xE000, 0x1},	// None
	{0x000E, 0x0},	// None
	{0x000F, 0x0},	// None
	{0x0010, 0x3},	// None
	{0x0011, 0xE8}, // None
	{0x0012, 0x0},	// None
	{0x0013, 0x0},	// None
	{0x0014, 0x0},	// None
	{0x0015, 0x0},	// None
	{0xE004, 0x1},	// None
	{0x000E, 0x0},	// None
	{0x000F, 0x0},	// None
	{0x0010, 0x3},	// None
	{0x0011, 0xE8}, // None
	{0x0012, 0x0},	// None
	{0x0013, 0x0},	// None
	{0x0014, 0x0},	// None
	{0x0015, 0x0},	// None
	{0xE004, 0x0},	// None
	{0x0032, 0x4},	// None
	{0x0033, 0xEE}, // None
	{0xE004, 0x1},	// None
	{0x0032, 0x4},	// None
	{0x0033, 0xEE}, // None
	{0xE004, 0x0},	// None
	{0x0007, 0x1},	// None
	{0x0008, 0x0},	// None
	{0x0009, 0x0},	// None
	{0x000A, 0x41}, // None
	{0x000B, 0x1A}, // None
	{0xE004, 0x1},	// None
	{0x0007, 0x1},	// None
	{0x0008, 0x0},	// None
	{0x0009, 0x0},	// None
	{0x000A, 0x17}, // None
	{0x000B, 0x70}, // None
	{0xE000, 0x0},	// None
	{0x0057, 0x0},	// None
	{0x0058, 0x0},	// None
	{0x0059, 0x2},	// None
	{0x005A, 0x2},	// None
	{0x005B, 0x0},	// None
	{0xE000, 0x0},	// None
	{0xE008, 0x0},	// None
	{0x0006, 0x1},	// None
	{0xE003, 0x0},	// None
	{0x0006, 0x0},	// None
	{0xE008, 0x0},	// None
	{0xE004, 0x0},	// None
	{0xE000, 0x1},	// None
	{0x0031, 0x0},	// None
	{0xE004, 0x1},	// None
	{0x0031, 0x0},	// None
	{0xE000, 0x0},	// None
	{0x0138, 0x0},	// None
	{0xE005, 0x0},	// None
	{0x0139, 0x0},	// None
	{0x013A, 0x0},	// None
	{0x013B, 0x64}, // None
	{0x013C, 0x0},	// None
	{0x013D, 0x0},	// None
	{0x013E, 0x64}, // None
	{0x013F, 0x6},	// None
	{0x0140, 0x1},	// None
	{0x0141, 0x10}, // None
	{0x0142, 0x1},	// None
	{0x0143, 0x0},	// None
	{0x0144, 0x0},	// None
	{0x0146, 0x0},	// None
	{0x0147, 0x0},	// None
	{0x0148, 0x0},	// None
	{0xE004, 0x0},	// None
	{0xE000, 0x1},	// None
	{0x0026, 0x0},	// None
	{0xE004, 0x1},	// None
	{0x0026, 0x0},	// None
	{0xE000, 0x0},	// None
	{0x0169, 0x12}, // None
	{0xE004, 0x0},	// None
	{0xE000, 0x1},	// None
	{0x001C, 0x1},	// None
	{0x0019, 0x0},	// None
	{0x001A, 0x7},	// None
	{0x001B, 0x53}, // None
	{0x0016, 0x8},	// None
	{0x0017, 0x0},	// None
	{0x0018, 0x0},	// None
	{0xE004, 0x1},	// None
	{0x001C, 0x0},	// None
	{0x0019, 0x0},	// None
	{0x001A, 0x7},	// None
	{0x001B, 0x53}, // None
	{0x0016, 0x8},	// None
	{0x0017, 0x0},	// None
	{0x0018, 0x0},	// None
	{0xE004, 0x0},	// None
	{0xE000, 0x1},	// None
	{0x001D, 0x1},	// None
	{0xE004, 0x1},	// None
	{0x001D, 0x0},	// None
	{0xE000, 0x0},	// None
	{0x001A, 0x0},	// None
	{0x001B, 0x0},	// None
	{0x001C, 0x0},	// None
	{0xE000, 0x0},	// None
	{0xE000, 0x1},	// bank1: below is fpn fix.
	{0xE004, 0x0},	// context0
	{0x0032,0x9}, //rowlen
	{0x0033,0x58},
	{0xE000, 0x0},	// None
	{0xE004, 0x0},	// context0
	{0x01A2,0x4},
	{0x01A3,0x6B},
	{0x01A4,0x8},
	{0x01A5,0x91},
	{0x01A6,0x5},
	{0x01A7,0x02},
	{0x01A8,0x9},
	{0x01A9,0x28},

};

static const struct mira016_reg full_400_400_100fps_10b_1lane_reg_post_soft_reset[] = {
	{0xE000, 0},
	{0xE004, 0},
	// Below are manually added after reg seq txt
	{0x0335, 1},  // iref sel
	{0x0324, 43}, // iref val

	{0x1d9, 1},	 // #vddana sel
	{0x0EB, 15}, // #vddana val trim

	{0x1dd, 1},	 // # vsspc sel
	{0x1ed, 14}, // # vsspc val

	{0x1df, 1}, // #cp sel
	{0x0ee, 4}, // #cp trim,
};

static const struct mira016_reg full_400_400_100fps_12b_1lane_reg_pre_soft_reset[] = {
	// Mira016_register_sequence_12b_1x_60fps_1000M.txt
	{0xE000, 0x0},	// None
	{0x01E4, 0x0},	// None
	{0x01E5, 0x13}, // None
	{0x01E2, 0x17}, // None
	{0x01E3, 0xA8}, // None
	{0x01E6, 0x0},	// None
	{0x01E7, 0xCA}, // None
	{0x016C, 0x1},	// None
	{0x016B, 0x1},	// None
	{0x0208, 0x1},	// None
	{0x0209, 0xF0}, // None
	{0x020A, 0x3},	// None
	{0x020B, 0x4D}, // None
	{0x020C, 0x2},	// None
	{0x020D, 0x10}, // None
	{0x020E, 0x3},	// None
	{0x020F, 0x1},	// None
	{0x0210, 0x0},	// None
	{0x0211, 0x13}, // None
	{0x0212, 0x0},	// None
	{0x0213, 0x3},	// None
	{0x0214, 0x3},	// None
	{0x0215, 0xEF}, // None
	{0x0216, 0x3},	// None
	{0x0217, 0xF3}, // None
	{0x0218, 0x3},	// None
	{0x0219, 0xF4}, // None
	{0x021A, 0x0},	// None
	{0x021B, 0x1},	// None
	{0x021C, 0x3},	// None
	{0x021D, 0xF8}, // None
	{0x021E, 0x0},	// None
	{0x021F, 0x2},	// None
	{0x0220, 0x1},	// None
	{0x0221, 0xF2}, // None
	{0x0222, 0x3},	// None
	{0x0223, 0x1B}, // None
	{0x0224, 0x0},	// None
	{0x0225, 0x21}, // None
	{0x0226, 0x3},	// None
	{0x0227, 0xF0}, // None
	{0x0228, 0x3},	// None
	{0x0229, 0xF1}, // None
	{0x022A, 0x3},	// None
	{0x022B, 0xF2}, // None
	{0x022C, 0x3},	// None
	{0x022D, 0xF5}, // None
	{0x022E, 0x3},	// None
	{0x022F, 0xF6}, // None
	{0x0230, 0x0},	// None
	{0x0231, 0xC1}, // None
	{0x0232, 0x0},	// None
	{0x0233, 0x2},	// None
	{0x0234, 0x1},	// None
	{0x0235, 0xF2}, // None
	{0x0236, 0x3},	// None
	{0x0237, 0x6B}, // None
	{0x0238, 0x3},	// None
	{0x0239, 0xFF}, // None
	{0x023A, 0x3},	// None
	{0x023B, 0x31}, // None
	{0x023C, 0x1},	// None
	{0x023D, 0xF0}, // None
	{0x023E, 0x3},	// None
	{0x023F, 0x87}, // None
	{0x0240, 0x0},	// None
	{0x0241, 0xA},	// None
	{0x0242, 0x0},	// None
	{0x0243, 0xB},	// None
	{0x0244, 0x1},	// None
	{0x0245, 0xF9}, // None
	{0x0246, 0x3},	// None
	{0x0247, 0xD},	// None
	{0x0248, 0x0},	// None
	{0x0249, 0x7},	// None
	{0x024A, 0x3},	// None
	{0x024B, 0xEF}, // None
	{0x024C, 0x3},	// None
	{0x024D, 0xF3}, // None
	{0x024E, 0x3},	// None
	{0x024F, 0xF4}, // None
	{0x0250, 0x3},	// None
	{0x0251, 0x0},	// None
	{0x0252, 0x0},	// None
	{0x0253, 0x7},	// None
	{0x0254, 0x0},	// None
	{0x0255, 0xC},	// None
	{0x0256, 0x1},	// None
	{0x0257, 0xF1}, // None
	{0x0258, 0x3},	// None
	{0x0259, 0x43}, // None
	{0x025A, 0x1},	// None
	{0x025B, 0xF8}, // None
	{0x025C, 0x3},	// None
	{0x025D, 0x10}, // None
	{0x025E, 0x0},	// None
	{0x025F, 0x7},	// None
	{0x0260, 0x3},	// None
	{0x0261, 0xF0}, // None
	{0x0262, 0x3},	// None
	{0x0263, 0xF1}, // None
	{0x0264, 0x3},	// None
	{0x0265, 0xF2}, // None
	{0x0266, 0x3},	// None
	{0x0267, 0xF5}, // None
	{0x0268, 0x3},	// None
	{0x0269, 0xF6}, // None
	{0x026A, 0x3},	// None
	{0x026B, 0x0},	// None
	{0x026C, 0x2},	// None
	{0x026D, 0x87}, // None
	{0x026E, 0x0},	// None
	{0x026F, 0x1},	// None
	{0x0270, 0x3},	// None
	{0x0271, 0xFF}, // None
	{0x0272, 0x3},	// None
	{0x0273, 0x0},	// None
	{0x0274, 0x3},	// None
	{0x0275, 0xFF}, // None
	{0x0276, 0x2},	// None
	{0x0277, 0x87}, // None
	{0x0278, 0x3},	// None
	{0x0279, 0x2},	// None
	{0x027A, 0x3},	// None
	{0x027B, 0xF},	// None
	{0x027C, 0x3},	// None
	{0x027D, 0xF7}, // None
	{0x027E, 0x0},	// None
	{0x027F, 0x16}, // None
	{0x0280, 0x0},	// None
	{0x0281, 0x33}, // None
	{0x0282, 0x0},	// None
	{0x0283, 0x4},	// None
	{0x0284, 0x0},	// None
	{0x0285, 0x11}, // None
	{0x0286, 0x3},	// None
	{0x0287, 0x9},	// None
	{0x0288, 0x0},	// None
	{0x0289, 0x2},	// None
	{0x028A, 0x0},	// None
	{0x028B, 0x20}, // None
	{0x028C, 0x0},	// None
	{0x028D, 0xB5}, // None
	{0x028E, 0x0},	// None
	{0x028F, 0xE5}, // None
	{0x0290, 0x0},	// None
	{0x0291, 0x12}, // None
	{0x0292, 0x0},	// None
	{0x0293, 0xB5}, // None
	{0x0294, 0x0},	// None
	{0x0295, 0xE5}, // None
	{0x0296, 0x0},	// None
	{0x0297, 0x10}, // None
	{0x0298, 0x0},	// None
	{0x0299, 0x2},	// None
	{0x029A, 0x0},	// None
	{0x029B, 0x20}, // None
	{0x029C, 0x0},	// None
	{0x029D, 0xB5}, // None
	{0x029E, 0x0},	// None
	{0x029F, 0xE5}, // None
	{0x02A0, 0x0},	// None
	{0x02A1, 0x12}, // None
	{0x02A2, 0x0},	// None
	{0x02A3, 0xB5}, // None
	{0x02A4, 0x0},	// None
	{0x02A5, 0xE5}, // None
	{0x02A6, 0x0},	// None
	{0x02A7, 0x0},	// None
	{0x02A8, 0x0},	// None
	{0x02A9, 0x12}, // None
	{0x02AA, 0x0},	// None
	{0x02AB, 0x12}, // None
	{0x02AC, 0x0},	// None
	{0x02AD, 0x20}, // None
	{0x02AE, 0x0},	// None
	{0x02AF, 0xB5}, // None
	{0x02B0, 0x0},	// None
	{0x02B1, 0xE5}, // None
	{0x02B2, 0x0},	// None
	{0x02B3, 0x0},	// None
	{0x02B4, 0x0},	// None
	{0x02B5, 0x12}, // None
	{0x02B6, 0x0},	// None
	{0x02B7, 0x12}, // None
	{0x02B8, 0x0},	// None
	{0x02B9, 0x20}, // None
	{0x02BA, 0x0},	// None
	{0x02BB, 0x47}, // None
	{0x02BC, 0x0},	// None
	{0x02BD, 0x27}, // None
	{0x02BE, 0x0},	// None
	{0x02BF, 0xB5}, // None
	{0x02C0, 0x0},	// None
	{0x02C1, 0xE5}, // None
	{0x02C2, 0x0},	// None
	{0x02C3, 0x0},	// None
	{0x02C4, 0x0},	// None
	{0x02C5, 0x4},	// None
	{0x02C6, 0x0},	// None
	{0x02C7, 0x43}, // None
	{0x02C8, 0x0},	// None
	{0x02C9, 0x1},	// None
	{0x02CA, 0x3},	// None
	{0x02CB, 0x2},	// None
	{0x02CC, 0x0},	// None
	{0x02CD, 0x8},	// None
	{0x02CE, 0x3},	// None
	{0x02CF, 0xFF}, // None
	{0x02D0, 0x2},	// None
	{0x02D1, 0x87}, // None
	{0x02D2, 0x3},	// None
	{0x02D3, 0xC7}, // None
	{0x02D4, 0x3},	// None
	{0x02D5, 0xF7}, // None
	{0x02D6, 0x0},	// None
	{0x02D7, 0x77}, // None
	{0x02D8, 0x0},	// None
	{0x02D9, 0x17}, // None
	{0x02DA, 0x0},	// None
	{0x02DB, 0x8},	// None
	{0x02DC, 0x3},	// None
	{0x02DD, 0xFF}, // None
	{0x02DE, 0x0},	// None
	{0x02DF, 0x38}, // None
	{0x02E0, 0x0},	// None
	{0x02E1, 0x17}, // None
	{0x02E2, 0x0},	// None
	{0x02E3, 0x8},	// None
	{0x02E4, 0x3},	// None
	{0x02E5, 0xFF}, // None
	{0x02E6, 0x3},	// None
	{0x02E7, 0xFF}, // None
	{0x02E8, 0x3},	// None
	{0x02E9, 0xFF}, // None
	{0x02EA, 0x3},	// None
	{0x02EB, 0xFF}, // None
	{0x02EC, 0x3},	// None
	{0x02ED, 0xFF}, // None
	{0x02EE, 0x3},	// None
	{0x02EF, 0xFF}, // None
	{0x02F0, 0x3},	// None
	{0x02F1, 0xFF}, // None
	{0x02F2, 0x3},	// None
	{0x02F3, 0xFF}, // None
	{0x02F4, 0x3},	// None
	{0x02F5, 0xFF}, // None
	{0x02F6, 0x3},	// None
	{0x02F7, 0xFF}, // None
	{0x02F8, 0x3},	// None
	{0x02F9, 0xFF}, // None
	{0x02FA, 0x3},	// None
	{0x02FB, 0xFF}, // None
	{0x02FC, 0x3},	// None
	{0x02FD, 0xFF}, // None
	{0x02FE, 0x3},	// None
	{0x02FF, 0xFF}, // None
	{0x0300, 0x3},	// None
	{0x0301, 0xFF}, // None
	{0x0302, 0x3},	// None
	{0x0303, 0xFF}, // None
	{0x01E9, 0x0},	// None
	{0x01E8, 0x19}, // None
	{0x01EA, 0x35}, // None
	{0x01EB, 0x37}, // None
	{0x01EC, 0x64}, // None
	{0x01ED, 0x6B}, // None
	{0x01F8, 0xF},	// None
	{0x01D8, 0x1},	// None
	{0x01DC, 0x1},	// None
	{0x01DE, 0x1},	// None
	{0x0189, 0x1},	// None
	{0x01B7, 0x1},	// None
	{0x01C1, 0x7},	// None
	{0x01C2, 0xF6}, // None
	{0x01C3, 0xFF}, // None
	{0x01C9, 0x7},	// None
	{0x0325, 0x0},	// None
	{0xE159, 0x0},	// None
	{0x033A, 0x0},	// None
	{0x01B8, 0x1},	// None
	{0x01BA, 0x33}, // None
	{0x01BE, 0x74}, // None
	{0x01BF, 0x36}, // None
	{0x01C0, 0x53}, // None
	{0x00EF, 0x0},	// None
	{0x0326, 0x0},	// None
	{0x00F0, 0x0},	// None
	{0x00F1, 0x0},	// None
	{0x0327, 0x0},	// None
	{0x00F2, 0x0},	// None
	{0x0071, 0x1},	// None
	{0x01B4, 0x1},	// None
	{0x01B5, 0x1},	// None
	{0x01F1, 0x1},	// None
	{0x01F4, 0x1},	// None
	{0x01F5, 0x1},	// None
	{0x0314, 0x1},	// None
	{0x0315, 0x1},	// None
	{0x0316, 0x1},	// None
	{0x0207, 0x0},	// None
	{0x4207, 0x2},	// None
	{0x2207, 0x2},	// None
	{0xE088, 0x1},	// None
	{0xE08D, 0x0},	// None
	{0xE08E, 0x30}, // None
	{0xE08F, 0xD4}, // None
	{0xE089, 0x56}, // None
	{0xE08A, 0x10}, // None
	{0xE08B, 0x1F}, // None
	{0xE08C, 0xF},	// None
	{0xE0A6, 0x0},	// None
	{0xE0A9, 0x10}, // None
	{0xE0AA, 0x0},	// None
	{0xE0AD, 0xA},	// None
	{0xE0A8, 0x30}, // None
	{0xE0A7, 0xF},	// None
	{0xE0AC, 0x10}, // None
	{0xE0AB, 0xF},	// None
	{0x209D, 0x0},	// None
	{0x0328, 0x0},	// None
	{0x0063, 0x1},	// None
	{0x01F7, 0xF},	// None
	{0xE0E3, 0x1},	// None
	{0xE0E7, 0x3},	// None
	{0xE33B, 0x0},	// None
	{0xE336, 0x0},	// None
	{0xE337, 0x0},	// None
	{0xE338, 0x0},	// None
	{0xE339, 0x0},	// None
	{0x00E9, 0x1},	// None
	{0x00EA, 0xFE}, // None
	{0x0309, 0x3},	// None
	{0x030A, 0x2},	// None
	{0x030B, 0x2},	// None
	{0x030C, 0x5},	// None
	{0x030E, 0x15}, // None
	{0x030D, 0x14}, // None
	{0x030F, 0x1},	// None
	{0x0310, 0xD},	// None
	{0x01D0, 0x1F}, // None
	{0x01D1, 0x1F}, // None
	{0x01CD, 0x11}, // None
	{0x0016, 0x0},	// None
	{0x0017, 0x5},	// None
	{0x01F2, 0x0},	// None
	{0x016A, 0x2},	// None
	{0xE0C0, 0x0},	// None
	{0xE0C1, 0x20}, // None
	{0xE0C2, 0x0},	// None
	{0xE0C3, 0x20}, // None
	{0x0168, 0x2C}, // None
	{0xE000, 0x0},	// None
	{0xE0A2, 0x0},	// None
	{0xE07B, 0x0},	// None
	{0xE078, 0x0},	// None
	{0xE079, 0x1},	// None
	{0x2077, 0x0},	// None
	{0x2076, 0x93}, // None
	{0x00CE, 0x1},	// None
	{0x0070, 0x6},	// None
	{0x016D, 0x22}, // None
	{0x0176, 0x42}, // None
	{0xE136, 0x0},	// None
	{0xE0C6, 0x0},	// None
	{0xE0C7, 0x0},	// None
	{0xE0C8, 0x1},	// None
	{0xE0C9, 0x0},	// None
	{0xE0CA, 0x0},	// None
	{0xE0CB, 0x1},	// None
	{0xE0CC, 0x80}, // None
	{0xE0CD, 0x80}, // None
	{0xE0BE, 0x2},	// None
	{0xE0BF, 0x2},	// None
	{0xE0C4, 0x8},	// None
	{0xE0C5, 0x8},	// None
	{0x2075, 0x0},	// None
	{0xE000, 0x0},	// None
	{0x001E, 0x1},	// None
	{0xE000, 0x0},	// None
	{0x207E, 0x0},	// None
	{0xE084, 0x0},	// None
	{0xE085, 0x0},	// None
	{0xE086, 0x1},	// None
	{0xE087, 0x0},	// None
	{0x207F, 0x0},	// None
	{0x2080, 0x0},	// None
	{0x2081, 0x3},	// None
	{0x2082, 0x0},	// None
	{0x2083, 0x2},	// None
	{0x0090, 0x0},	// None
	{0x2097, 0x0},	// None
	{0xE000, 0x0},	// None
	{0x0011, 0x3},	// None
	{0x011D, 0x0},	// None
	{0xE000, 0x0},	// None
	{0x0012, 0x0},	// None
	{0x0013, 0x18}, // None
	{0x015A, 0x0},	// None
	{0x015B, 0x57}, // None
	{0x015C, 0x0},	// None
	{0x015D, 0x57}, // None
	{0x015E, 0x0},	// None
	{0x015F, 0x57}, // None
	{0x0162, 0x0},	// None
	{0x0163, 0x5},	// None
	{0x0164, 0x4},	// None
	{0x0165, 0x79}, // None
	{0x0166, 0x4},	// None
	{0x0167, 0x79}, // None
	{0xE000, 0x0},	// None
	{0x01BB, 0xB4}, // None
	{0x01BC, 0xAC}, // None
	{0x00D0, 0x0},	// None
	{0x01F0, 0x7},	// None
	{0x01F3, 0x1},	// None
	{0x016E, 0xFD}, // None
	{0x0172, 0x0},	// None
	{0x0173, 0x0},	// None
	{0x016F, 0xFF}, // None
	{0x0170, 0xFF}, // None
	{0x0171, 0xFD}, // None
	{0x0174, 0x0},	// None
	{0x0175, 0x0},	// None
	{0x0177, 0xDC}, // None
	{0x018B, 0x4},	// None
	{0x018C, 0xE},	// None
	{0x018D, 0x2},	// None
	{0x018E, 0x56}, // None
	{0x018F, 0xC},	// None
	{0x0190, 0xE},	// None
	{0x00E8, 0x3},	// None
	{0xE000, 0x1},	// None
	{0xE000, 0x1},	// None
	{0xE024, 0xF},	// None
	{0xE000, 0x0},	// None
	{0xE000, 0x0},	// None
	{0x005C, 0x0},	// None
	{0x005D, 0x60}, // None
	{0x01EE, 0x1B}, // None
	{0x01EF, 0x46}, // None
	{0x01A2, 0x0},	// None
	{0x01A3, 0x1},	// None
	{0x031F, 0x0},	// None
	{0x0320, 0xA},	// None
	{0x01A6, 0x0},	// None
	{0x01A7, 0x98}, // None
	{0x01A4, 0x5},	// None
	{0x01A5, 0xA7}, // None
	{0x0321, 0x5},	// None
	{0x0322, 0xB0}, // None
	{0x01A8, 0x6},	// None
	{0x01A9, 0x3E}, // None
	{0x01A0, 0x0},	// None
	{0x01A1, 0xF3}, // None
	{0x01B2, 0x1},	// None
	{0x01B3, 0xD},	// None
	{0x01B0, 0x1},	// None
	{0x01B1, 0x6},	// None
	{0x01AC, 0x1},	// None
	{0x01AD, 0x11}, // None
	{0xE000, 0x0},	// None
	{0x0193, 0x8},	// None
	{0x0194, 0x4},	// None
	{0xE32C, 0x1},	// None
	{0xE32D, 0x0},	// None
	{0xE14E, 0x0},	// None
	{0xE312, 0x7F}, // None
	{0xE329, 0x1},	// None
	{0xE32B, 0x0},	// None
	{0xE331, 0xF},	// None
	{0xE332, 0x3},	// None
	{0xE32A, 0x0},	// None
	{0xE11E, 0x1},	// None
	{0xE000, 0x0},	// None
	{0xE009, 0x1},	// None
	{0x212F, 0x1},	// None
	{0x2130, 0x1},	// None
	{0x2131, 0x1},	// None
	{0x2132, 0x1},	// None
	{0x2133, 0x1},	// None
	{0x2134, 0x1},	// None
	{0x2135, 0x1},	// None
	{0xE0E1, 0x1},	// None
	{0x018A, 0x1},	// None
	{0x00E0, 0x1},	// None
	{0xE32E, 0x1},	// None
	{0xE340, 0x1},	// None
	{0xE004, 0x0},	// None
	{0xE000, 0x1},	// None
	{0xE02C, 0x0},	// None
	{0xE02D, 0xE},	// None
	{0xE02E, 0x1},	// None
	{0xE02F, 0x9D}, // None
	{0xE030, 0x0},	// None
	{0xE025, 0x0},	// None
	{0xE02A, 0x0},	// None
	{0x2029, 0x28}, // None
	{0x0034, 0x0},	// None
	{0x0035, 0xC8}, // None
	{0xE004, 0x1},	// None
	{0xE02C, 0x0},	// None
	{0xE02D, 0xE},	// None
	{0xE02E, 0x1},	// None
	{0xE02F, 0x9D}, // None
	{0xE030, 0x0},	// None
	{0xE025, 0x0},	// None
	{0xE02A, 0x0},	// None
	{0x2029, 0x28}, // None
	{0x0034, 0x0},	// None
	{0x0035, 0xC8}, // None
	{0xE000, 0x1},	// None
	{0xE004, 0x0},	// None
	{0xE000, 0x1},	// None
	{0x001E, 0x0},	// None
	{0x001F, 0x2},	// None
	{0x002B, 0x0},	// None
	{0xE004, 0x1},	// None
	{0x001E, 0x0},	// None
	{0x001F, 0x2},	// None
	{0x002B, 0x0},	// None
	{0xE000, 0x0},	// None
	{0xE000, 0x0},	// None
	{0x001F, 0x0},	// None
	{0x0020, 0x0},	// None
	{0x0023, 0x0},	// None
	{0x0024, 0x1},	// None
	{0x0025, 0x90}, // None
	{0x0026, 0x0},	// None
	{0x0027, 0xE},	// None
	{0x0028, 0x0},	// None
	{0x0029, 0x1},	// None
	{0x002A, 0x90}, // None
	{0x002B, 0x0},	// None
	{0x002C, 0xE},	// None
	{0x002D, 0x0},	// None
	{0x002E, 0x0},	// None
	{0x002F, 0x0},	// None
	{0x0030, 0x0},	// None
	{0x0031, 0x0},	// None
	{0x0032, 0x0},	// None
	{0x0033, 0x0},	// None
	{0x0034, 0x0},	// None
	{0x0035, 0x0},	// None
	{0x0036, 0x0},	// None
	{0x0037, 0x0},	// None
	{0x0038, 0x0},	// None
	{0x0039, 0x0},	// None
	{0x003A, 0x0},	// None
	{0x003B, 0x0},	// None
	{0x003C, 0x0},	// None
	{0x003D, 0x0},	// None
	{0x003E, 0x0},	// None
	{0x003F, 0x0},	// None
	{0x0040, 0x0},	// None
	{0x0041, 0x0},	// None
	{0x0042, 0x0},	// None
	{0x0043, 0x0},	// None
	{0x0044, 0x0},	// None
	{0x0045, 0x0},	// None
	{0x0046, 0x0},	// None
	{0x0047, 0x0},	// None
	{0x0048, 0x0},	// None
	{0x0049, 0x0},	// None
	{0x004A, 0x0},	// None
	{0x004B, 0x0},	// None
	{0x004C, 0x0},	// None
	{0x004D, 0x0},	// None
	{0x004E, 0x0},	// None
	{0x004F, 0x0},	// None
	{0x0050, 0x0},	// None
	{0x0051, 0x0},	// None
	{0x0052, 0x0},	// None
	{0x0053, 0x0},	// None
	{0x0054, 0x0},	// None
	{0x0055, 0x0},	// None
	{0xE004, 0x0},	// None
	{0xE000, 0x1},	// None
	{0x000E, 0x0},	// None
	{0x000F, 0x0},	// None
	{0x0010, 0x3},	// None
	{0x0011, 0xE8}, // None
	{0x0012, 0x0},	// None
	{0x0013, 0x0},	// None
	{0x0014, 0x0},	// None
	{0x0015, 0x0},	// None
	{0xE004, 0x1},	// None
	{0x000E, 0x0},	// None
	{0x000F, 0x0},	// None
	{0x0010, 0x3},	// None
	{0x0011, 0xE8}, // None
	{0x0012, 0x0},	// None
	{0x0013, 0x0},	// None
	{0x0014, 0x0},	// None
	{0x0015, 0x0},	// None
	{0xE004, 0x0},	// None
	{0x0032, 0x5},	// None
	{0x0033, 0xE0}, // None
	{0xE004, 0x1},	// None
	{0x0032, 0x5},	// None
	{0x0033, 0xE0}, // None
	{0xE004, 0x0},	// None
	{0x0007, 0x1},	// None
	{0x0008, 0x0},	// None
	{0x0009, 0x0},	// None
	{0x000A, 0x41}, // None
	{0x000B, 0x1A}, // None
	{0xE004, 0x1},	// None
	{0x0007, 0x1},	// None
	{0x0008, 0x0},	// None
	{0x0009, 0x0},	// None
	{0x000A, 0x17}, // None
	{0x000B, 0x70}, // None
	{0xE000, 0x0},	// None
	{0x0057, 0x0},	// None
	{0x0058, 0x0},	// None
	{0x0059, 0x2},	// None
	{0x005A, 0x2},	// None
	{0x005B, 0x0},	// None
	{0xE000, 0x0},	// None
	{0xE008, 0x0},	// None
	{0x0006, 0x1},	// None
	{0xE003, 0x0},	// None
	{0x0006, 0x0},	// None
	{0xE008, 0x0},	// None
	{0xE004, 0x0},	// None
	{0xE000, 0x1},	// None
	{0x0031, 0x0},	// None
	{0xE004, 0x1},	// None
	{0x0031, 0x0},	// None
	{0xE000, 0x0},	// None
	{0x0138, 0x0},	// None
	{0xE005, 0x0},	// None
	{0x0139, 0x0},	// None
	{0x013A, 0x0},	// None
	{0x013B, 0x64}, // None
	{0x013C, 0x0},	// None
	{0x013D, 0x0},	// None
	{0x013E, 0x64}, // None
	{0x013F, 0x6},	// None
	{0x0140, 0x1},	// None
	{0x0141, 0x10}, // None
	{0x0142, 0x1},	// None
	{0x0143, 0x0},	// None
	{0x0144, 0x0},	// None
	{0x0146, 0x0},	// None
	{0x0147, 0x0},	// None
	{0x0148, 0x0},	// None
	{0xE004, 0x0},	// None
	{0xE000, 0x1},	// None
	{0x0026, 0x0},	// None
	{0xE004, 0x1},	// None
	{0x0026, 0x0},	// None
	{0xE000, 0x0},	// None
	{0x0169, 0x12}, // None
	{0xE004, 0x0},	// None
	{0xE000, 0x1},	// None
	{0x001C, 0x1},	// None
	{0x0019, 0x0},	// None
	{0x001A, 0x7},	// None
	{0x001B, 0x53}, // None
	{0x0016, 0x8},	// None
	{0x0017, 0x0},	// None
	{0x0018, 0x0},	// None
	{0xE004, 0x1},	// None
	{0x001C, 0x0},	// None
	{0x0019, 0x0},	// None
	{0x001A, 0x7},	// None
	{0x001B, 0x53}, // None
	{0x0016, 0x8},	// None
	{0x0017, 0x0},	// None
	{0x0018, 0x0},	// None
	{0xE004, 0x0},	// None
	{0xE000, 0x1},	// None
	{0x001D, 0x1},	// None
	{0xE004, 0x1},	// None
	{0x001D, 0x0},	// None
	{0xE000, 0x0},	// None
	{0x001A, 0x0},	// None
	{0x001B, 0x0},	// None
	{0x001C, 0x0},	// None
	{0xE000, 0x0},	// None
	{0xE000, 0},
	{0xE004, 0},
	{0x33D, 1},
};

static const struct mira016_reg full_400_400_100fps_12b_1lane_reg_post_soft_reset[] = {
	{0xE000, 0},
	{0xE004, 0},
	// Below are manually added after reg seq txt
	{0x0335, 1},  // iref sel
	{0x0324, 43}, // iref val

	{0x1d9, 1},	 // #vddana sel
	{0x0EB, 15}, // #vddana val trim

	{0x1dd, 1},	 // # vsspc sel
	{0x1ed, 14}, // # vsspc val

	{0x1df, 1}, // #cp sel
	{0x0ee, 4}, // #cp trim,
};

static const struct mira016_reg partial_analog_gain_x1_12bit[] = {
	{57344, 0},
	{443, 180},
	{444, 172},
	{208, 0},
	{496, 7},
	{499, 1},
	{366, 253},
	{370, 0},
	{371, 0},
	{367, 255},
	{368, 255},
	{369, 253},
	{372, 0},
	{373, 0},
	{395, 4},
	{396, 14},
	{397, 2},
	{398, 86},
	{399, 12},
	{400, 14},
	{375, 220},
	{494, 27},
	{495, 70},
	{418, 0},
	{419, 1},
	{799, 0},
	{800, 10},
	{422, 0},
	{423, 152},
	{420, 5},
	{421, 167},
	{801, 5},
	{802, 176},
	{424, 6},
	{425, 62},
	{416, 0},
	{417, 243},
	{434, 1},
	{435, 13},
	{432, 1},
	{433, 6},
	{428, 1},
	{429, 17},
	{57344, 1},
	{57344, 1},
	{57380, 15},
	{57344, 0},
	{57344, 0},
	{92, 0},
	{93, 96},
	{57344, 0},
	{403, 8},
	{404, 3},
	{57344, 0},
	{57353, 1},
	{8495, 1},
	{8496, 1},
	{8497, 1},
	{8498, 1},
	{8499, 1},
	{8500, 1},
	{8501, 1},
	{57569, 1},
	{394, 1},
	{224, 1},
	{58158, 1},
	{58176, 1},
	{57344, 0},
	{57817, 1},
	{57344, 0},
	{57579, 16},
	{57344, 0},
	{57821, 1},
	{57344, 0},
	{57581, 14},
	{57344, 0},
	{57823, 1},
	{57344, 0},
	{57582, 4},
	{57344, 0},
	{58165, 1},
	{57344, 0},
	{58148, 42},
	{57344, 0},
	{57850, 1},
	{57344, 1},
	{57348, 0},
	{57344, 1},
	{30, 0},
	{31, 2},
	{43, 0},
	{57348, 1},
	{30, 0},
	{31, 2},
	{43, 0},
	{57344, 0},
	{57344, 0},
	{31, 0},
	{32, 0},
	{35, 0},
	{36, 1},
	{37, 144},
	{38, 0},
	{39, 14},
	{40, 0},
	{41, 1},
	{42, 144},
	{43, 0},
	{44, 14},
	{45, 0},
	{46, 0},
	{47, 0},
	{48, 0},
	{49, 0},
	{50, 0},
	{51, 0},
	{52, 0},
	{53, 0},
	{54, 0},
	{55, 0},
	{56, 0},
	{57, 0},
	{58, 0},
	{59, 0},
	{60, 0},
	{61, 0},
	{62, 0},
	{63, 0},
	{64, 0},
	{65, 0},
	{66, 0},
	{67, 0},
	{68, 0},
	{69, 0},
	{70, 0},
	{71, 0},
	{72, 0},
	{73, 0},
	{74, 0},
	{75, 0},
	{76, 0},
	{77, 0},
	{78, 0},
	{79, 0},
	{80, 0},
	{81, 0},
	{82, 0},
	{83, 0},
	{84, 0},
	{85, 0},
	{57344, 1},
	{57348, 0},
	{57344, 1},
	{57388, 0},
	{57389, 14},
	{57390, 1},
	{57391, 157},
	{57392, 0},
	{57381, 0},
	{57386, 0},
	{8233, 40},
	{52, 0},
	{53, 200},
	{57348, 0},
	{30, 0},
	{31, 2},
	{43, 0},
	// { 57348, 0},
	// { 14, 0},
	// { 15, 0},
	// { 16, 3},
	// { 17, 232},
	// { 18, 0},
	// { 19, 0},
	// { 20, 0},
	// { 21, 0},
	// frame time
	{57348, 0},
	{50, 5},
	{51, 224},
	{57348, 0},
	{7, 1},
	// { 8, 0},
	// { 9, 0},
	// { 10, 19},
	// { 11, 136},
	{57348, 0},
	{49, 0},
	{57348, 0},
	{38, 0},
	{57348, 0},
	// { 28, 0},
	// { 25, 0},
	// { 26, 7},
	// { 27, 83},
	// { 22, 8},
	// { 23, 0},
	// { 24, 0},
	{57348, 0},
	// { 29, 0},
	{57344, 0},
	{57344, 0},
	{58156, 1},
	{57344, 0},
	{58157, 1},
	{57344, 0},
	{57678, 0},
	{57344, 0},
	{58130, 127},
	{57344, 0},
	{58153, 3},
	{57344, 0},
	{58155, 0},
	{57344, 0},
	{58161, 15},
	{57344, 0},
	{58162, 0},
	{57344, 0},
	{58154, 0},
	{57344, 0},
	{57630, 1},
	{0, 0},
	{0, 0},
	{434, 1},
	{435, 21},

};

static const struct mira016_reg partial_analog_gain_x2_12bit[] = {
	{57344, 0},
	{443, 156},
	{444, 148},
	{208, 0},
	{496, 7},
	{499, 0},
	{366, 255},
	{370, 254},
	{371, 0},
	{367, 255},
	{368, 255},
	{369, 255},
	{372, 254},
	{373, 0},
	{395, 8},
	{396, 14},
	{397, 2},
	{398, 86},
	{399, 16},
	{400, 14},
	{375, 220},
	{494, 27},
	{495, 70},
	{418, 0},
	{419, 1},
	{799, 0},
	{800, 10},
	{422, 0},
	{423, 152},
	{420, 7},
	{421, 167},
	{801, 7},
	{802, 176},
	{424, 8},
	{425, 62},
	{416, 1},
	{417, 72},
	{434, 1},
	{435, 98},
	{432, 1},
	{433, 91},
	{428, 1},
	{429, 102},
	{57344, 1},
	{57344, 1},
	{57380, 15},
	{57344, 0},
	{57344, 0},
	{92, 0},
	{93, 96},
	{57344, 0},
	{403, 15},
	{404, 234},
	{57344, 0},
	{57353, 1},
	{8495, 1},
	{8496, 1},
	{8497, 1},
	{8498, 1},
	{8499, 1},
	{8500, 1},
	{8501, 1},
	{57569, 1},
	{394, 1},
	{224, 1},
	{58158, 1},
	{58176, 1},
	{57344, 0},
	{57817, 1},
	{57344, 0},
	{57579, 16},
	{57344, 0},
	{57821, 1},
	{57344, 0},
	{57581, 14},
	{57344, 0},
	{57823, 1},
	{57344, 0},
	{57582, 4},
	{57344, 0},
	{58165, 1},
	{57344, 0},
	{58148, 42},
	{57344, 0},
	{57850, 1},
	{57344, 1},
	{57348, 0},
	{57344, 1},
	{30, 0},
	{31, 2},
	{43, 0},
	{57348, 1},
	{30, 0},
	{31, 2},
	{43, 0},
	{57344, 0},
	{57344, 0},
	{31, 0},
	{32, 0},
	{35, 0},
	{36, 1},
	{37, 144},
	{38, 0},
	{39, 14},
	{40, 0},
	{41, 1},
	{42, 144},
	{43, 0},
	{44, 14},
	{45, 0},
	{46, 0},
	{47, 0},
	{48, 0},
	{49, 0},
	{50, 0},
	{51, 0},
	{52, 0},
	{53, 0},
	{54, 0},
	{55, 0},
	{56, 0},
	{57, 0},
	{58, 0},
	{59, 0},
	{60, 0},
	{61, 0},
	{62, 0},
	{63, 0},
	{64, 0},
	{65, 0},
	{66, 0},
	{67, 0},
	{68, 0},
	{69, 0},
	{70, 0},
	{71, 0},
	{72, 0},
	{73, 0},
	{74, 0},
	{75, 0},
	{76, 0},
	{77, 0},
	{78, 0},
	{79, 0},
	{80, 0},
	{81, 0},
	{82, 0},
	{83, 0},
	{84, 0},
	{85, 0},
	{57344, 1},
	{57348, 0},
	{57344, 1},
	{57388, 0},
	{57389, 14},
	{57390, 1},
	{57391, 157},
	{57392, 0},
	{57381, 0},
	{57386, 0},
	{8233, 40},
	{52, 0},
	{53, 200},
	{57348, 0},
	{30, 0},
	{31, 2},
	{43, 0},
	// { 57348, 0},
	// { 14, 0},
	// { 15, 0},
	// { 16, 3},
	// { 17, 232},
	// { 18, 0},
	// { 19, 0},
	// { 20, 0},
	// { 21, 0},
	{57348, 0},
	{50, 8},
	{51, 8},
	{57348, 0},
	{7, 1},
	// { 8, 0},
	// { 9, 0},
	// { 10, 19},
	// { 11, 136},
	{57348, 0},
	{49, 0},
	{57348, 0},
	{38, 0},
	{57348, 0},
	// { 28, 0},
	// { 25, 0},
	// { 26, 7},
	// { 27, 83},
	// { 22, 8},
	// { 23, 0},
	// { 24, 0},
	{57348, 0},
	// { 29, 0},
	{57344, 0},
	{57344, 0},
	{58156, 1},
	{57344, 0},
	{58157, 1},
	{57344, 0},
	{57678, 0},
	{57344, 0},
	{58130, 127},
	{57344, 0},
	{58153, 3},
	{57344, 0},
	{58155, 0},
	{57344, 0},
	{58161, 15},
	{57344, 0},
	{58162, 0},
	{57344, 0},
	{58154, 0},
	{57344, 0},
	{57630, 1},
	{0, 0},
	{0, 0},
	{434, 1},
	{435, 106},
};

// converted_Draco_i2c_configuration_sequence_hex_8bit_1x_360fps_Version3
static const struct mira016_reg full_400_400_100fps_8b_1lane_reg_pre_soft_reset[] = {
	// Sensor Operating Mode
	//"Mira016_register_sequence_8b_1-16x_60fps_1000M.txt"
	{0xE000, 0x0},	// None
	{0x01E4, 0x0},	// None
	{0x01E5, 0x13}, // None
	{0x01E2, 0x17}, // None
	{0x01E3, 0xA8}, // None
	{0x01E6, 0x0},	// None
	{0x01E7, 0xCA}, // None
	{0x016C, 0x1},	// None
	{0x016B, 0x1},	// None
	{0x0208, 0x1},	// None
	{0x0209, 0xF0}, // None
	{0x020A, 0x3},	// None
	{0x020B, 0x4D}, // None
	{0x020C, 0x2},	// None
	{0x020D, 0x10}, // None
	{0x020E, 0x3},	// None
	{0x020F, 0x1},	// None
	{0x0210, 0x0},	// None
	{0x0211, 0x13}, // None
	{0x0212, 0x0},	// None
	{0x0213, 0x3},	// None
	{0x0214, 0x3},	// None
	{0x0215, 0xEF}, // None
	{0x0216, 0x3},	// None
	{0x0217, 0xF3}, // None
	{0x0218, 0x3},	// None
	{0x0219, 0xF4}, // None
	{0x021A, 0x0},	// None
	{0x021B, 0x1},	// None
	{0x021C, 0x3},	// None
	{0x021D, 0xF8}, // None
	{0x021E, 0x0},	// None
	{0x021F, 0x2},	// None
	{0x0220, 0x1},	// None
	{0x0221, 0xF2}, // None
	{0x0222, 0x3},	// None
	{0x0223, 0x1B}, // None
	{0x0224, 0x0},	// None
	{0x0225, 0x21}, // None
	{0x0226, 0x3},	// None
	{0x0227, 0xF0}, // None
	{0x0228, 0x3},	// None
	{0x0229, 0xF1}, // None
	{0x022A, 0x3},	// None
	{0x022B, 0xF2}, // None
	{0x022C, 0x3},	// None
	{0x022D, 0xF5}, // None
	{0x022E, 0x3},	// None
	{0x022F, 0xF6}, // None
	{0x0230, 0x0},	// None
	{0x0231, 0xC1}, // None
	{0x0232, 0x0},	// None
	{0x0233, 0x2},	// None
	{0x0234, 0x1},	// None
	{0x0235, 0xF2}, // None
	{0x0236, 0x3},	// None
	{0x0237, 0x6B}, // None
	{0x0238, 0x3},	// None
	{0x0239, 0xFF}, // None
	{0x023A, 0x3},	// None
	{0x023B, 0x31}, // None
	{0x023C, 0x1},	// None
	{0x023D, 0xF0}, // None
	{0x023E, 0x3},	// None
	{0x023F, 0x87}, // None
	{0x0240, 0x0},	// None
	{0x0241, 0xA},	// None
	{0x0242, 0x0},	// None
	{0x0243, 0xB},	// None
	{0x0244, 0x1},	// None
	{0x0245, 0xF9}, // None
	{0x0246, 0x3},	// None
	{0x0247, 0xD},	// None
	{0x0248, 0x0},	// None
	{0x0249, 0x7},	// None
	{0x024A, 0x3},	// None
	{0x024B, 0xEF}, // None
	{0x024C, 0x3},	// None
	{0x024D, 0xF3}, // None
	{0x024E, 0x3},	// None
	{0x024F, 0xF4}, // None
	{0x0250, 0x3},	// None
	{0x0251, 0x0},	// None
	{0x0252, 0x0},	// None
	{0x0253, 0x7},	// None
	{0x0254, 0x0},	// None
	{0x0255, 0xC},	// None
	{0x0256, 0x1},	// None
	{0x0257, 0xF1}, // None
	{0x0258, 0x3},	// None
	{0x0259, 0x43}, // None
	{0x025A, 0x1},	// None
	{0x025B, 0xF8}, // None
	{0x025C, 0x3},	// None
	{0x025D, 0x10}, // None
	{0x025E, 0x0},	// None
	{0x025F, 0x7},	// None
	{0x0260, 0x3},	// None
	{0x0261, 0xF0}, // None
	{0x0262, 0x3},	// None
	{0x0263, 0xF1}, // None
	{0x0264, 0x3},	// None
	{0x0265, 0xF2}, // None
	{0x0266, 0x3},	// None
	{0x0267, 0xF5}, // None
	{0x0268, 0x3},	// None
	{0x0269, 0xF6}, // None
	{0x026A, 0x3},	// None
	{0x026B, 0x0},	// None
	{0x026C, 0x2},	// None
	{0x026D, 0x87}, // None
	{0x026E, 0x0},	// None
	{0x026F, 0x1},	// None
	{0x0270, 0x3},	// None
	{0x0271, 0xFF}, // None
	{0x0272, 0x3},	// None
	{0x0273, 0x0},	// None
	{0x0274, 0x3},	// None
	{0x0275, 0xFF}, // None
	{0x0276, 0x2},	// None
	{0x0277, 0x87}, // None
	{0x0278, 0x3},	// None
	{0x0279, 0x2},	// None
	{0x027A, 0x3},	// None
	{0x027B, 0xF},	// None
	{0x027C, 0x3},	// None
	{0x027D, 0xF7}, // None
	{0x027E, 0x0},	// None
	{0x027F, 0x16}, // None
	{0x0280, 0x0},	// None
	{0x0281, 0x33}, // None
	{0x0282, 0x0},	// None
	{0x0283, 0x4},	// None
	{0x0284, 0x0},	// None
	{0x0285, 0x11}, // None
	{0x0286, 0x3},	// None
	{0x0287, 0x9},	// None
	{0x0288, 0x0},	// None
	{0x0289, 0x2},	// None
	{0x028A, 0x0},	// None
	{0x028B, 0x20}, // None
	{0x028C, 0x0},	// None
	{0x028D, 0xB5}, // None
	{0x028E, 0x0},	// None
	{0x028F, 0xE5}, // None
	{0x0290, 0x0},	// None
	{0x0291, 0x12}, // None
	{0x0292, 0x0},	// None
	{0x0293, 0xB5}, // None
	{0x0294, 0x0},	// None
	{0x0295, 0xE5}, // None
	{0x0296, 0x0},	// None
	{0x0297, 0x10}, // None
	{0x0298, 0x0},	// None
	{0x0299, 0x2},	// None
	{0x029A, 0x0},	// None
	{0x029B, 0x20}, // None
	{0x029C, 0x0},	// None
	{0x029D, 0xB5}, // None
	{0x029E, 0x0},	// None
	{0x029F, 0xE5}, // None
	{0x02A0, 0x0},	// None
	{0x02A1, 0x12}, // None
	{0x02A2, 0x0},	// None
	{0x02A3, 0xB5}, // None
	{0x02A4, 0x0},	// None
	{0x02A5, 0xE5}, // None
	{0x02A6, 0x0},	// None
	{0x02A7, 0x0},	// None
	{0x02A8, 0x0},	// None
	{0x02A9, 0x12}, // None
	{0x02AA, 0x0},	// None
	{0x02AB, 0x12}, // None
	{0x02AC, 0x0},	// None
	{0x02AD, 0x20}, // None
	{0x02AE, 0x0},	// None
	{0x02AF, 0xB5}, // None
	{0x02B0, 0x0},	// None
	{0x02B1, 0xE5}, // None
	{0x02B2, 0x0},	// None
	{0x02B3, 0x0},	// None
	{0x02B4, 0x0},	// None
	{0x02B5, 0x12}, // None
	{0x02B6, 0x0},	// None
	{0x02B7, 0x12}, // None
	{0x02B8, 0x0},	// None
	{0x02B9, 0x20}, // None
	{0x02BA, 0x0},	// None
	{0x02BB, 0x47}, // None
	{0x02BC, 0x0},	// None
	{0x02BD, 0x27}, // None
	{0x02BE, 0x0},	// None
	{0x02BF, 0xB5}, // None
	{0x02C0, 0x0},	// None
	{0x02C1, 0xE5}, // None
	{0x02C2, 0x0},	// None
	{0x02C3, 0x0},	// None
	{0x02C4, 0x0},	// None
	{0x02C5, 0x4},	// None
	{0x02C6, 0x0},	// None
	{0x02C7, 0x43}, // None
	{0x02C8, 0x0},	// None
	{0x02C9, 0x1},	// None
	{0x02CA, 0x3},	// None
	{0x02CB, 0x2},	// None
	{0x02CC, 0x0},	// None
	{0x02CD, 0x8},	// None
	{0x02CE, 0x3},	// None
	{0x02CF, 0xFF}, // None
	{0x02D0, 0x2},	// None
	{0x02D1, 0x87}, // None
	{0x02D2, 0x3},	// None
	{0x02D3, 0xC7}, // None
	{0x02D4, 0x3},	// None
	{0x02D5, 0xF7}, // None
	{0x02D6, 0x0},	// None
	{0x02D7, 0x77}, // None
	{0x02D8, 0x0},	// None
	{0x02D9, 0x17}, // None
	{0x02DA, 0x0},	// None
	{0x02DB, 0x8},	// None
	{0x02DC, 0x3},	// None
	{0x02DD, 0xFF}, // None
	{0x02DE, 0x0},	// None
	{0x02DF, 0x38}, // None
	{0x02E0, 0x0},	// None
	{0x02E1, 0x17}, // None
	{0x02E2, 0x0},	// None
	{0x02E3, 0x8},	// None
	{0x02E4, 0x3},	// None
	{0x02E5, 0xFF}, // None
	{0x02E6, 0x3},	// None
	{0x02E7, 0xFF}, // None
	{0x02E8, 0x3},	// None
	{0x02E9, 0xFF}, // None
	{0x02EA, 0x3},	// None
	{0x02EB, 0xFF}, // None
	{0x02EC, 0x3},	// None
	{0x02ED, 0xFF}, // None
	{0x02EE, 0x3},	// None
	{0x02EF, 0xFF}, // None
	{0x02F0, 0x3},	// None
	{0x02F1, 0xFF}, // None
	{0x02F2, 0x3},	// None
	{0x02F3, 0xFF}, // None
	{0x02F4, 0x3},	// None
	{0x02F5, 0xFF}, // None
	{0x02F6, 0x3},	// None
	{0x02F7, 0xFF}, // None
	{0x02F8, 0x3},	// None
	{0x02F9, 0xFF}, // None
	{0x02FA, 0x3},	// None
	{0x02FB, 0xFF}, // None
	{0x02FC, 0x3},	// None
	{0x02FD, 0xFF}, // None
	{0x02FE, 0x3},	// None
	{0x02FF, 0xFF}, // None
	{0x0300, 0x3},	// None
	{0x0301, 0xFF}, // None
	{0x0302, 0x3},	// None
	{0x0303, 0xFF}, // None
	{0x01E9, 0x0},	// None
	{0x01E8, 0x19}, // None
	{0x01EA, 0x35}, // None
	{0x01EB, 0x37}, // None
	{0x01EC, 0x64}, // None
	{0x01ED, 0x6B}, // None
	{0x01F8, 0xF},	// None
	{0x01D8, 0x1},	// None
	{0x01DC, 0x1},	// None
	{0x01DE, 0x1},	// None
	{0x0189, 0x1},	// None
	{0x01B7, 0x1},	// None
	{0x01C1, 0x7},	// None
	{0x01C2, 0xF6}, // None
	{0x01C3, 0xFF}, // None
	{0x01C9, 0x7},	// None
	{0x0325, 0x0},	// None
	{0xE159, 0x0},	// None
	{0x033A, 0x0},	// None
	{0x01B8, 0x1},	// None
	{0x01BA, 0x33}, // None
	{0x01BE, 0x74}, // None
	{0x01BF, 0x36}, // None
	{0x01C0, 0x53}, // None
	{0x00EF, 0x0},	// None
	{0x0326, 0x0},	// None
	{0x00F0, 0x0},	// None
	{0x00F1, 0x0},	// None
	{0x0327, 0x0},	// None
	{0x00F2, 0x0},	// None
	{0x0071, 0x1},	// None
	{0x01B4, 0x1},	// None
	{0x01B5, 0x1},	// None
	{0x01F1, 0x1},	// None
	{0x01F4, 0x1},	// None
	{0x01F5, 0x1},	// None
	{0x0314, 0x1},	// None
	{0x0315, 0x1},	// None
	{0x0316, 0x1},	// None
	{0x0207, 0x0},	// None
	{0x4207, 0x2},	// None
	{0x2207, 0x2},	// None
	{0xE088, 0x1},	// None
	{0xE08D, 0x0},	// None
	{0xE08E, 0x30}, // None
	{0xE08F, 0xD4}, // None
	{0xE089, 0x56}, // None
	{0xE08A, 0x10}, // None
	{0xE08B, 0x1F}, // None
	{0xE08C, 0xF},	// None
	{0xE0A6, 0x0},	// None
	{0xE0A9, 0x10}, // None
	{0xE0AA, 0x0},	// None
	{0xE0AD, 0xA},	// None
	{0xE0A8, 0x30}, // None
	{0xE0A7, 0xF},	// None
	{0xE0AC, 0x10}, // None
	{0xE0AB, 0xF},	// None
	{0x209D, 0x0},	// None
	{0x0328, 0x0},	// None
	{0x0063, 0x1},	// None
	{0x01F7, 0xF},	// None
	{0xE0E3, 0x1},	// None
	{0xE0E7, 0x3},	// None
	{0xE33B, 0x0},	// None
	{0xE336, 0x0},	// None
	{0xE337, 0x0},	// None
	{0xE338, 0x0},	// None
	{0xE339, 0x0},	// None
	{0x00E9, 0x1},	// None
	{0x00EA, 0xFE}, // None
	{0x0309, 0x3},	// None
	{0x030A, 0x2},	// None
	{0x030B, 0x2},	// None
	{0x030C, 0x5},	// None
	{0x030E, 0x15}, // None
	{0x030D, 0x14}, // None
	{0x030F, 0x1},	// None
	{0x0310, 0xD},	// None
	{0x01D0, 0x1F}, // None
	{0x01D1, 0x1F}, // None
	{0x01CD, 0x11}, // None
	{0x0016, 0x0},	// None
	{0x0017, 0x5},	// None
	{0x01F2, 0x0},	// None
	{0x016A, 0x0},	// None
	{0xE0C0, 0x0},	// None
	{0xE0C1, 0x8},	// None
	{0xE0C2, 0x0},	// None
	{0xE0C3, 0x8},	// None
	{0x0168, 0x2A}, // None
	{0xE000, 0x0},	// None
	{0xE0A2, 0x0},	// None
	{0xE07B, 0x0},	// None
	{0xE078, 0x0},	// None
	{0xE079, 0x1},	// None
	{0x2077, 0x0},	// None
	{0x2076, 0x93}, // None
	{0x00CE, 0x1},	// None
	{0x0070, 0x6},	// None
	{0x016D, 0x22}, // None
	{0x0176, 0x42}, // None
	{0xE136, 0x0},	// None
	{0xE0C6, 0x0},	// None
	{0xE0C7, 0x0},	// None
	{0xE0C8, 0x1},	// None
	{0xE0C9, 0x0},	// None
	{0xE0CA, 0x0},	// None
	{0xE0CB, 0x1},	// None
	{0xE0CC, 0x80}, // None
	{0xE0CD, 0x80}, // None
	{0xE0BE, 0x2},	// None
	{0xE0BF, 0x2},	// None
	{0xE0C4, 0x8},	// None
	{0xE0C5, 0x8},	// None
	{0x2075, 0x0},	// None
	{0xE000, 0x0},	// None
	{0x001E, 0x1},	// None
	{0xE000, 0x0},	// None
	{0x207E, 0x0},	// None
	{0xE084, 0x0},	// None
	{0xE085, 0x0},	// None
	{0xE086, 0x1},	// None
	{0xE087, 0x0},	// None
	{0x207F, 0x0},	// None
	{0x2080, 0x0},	// None
	{0x2081, 0x3},	// None
	{0x2082, 0x0},	// None
	{0x2083, 0x2},	// None
	{0x0090, 0x0},	// None
	{0x2097, 0x0},	// None
	{0xE000, 0x0},	// None
	{0x0011, 0x3},	// None
	{0x011D, 0x0},	// None
	{0xE000, 0x0},	// None
	{0x0012, 0x0},	// None
	{0x0013, 0x18}, // None
	{0x015A, 0x0},	// None
	{0x015B, 0x57}, // None
	{0x015C, 0x0},	// None
	{0x015D, 0x57}, // None
	{0x015E, 0x0},	// None
	{0x015F, 0x57}, // None
	{0x0162, 0x0},	// None
	{0x0163, 0x5},	// None
	{0x0164, 0x4},	// None
	{0x0165, 0x79}, // None
	{0x0166, 0x4},	// None
	{0x0167, 0x79}, // None
	{0xE000, 0x0},	// None
	{0x01BB, 0xB4}, // None
	{0x01BC, 0xAC}, // None
	{0x00D0, 0x0},	// None
	{0x016E, 0xFD}, // None
	{0x0172, 0x0},	// None
	{0x0173, 0x0},	// None
	{0x016F, 0x7E}, // None
	{0x0170, 0x0},	// None
	{0x0171, 0xFD}, // None
	{0x0174, 0x0},	// None
	{0x0175, 0x0},	// None
	{0x0177, 0x78}, // None
	{0x018B, 0x4},	// None
	{0x018C, 0xE},	// None
	{0x018D, 0x2},	// None
	{0x018E, 0x56}, // None
	{0x018F, 0x6},	// None
	{0x0190, 0xE},	// None
	{0x00E8, 0x8},	// None
	{0x01EE, 0x1B}, // None
	{0x01EF, 0x46}, // None
	{0x01A2, 0x0},	// None
	{0x01A3, 0x1},	// None
	{0x031F, 0x0},	// None
	{0x0320, 0xA},	// None
	{0x01A6, 0x0},	// None
	{0x01A7, 0x98}, // None
	{0x01A4, 0x4},	// None
	{0x01A5, 0x27}, // None
	{0x0321, 0x4},	// None
	{0x0322, 0x30}, // None
	{0x01A8, 0x4},	// None
	{0x01A9, 0xBE}, // None
	{0x01A0, 0x1},	// None
	{0x01A1, 0x3A}, // None
	{0x01B2, 0x1},	// None
	{0x01B3, 0x54}, // None
	{0x01B0, 0x1},	// None
	{0x01B1, 0x4D}, // None
	{0x01AC, 0x1},	// None
	{0x01AD, 0x58}, // None
	{0x01F0, 0x24}, // None
	{0x01F3, 0x2},	// None
	{0xE000, 0x1},	// None
	{0xE000, 0x1},	// None
	{0xE024, 0x3},	// None
	{0xE000, 0x0},	// None
	{0xE000, 0x0},	// None
	{0xE000, 0x0},	// None
	{0x005C, 0x0},	// None
	{0x005D, 0x6}, // None
	{0xE000, 0x0},	// None
	{0x0193, 0x8},	// None
	{0x0194, 0x4},	// None
	{0xE32C, 0x1},	// None
	{0xE32D, 0x0},	// None
	{0xE14E, 0x0},	// None
	{0xE312, 0x7F}, // None
	{0xE329, 0x1},	// None
	{0xE32B, 0x0},	// None
	{0xE331, 0xF},	// None
	{0xE332, 0x3},	// None
	{0xE32A, 0x0},	// None
	{0xE11E, 0x1},	// None
	{0xE000, 0x0},	// None
	{0xE009, 0x1},	// None
	{0x212F, 0x1},	// None
	{0x2130, 0x1},	// None
	{0x2131, 0x1},	// None
	{0x2132, 0x1},	// None
	{0x2133, 0x1},	// None
	{0x2134, 0x1},	// None
	{0x2135, 0x1},	// None
	{0xE0E1, 0x1},	// None
	{0x018A, 0x1},	// None
	{0x00E0, 0x1},	// None
	{0xE32E, 0x1},	// None
	{0xE340, 0x1},	// None
	{0xE004, 0x0},	// None
	{0xE000, 0x1},	// None
	{0xE02C, 0x0},	// None
	{0xE02D, 0xE},	// None
	{0xE02E, 0x1},	// None
	{0xE02F, 0x9D}, // None
	{0xE030, 0x0},	// None
	{0xE025, 0x0},	// None
	{0xE02A, 0x0},	// None
	{0x2029, 0x28}, // None
	{0x0034, 0x0},	// None
	{0x0035, 0xC8}, // None
	{0xE004, 0x1},	// None
	{0xE02C, 0x0},	// None
	{0xE02D, 0xE},	// None
	{0xE02E, 0x1},	// None
	{0xE02F, 0x9D}, // None
	{0xE030, 0x0},	// None
	{0xE025, 0x0},	// None
	{0xE02A, 0x0},	// None
	{0x2029, 0x28}, // None
	{0x0034, 0x0},	// None
	{0x0035, 0xC8}, // None
	{0xE000, 0x1},	// None
	{0xE004, 0x0},	// None
	{0xE000, 0x1},	// None
	{0x001E, 0x0},	// None
	{0x001F, 0x2},	// None
	{0x002B, 0x0},	// None
	{0xE004, 0x1},	// None
	{0x001E, 0x0},	// None
	{0x001F, 0x2},	// None
	{0x002B, 0x0},	// None
	{0xE000, 0x0},	// None
	{0xE000, 0x0},	// None
	{0x001F, 0x0},	// None
	{0x0020, 0x0},	// None
	{0x0023, 0x0},	// None
	{0x0024, 0x1},	// None
	{0x0025, 0x90}, // None
	{0x0026, 0x0},	// None
	{0x0027, 0xE},	// None
	{0x0028, 0x0},	// None
	{0x0029, 0x1},	// None
	{0x002A, 0x90}, // None
	{0x002B, 0x0},	// None
	{0x002C, 0xE},	// None
	{0x002D, 0x0},	// None
	{0x002E, 0x0},	// None
	{0x002F, 0x0},	// None
	{0x0030, 0x0},	// None
	{0x0031, 0x0},	// None
	{0x0032, 0x0},	// None
	{0x0033, 0x0},	// None
	{0x0034, 0x0},	// None
	{0x0035, 0x0},	// None
	{0x0036, 0x0},	// None
	{0x0037, 0x0},	// None
	{0x0038, 0x0},	// None
	{0x0039, 0x0},	// None
	{0x003A, 0x0},	// None
	{0x003B, 0x0},	// None
	{0x003C, 0x0},	// None
	{0x003D, 0x0},	// None
	{0x003E, 0x0},	// None
	{0x003F, 0x0},	// None
	{0x0040, 0x0},	// None
	{0x0041, 0x0},	// None
	{0x0042, 0x0},	// None
	{0x0043, 0x0},	// None
	{0x0044, 0x0},	// None
	{0x0045, 0x0},	// None
	{0x0046, 0x0},	// None
	{0x0047, 0x0},	// None
	{0x0048, 0x0},	// None
	{0x0049, 0x0},	// None
	{0x004A, 0x0},	// None
	{0x004B, 0x0},	// None
	{0x004C, 0x0},	// None
	{0x004D, 0x0},	// None
	{0x004E, 0x0},	// None
	{0x004F, 0x0},	// None
	{0x0050, 0x0},	// None
	{0x0051, 0x0},	// None
	{0x0052, 0x0},	// None
	{0x0053, 0x0},	// None
	{0x0054, 0x0},	// None
	{0x0055, 0x0},	// None
	{0xE004, 0x0},	// None
	{0xE000, 0x1},	// None
	{0x000E, 0x0},	// None
	{0x000F, 0x0},	// None
	{0x0010, 0x3},	// None
	{0x0011, 0xE8}, // None
	{0x0012, 0x0},	// None
	{0x0013, 0x0},	// None
	{0x0014, 0x0},	// None
	{0x0015, 0x0},	// None
	{0xE004, 0x1},	// None
	{0x000E, 0x0},	// None
	{0x000F, 0x0},	// None
	{0x0010, 0x3},	// None
	{0x0011, 0xE8}, // None
	{0x0012, 0x0},	// None
	{0x0013, 0x0},	// None
	{0x0014, 0x0},	// None
	{0x0015, 0x0},	// None
	{0xE004, 0x0},	// None
	{0x0032, 0x4},	// None
	{0x0033, 0xEE}, // None
	{0xE004, 0x1},	// None
	{0x0032, 0x4},	// None
	{0x0033, 0xEE}, // None
	{0xE004, 0x0},	// None
	{0x0007, 0x1},	// None
	{0x0008, 0x0},	// None
	{0x0009, 0x0},	// None
	{0x000A, 0x41}, // None
	{0x000B, 0x1A}, // None
	{0xE004, 0x1},	// None
	{0x0007, 0x1},	// None
	{0x0008, 0x0},	// None
	{0x0009, 0x0},	// None
	{0x000A, 0x17}, // None
	{0x000B, 0x70}, // None
	{0xE000, 0x0},	// None
	{0x0057, 0x0},	// None
	{0x0058, 0x0},	// None
	{0x0059, 0x2},	// None
	{0x005A, 0x2},	// None
	{0x005B, 0x0},	// None
	{0xE000, 0x0},	// None
	{0xE008, 0x0},	// None
	{0x0006, 0x1},	// None
	{0xE003, 0x0},	// None
	{0x0006, 0x0},	// None
	{0xE008, 0x0},	// None
	{0xE004, 0x0},	// None
	{0xE000, 0x1},	// None
	{0x0031, 0x0},	// None
	{0xE004, 0x1},	// None
	{0x0031, 0x0},	// None
	{0xE000, 0x0},	// None
	{0x0138, 0x0},	// None
	{0xE005, 0x0},	// None
	{0x0139, 0x0},	// None
	{0x013A, 0x0},	// None
	{0x013B, 0x64}, // None
	{0x013C, 0x0},	// None
	{0x013D, 0x0},	// None
	{0x013E, 0x64}, // None
	{0x013F, 0x6},	// None
	{0x0140, 0x1},	// None
	{0x0141, 0x10}, // None
	{0x0142, 0x1},	// None
	{0x0143, 0x0},	// None
	{0x0144, 0x0},	// None
	{0x0146, 0x0},	// None
	{0x0147, 0x0},	// None
	{0x0148, 0x0},	// None
	{0xE004, 0x0},	// None
	{0xE000, 0x1},	// None
	{0x0026, 0x0},	// None
	{0xE004, 0x1},	// None
	{0x0026, 0x0},	// None
	{0xE000, 0x0},	// None
	{0x0169, 0x12}, // None
	{0xE004, 0x0},	// None
	{0xE000, 0x1},	// None
	{0x001C, 0x1},	// None
	{0x0019, 0x0},	// None
	{0x001A, 0x7},	// None
	{0x001B, 0x53}, // None
	{0x0016, 0x8},	// None
	{0x0017, 0x0},	// None
	{0x0018, 0x0},	// None
	{0xE004, 0x1},	// None
	{0x001C, 0x0},	// None
	{0x0019, 0x0},	// None
	{0x001A, 0x7},	// None
	{0x001B, 0x53}, // None
	{0x0016, 0x8},	// None
	{0x0017, 0x0},	// None
	{0x0018, 0x0},	// None
	{0xE004, 0x0},	// None
	{0xE000, 0x1},	// None
	{0x001D, 0x1},	// None
	{0xE004, 0x1},	// None
	{0x001D, 0x0},	// None
	{0xE000, 0x0},	// None
	{0x001A, 0x0},	// None
	{0x001B, 0x0},	// None
	{0x001C, 0x0},	// None
	{0xE000, 0x0},	// None

};

static const struct mira016_reg full_400_400_100fps_8b_1lane_reg_post_soft_reset[] = {
	{0xE000, 0},
	{0xE004, 0},
	{0x0335, 1},  // iref sel
	{0x0324, 43}, // iref val
	{0x1d9, 1},	 // #vddana sel
	{0x0EB, 15}, // #vddana val trim
	{0x1dd, 1},	 // # vsspc sel
	{0x1ed, 14}, // # vsspc val
	{0x1df, 1}, // #cp sel
	{0x0ee, 4}, // #cp trim,
};

static const struct mira016_fine_gain_lut_new fine_gain_lut_10bit_hs_4x[] = {
	// gain_256	gdig_preamp	rg_adcgain	rg_mult
	{256, 15, 36, 2},
	{260, 15, 35, 2},
	{269, 15, 33, 2},
	{274, 15, 32, 2},
	{286, 15, 30, 2},
	{291, 15, 29, 2},
	{304, 15, 27, 2},
	{310, 15, 26, 2},
	{324, 15, 24, 2},
	{331, 15, 23, 2},
	{338, 15, 22, 2},
	{348, 15, 62, 1},
	{361, 15, 59, 1},
	{370, 15, 57, 1},
	{380, 15, 55, 1},
	{389, 15, 53, 1},
	{401, 15, 51, 1},
	{418, 15, 48, 1},
	{430, 15, 46, 1},
	{436, 15, 45, 1},
	{451, 15, 43, 1},
	{465, 15, 41, 1},
	{479, 15, 39, 1},
	{495, 15, 37, 1},
	{503, 15, 36, 1},
	{522, 15, 34, 1},
	{542, 15, 32, 1},
	{555, 15, 31, 1},
	{575, 15, 29, 1},
	{587, 15, 28, 1},
	{613, 15, 26, 1},
	{626, 15, 25, 1},
	{639, 15, 24, 1},
	{669, 15, 22, 1},
	{685, 15, 63, 0},
	{701, 15, 61, 0},
	{728, 15, 58, 0},
	{746, 15, 56, 0},
	{766, 15, 54, 0},
	{798, 15, 51, 0},
	{820, 15, 49, 0},
	{845, 15, 47, 0},
	{871, 15, 45, 0},
	{911, 15, 42, 0},
	{926, 15, 41, 0},
	{957, 15, 39, 0},
	{989, 15, 37, 0},
	{1024,15, 35, 0},

};
static const struct mira016_fine_gain_lut_new fine_gain_lut_8bit_16x[] = {
	// gain_256,gdig_preamp,rg_bias,rg_mult
	{256, 3, 36, 2},
	{262, 3, 35, 2},
	{272, 3, 33, 2},
	{276, 3, 32, 2},
	{288, 3, 30, 2},
	{292, 3, 29, 2},
	{298, 3, 28, 2},
	{311, 3, 26, 2},
	{305, 3, 27, 2},
	{318, 3, 25, 2},
	{330, 3, 23, 2},
	{342, 3, 22, 2},
	{350, 3, 22, 2},
	{341, 3, 22, 2},
	{385, 3, 55, 1},
	{391, 3, 54, 1},
	{401, 3, 52, 1},
	{408, 3, 51, 1},
	{419, 3, 49, 1},
	{431, 3, 47, 1},
	{451, 3, 44, 1},
	{459, 3, 43, 1},
	{466, 3, 42, 1},
	{478, 3, 40, 1},
	{491, 3, 39, 1},
	{505, 3, 37, 1},
	{526, 3, 35, 1},
	{555, 3, 32, 1},
	{568, 3, 31, 1},
	{592, 3, 29, 1},
	{602, 3, 28, 1},
	{617, 3, 27, 1},
	{628, 3, 26, 1},
	{657, 3, 24, 1},
	{672, 3, 23, 1},
	{687, 3, 22, 1},
	{726, 3, 61, 0},
	{749, 3, 58, 0},
	{761, 3, 57, 0},
	{780, 3, 55, 0},
	{801, 3, 53, 0},
	{848, 3, 49, 0},
	{873, 3, 47, 0},
	{901, 3, 45, 0},
	{944, 3, 42, 0},
	{975, 3, 40, 0},
	{979, 7, 39, 1},
	{1030, 7, 36, 1},
	{1051, 7, 35, 1},
	{1089, 7, 33, 1},
	{1110, 7, 32, 1},
	{1159, 7, 30, 1},
	{1205, 7, 28, 1},
	{1235, 7, 27, 1},
	{1290, 7, 25, 1},
	{1314, 7, 24, 1},
	{1379, 7, 22, 1},
	{1462, 7, 60, 0},
	{1502, 7, 58, 0},
	{1559, 7, 55, 0},
	{1623, 7, 52, 0},
	{1648, 7, 51, 0},
	{1718, 7, 48, 0},
	{1745, 7, 47, 0},
	{1794, 7, 45, 0},
	{1854, 7, 43, 0},
	{1920, 7, 41, 0},
	{1947, 7, 40, 0},
	{1990, 15, 38, 1},
	{2061, 15, 36, 1},
	{2102, 15, 35, 1},
	{2141, 15, 34, 1},
	{2220, 15, 32, 1},
	{2271, 15, 31, 1},
	{2312, 15, 30, 1},
	{2362, 15, 29, 1},
	{2464, 15, 27, 1},
	{2520, 15, 26, 1},
	{2575, 15, 25, 1},
	{2689, 15, 23, 1},
	{2815, 15, 21, 1},
	{2889, 15, 61, 0},
	{2964, 15, 59, 0},
	{3036, 15, 57, 0},
	{3080, 15, 56, 0},
	{3194, 15, 53, 0},
	{3297, 15, 51, 0},
	{3388, 15, 49, 0},
	{3491, 15, 47, 0},
	{3593, 15, 45, 0},
	{3643, 15, 44, 0},
	{3776, 15, 42, 0},
	{3834, 15, 41, 0},
	{3958, 15, 39, 0},
	{4092, 15, 37, 0},
};

#define AMS_CAMERA_CID_BASE (V4L2_CTRL_CLASS_CAMERA | 0x2000)
#define AMS_CAMERA_CID_MIRA_REG_W (AMS_CAMERA_CID_BASE + 0)
#define AMS_CAMERA_CID_MIRA_REG_R (AMS_CAMERA_CID_BASE + 1)

/* Most significant Byte is flag, and most significant bit is unused. */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_FOR_READ 0b00000001
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_USE_BANK 0b00000010
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_BANK 0b00000100
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_CONTEXT 0b00001000
/* Use bit 5 to indicate special command, bit 1,2,3,4 for command. */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_CMD_SEL 0b00010000
/* Special command for sleep. The other 3 Bytes (addr+val) is sleep values in us. */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_SLEEP_US 0b00010000
/* Special command to enable power on (/off) when stream on (/off). */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_RESET_ON 0b00010010
/* Special command to disable power on (/off) when stream on (/off). */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_RESET_OFF 0b00010100
/* Special command to enable base register sequence upload, overwrite skip-reg-upload in dtoverlay */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_REG_UP_ON 0b00010110
/* Special command to disable base register sequence upload, overwrite skip-reg-upload in dtoverlay */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_REG_UP_OFF 0b00011000
/* Special command to manually power on */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_POWER_ON 0b00011010
/* Special command to manually power off */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_POWER_OFF 0b00011100
/* Special command to turn illumination trigger on */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_ILLUM_TRIG_ON 0b00011110
/* Special command to turn illumination trigger off */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_ILLUM_TRIG_OFF 0b00010001
/* Special command to set ILLUM_WIDTH. The other 3 Bytes (addr+val) is width value. */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_ILLUM_WIDTH 0b00010011
/* Special command to set ILLUM_DELAY. The other 3 Bytes (addr+val) is width value. */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_ILLUM_DELAY 0b00010101
/* Special command to enable ILLUM_WIDTH automatically tracking exposure time */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_ILLUM_EXP_T_ON 0b00010111
/* Special command to disable ILLUM_WIDTH automatically tracking exposure time */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_ILLUM_EXP_T_OFF 0b00011001
/* Special command to enable force_stream_ctrl */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_STREAM_CTRL_ON 0b00011011
/* Special command to disable force_stream_ctrl */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_STREAM_CTRL_OFF 0b00011101

/*
 * Bit 6&7 of flag are combined to specify I2C dev (default is Mira).
 * If bit 6&7 is 0b01, the reg_addr and reg_val are for a TBD I2C address.
 * The TBD I2C address is default to MIRA016LED_I2C_ADDR.
 * To change the TBD I2C address, set bit 6&7 to 0b10,
 * then the reg_val will become TBD I2C address.
 * The TBD I2C address is stored in mira016->tbd_client_i2c_addr.
 */
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_SEL 0b01100000
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_MIRA 0b00000000
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_TBD 0b00100000
#define AMS_CAMERA_CID_MIRA016_REG_FLAG_I2C_SET_TBD 0b01000000

/* Pre-allocated i2c_client */
#define MIRA016PMIC_I2C_ADDR 0x2D
#define MIRA016UC_I2C_ADDR 0x0A
#define MIRA016LED_I2C_ADDR 0x53

#define MIRA016_NATIVE_WIDTH 400U
#define MIRA016_NATIVE_HEIGHT 400U

#define MIRA016_PIXEL_ARRAY_LEFT 0U
#define MIRA016_PIXEL_ARRAY_TOP 0U
#define MIRA016_PIXEL_ARRAY_WIDTH 400U
#define MIRA016_PIXEL_ARRAY_HEIGHT 400U

#define MIRA016_ANALOG_GAIN_MIN 0
#define MIRA016_ANALOG_GAIN_STEP 1
#define MIRA016_ANALOG_GAIN_DEFAULT MIRA016_ANALOG_GAIN_MIN

#define MIRA016_BANK_SEL_REG 0xE000
#define MIRA016_RW_CONTEXT_REG 0xE004
#define MIRA016_CMD_REQ_1_REG 0x000A
#define MIRA016_CMD_HALT_BLOCK_REG 0x000C

// Exposure time is indicated in us
#define MIRA016_EXP_TIME_L_REG 0x000E
#define MIRA016_EXP_TIME_S_REG 0x0012
// Target frame time is indicated in us
#define MIRA016_TARGET_FRAME_TIME_REG 0x0008

#define MIRA016_SUPPORTED_XCLK_FREQ 24000000
#define MIRA016_XCLR_MIN_DELAY_US 150000
#define MIRA016_XCLR_DELAY_RANGE_US 3000

/* Embedded metadata stream structure */
#define MIRA016_EMBEDDED_LINE_WIDTH 16384
#define MIRA016_NUM_EMBEDDED_LINES 1


#define MIRA016_GDIG_PREAMP 0x0024
#define MIRA016_BIAS_RG_ADCGAIN 0x01F0
#define MIRA016_BIAS_RG_MULT 0x01F3

#define MIRA016_OTP_COMMAND 0x0066
#define MIRA016_OTP_ADDR 0x0067
#define MIRA016_OTP_START 0x0064
#define MIRA016_OTP_BUSY 0x0065
#define MIRA016_OTP_DOUT 0x006C
#define MIRA016_OTP_CAL_VALUE_DEFAULT 2250
#define MIRA016_OTP_CAL_FINE_VALUE_DEFAULT 35
#define MIRA016_OTP_CAL_FINE_VALUE_MIN 1
#define MIRA016_OTP_CAL_FINE_VALUE_MAX 60 // TODO


#define MIRA016_DEFAULT_LINE_LENGTH (2) //  (HSIZE+HBLANK)  / pixel rate

// Some timings
#define MIRA016_DATA_RATE 1000 // Mbit/s
#define MIRA016_SEQ_TIME_BASE 8 / MIRA016_DATA_RATE
#define MIRA016_LPS_CYCLE_TIME 1145
#define MIRA016_GLOB_TIME 68
#define MIRA016_ROW_LENGTH 1504 // 12b gain 1x TODO fix it for other modes
#define MIRA016_LPS_DISABLED 0
#define MIRA016_TROW_US MIRA016_ROW_LENGTH * 8 / MIRA016_DATA_RATE

#define MIRA016_READOUT_TIME MIRA016_TROW_US * (11 + MIRA016_PIXEL_ARRAY_HEIGHT)

// Default exposure is adjusted to 1 ms
#define MIRA016_GRAN_TG MIRA016_DATA_RATE * 50 / 1500  // 33
#define MIRA016_LUT_DEL_008 0

#define MIRA016_MIN_ROW_LENGTH MIRA016_ROW_LENGTH // 1042 for 8 bit
#define MIRA016_MIN_ROW_LENGTH_US (MIRA016_MIN_ROW_LENGTH * 8 / MIRA016_DATA_RATE)
#define MIRA016_EXPOSURE_MIN_US (int)(1 + (151 + MIRA016_LUT_DEL_008) * MIRA016_GRAN_TG * MIRA016_SEQ_TIME_BASE)
#define MIRA016_EXPOSURE_MAX_US (1000000)
#define MIRA016_EXPOSURE_MIN_LINES (MIRA016_EXPOSURE_MIN_US / MIRA016_DEFAULT_LINE_LENGTH)
#define MIRA016_EXPOSURE_MAX_LINES (MIRA016_EXPOSURE_MAX_US / MIRA016_DEFAULT_LINE_LENGTH)


#define MIRA016_DEFAULT_EXPOSURE_LINES 1000
#define MIRA016_DEFAULT_EXPOSURE_US MIRA016_DEFAULT_EXPOSURE_LINES *MIRA016_DEFAULT_LINE_LENGTH

// Default exposure for V4L2 is in row time

// #define MIRA016_MIN_VBLANK 11 // for 10b or 8b, 360fps
#define MIRA016_MIN_VBLANK_60 8000 // 200 fps
#define MIRA016_MIN_VBLANK_200 2100 // 200 fps
#define MIRA016_MIN_VBLANK_360 1000 // 200 fps
#define MIRA016_MAX_VBLANK 500000

#define MIRA016_DEFAULT_VBLANK_60 8000 // 200 fps
#define MIRA016_HBLANK 0

#define MIRA016_DEFAULT_LINK_FREQ 750000000
#define MIRA016_PIXEL_RATE (200000000) /*reduce factor 2 because max isp pixel rate is 380Mpix/s*/


// pixel_rate = link_freq * 2 * nr_of_lanes / bits_per_sample
// 0.9Gb/s * 2 * 1 / 12 = 157286400
// 1.5 Gbit/s * 2 * 1 / 12 = 250 000 000
/* Should match device tree link freq */
// #define MIRA016_DEFAULT_LINK_FREQ 750000000


/* Illumination trigger */
#define MIRA016_EN_TRIG_SYNC 0x001D		  // bank 1
#define MIRA016_TRIG_SYNC_DELAY 0x001A	  // bank 0
#define MIRA016_DMUX0_SEL 0x00F3		  // bank 0
#define MIRA016_TRIG_SYNC_ON_REQ_1 0x001D // bank 0
/* Illumination trigger */
#define MIRA016_EN_TRIG_ILLUM 0x001C
#define MIRA016_ILLUM_WIDTH_REG 0x0019
#define MIRA016_ILLUM_DELAY_REG 0x0016
#define MIRA016_ILLUM_WIDTH_DEFAULT (MIRA016_DEFAULT_EXPOSURE_US * MIRA016_DATA_RATE / 8)
#define MIRA016_ILLUM_DELAY_DEFAULT (1 << 19)
#define MIRA016_ILLUM_ENABLE_DEFAULT 1
#define MIRA016_ILLUM_SYNC_DEFAULT 1

/* 
 * EOB target = Electro Optical Black Level target
 * From the user guide:
 * The EOB target is the value that the sensor will try to achieve in the
 * EOB region aka under shielded pixels.
 * The EOB target is set in the register 0x005C.
 * ADC CONFIG | EOB TARGET | EFFECTIVE BLACK LEVEL
 * 8 bit fine gain | 6 | 6
 * 10 bit fine gain | 24 | 24
 * 12 bit coarse gain | 96 | 96

 */

#define MIRA016_EOB_TARGET_8BIT 6
#define MIRA016_EOB_TARGET_10BIT 24
#define MIRA016_EOB_TARGET_12BIT 96

#define MIRA016_YWIN_DIR_REG 0x0023 // YWIN direction register
#define MIRA016_YWIN_START_REG 0x002B // YWIN start register
#define MIRA016_XMIRROR_REG 0xe030 // YWIN start register


/* Mode : resolution and related config&values */
struct mira016_mode
{
	/* Frame width */
	unsigned int width;
	/* Frame height */
	unsigned int height;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* Default register values */
	struct mira016_reg_list reg_list_pre_soft_reset;
	struct mira016_reg_list reg_list_post_soft_reset;
	u32 gain_min;
	u32 gain_max;
	u32 gain_step;

	u32 min_vblank;
	u32 max_vblank;
	u32 hblank;
	u32 row_length;

	/* Format code */
	u32 code;

	/* bit_depth needed for analog gain selection */
	u8 bit_depth;
};





/* regulator supplies */
static const char *const mira016_supply_name[] = {
	// TODO(jalv): Check supply names
	/* Supplies can be enabled in any order */
	"vdd28", /*  Analog supply, 2.8 volts */
	"vdd11", /* Digital supply, 1.1 volts */
};

#define MIRA016_NUM_SUPPLIES ARRAY_SIZE(mira016_supply_name)

/*
 * The supported formats. All flip/mirror combinations have the same byte order because the sensor
 * is monochrome
 */
static const u32 mira016_mbus_formats[] = {
	MEDIA_BUS_FMT_SRGGB12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SBGGR12_1X12,

	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,

	MEDIA_BUS_FMT_SRGGB8_1X8,
	MEDIA_BUS_FMT_SGRBG8_1X8,
	MEDIA_BUS_FMT_SGBRG8_1X8,
	MEDIA_BUS_FMT_SBGGR8_1X8,

};

/* Mode configs */
/*
 * Only one mode is exposed to the public (576x768 at 12 bit).
 * Three codes (8/10/12 bit) are exposed to public.
 * The public user specifies the code.
 * That is used to specify which internal supported_mode to use.
 */
#define MIRA016_SUPPORTED_MODE_SIZE_PUBLIC 1
static const struct mira016_mode supported_modes[] = {
		{
		/* 8 bit mode */
		.width = 400,
		.height = 400,
		.crop = {.left = MIRA016_PIXEL_ARRAY_LEFT, .top = MIRA016_PIXEL_ARRAY_TOP, 
		.width = 400, .height = 400},
		.reg_list_pre_soft_reset = {
			.num_of_regs = ARRAY_SIZE(full_400_400_100fps_8b_1lane_reg_pre_soft_reset),
			.regs = full_400_400_100fps_8b_1lane_reg_pre_soft_reset,
		},
		.reg_list_post_soft_reset = {
			.num_of_regs = ARRAY_SIZE(full_400_400_100fps_8b_1lane_reg_post_soft_reset),
			.regs = full_400_400_100fps_8b_1lane_reg_post_soft_reset,
		},
		.min_vblank = MIRA016_MIN_VBLANK_60,
		.max_vblank = MIRA016_MAX_VBLANK,
		.hblank = MIRA016_HBLANK, // TODO
		.bit_depth = 8,
		.code = MEDIA_BUS_FMT_SGRBG8_1X8,
		.gain_min = 0,
		.gain_step = 1,
		.gain_max = ARRAY_SIZE(fine_gain_lut_8bit_16x) - 1,
	},
	{
		/* 10 bit highspeed / low power mode */
		.width = 400,
		.height = 400,
		.crop = {.left = MIRA016_PIXEL_ARRAY_LEFT, .top = MIRA016_PIXEL_ARRAY_TOP,
		.width = 400, .height = 400},
		.reg_list_pre_soft_reset = {
			.num_of_regs = ARRAY_SIZE(full_400_400_100fps_10b_1lane_reg_pre_soft_reset),
			.regs = full_400_400_100fps_10b_1lane_reg_pre_soft_reset,
		},
		.reg_list_post_soft_reset = {
			.num_of_regs = ARRAY_SIZE(full_400_400_100fps_10b_1lane_reg_post_soft_reset),
			.regs = full_400_400_100fps_10b_1lane_reg_post_soft_reset,
		},
		.min_vblank = MIRA016_MIN_VBLANK_60,
		.max_vblank = MIRA016_MAX_VBLANK,
		.hblank = MIRA016_HBLANK, // TODO
		.bit_depth = 10,
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.gain_min = 0,
		.gain_step = 1,
		.gain_max = ARRAY_SIZE(fine_gain_lut_10bit_hs_4x) - 1,
	},
	{
		/* 12 bit mode */
		.width = 400,
		.height = 400,
		.crop = {
			.left = MIRA016_PIXEL_ARRAY_LEFT,
			.top = MIRA016_PIXEL_ARRAY_TOP,
			.width = 400,
			.height = 400},
		.reg_list_pre_soft_reset = {
			.num_of_regs = ARRAY_SIZE(full_400_400_100fps_12b_1lane_reg_pre_soft_reset),
			.regs = full_400_400_100fps_12b_1lane_reg_pre_soft_reset,
		},
		.reg_list_post_soft_reset = {
			.num_of_regs = ARRAY_SIZE(full_400_400_100fps_12b_1lane_reg_post_soft_reset),
			.regs = full_400_400_100fps_12b_1lane_reg_post_soft_reset,
		},
		.min_vblank = MIRA016_MIN_VBLANK_60,
		.max_vblank = MIRA016_MAX_VBLANK,
		.hblank = MIRA016_HBLANK, // TODO
		.bit_depth = 12,
		.code = MEDIA_BUS_FMT_SGRBG12_1X12,
		.gain_min = 0,
		.gain_step = 24,
		.gain_max = 24, // refer to the lookup table.
	},

};

struct mira016
{
	struct v4l2_subdev sd;
	struct media_pad pad;

	struct v4l2_mbus_framefmt fmt;

	struct clk *xclk; /* system clock to MIRA016 */
	u32 xclk_freq;

	// struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[MIRA016_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *gain;
	// custom v4l2 control

	/* Current mode */
	const struct mira016_mode *mode;
	/* current bit depth, may defer from mode->bit_depth */
	u8 bit_depth;
	/* OTP_CALIBRATION_VALUE stored in OTP memory */

	// u16 otp_dark_cal_8bit;
	// u16 otp_dark_cal_10bit_hs;
	// u16 otp_dark_cal_10bit;
	// u16 otp_dark_cal_12bit;

	/* Whether to skip base register sequence upload */
	u32 skip_reg_upload;
	/* Whether to reset sensor when stream on/off */
	u32 skip_reset;
	/* Whether regulator and clk are powered on */
	u32 powered;
	/* Illumination trigger enable */
	u8 illum_enable;
	/* Illumination trigger width. Use [23:0] for 24-bit register. */
	u32 illum_width;
	/* Illumination trigger delay. Use [19:0] for 20-bit register */
	u32 illum_delay;
	/* Illumination trigger width automatically set to exposure time */
	u8 illum_width_auto;

	u32 target_frame_time_us;
	u32 row_length;
	/*
	 * Mutex for serialized access:
	 * Protect sensor module set pad format and start/stop streaming safely.
	 */
	struct mutex mutex;

	/* Streaming on/off */
	bool streaming;

	/* pmic, uC, LED */
	struct i2c_client *pmic_client;
	struct i2c_client *uc_client;
	struct i2c_client *led_client;
	/* User specified I2C device address */
	u32 tbd_client_i2c_addr;
};

static inline struct mira016 *to_mira016(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct mira016, sd);
}

static int mira016_read(struct mira016 *mira016, u16 reg, u8 *val)
{
	int ret;
	unsigned char data_w[2] = {reg >> 8, reg & 0xff};
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);

	ret = i2c_master_send(client, data_w, 2);
	/*
	 * A negative return code, or sending the wrong number of bytes, both
	 * count as an error.
	 */
	if (ret != 2)
	{
		dev_dbg(&client->dev, "%s: i2c write error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
		return ret;
	}

	ret = i2c_master_recv(client, val, 1);
	/*
	 * The only return value indicating success is 1. Anything else, even
	 * a non-negative value, indicates something went wrong.
	 */
	if (ret == 1)
	{
		ret = 0;
	}
	else
	{
		dev_dbg(&client->dev, "%s: i2c read error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
	}

	return ret;
}

static int mira016_write(struct mira016 *mira016, u16 reg, u8 val)
{
	int ret;
	unsigned char data[3] = {reg >> 8, reg & 0xff, val};
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);

	ret = i2c_master_send(client, data, 3);

	/*
	 * Writing the wrong number of bytes also needs to be flagged as an
	 * error. Success needs to produce a 0 return code.
	 */
	if (ret == 3)
	{
		ret = 0;
	}
	else
	{
		dev_dbg(&client->dev, "%s: i2c write error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
	}

	/*
	 * The code below is for debug purpose.
	 * It reads back the written values.
	 * Some registers have different read and write addresses.
	 * These registers typically have WR addr 0xE... but RD addr 0x4...
	 */
	/*
	{
		usleep_range(50, 300);
		u8 ret_val;
		u16 ret_reg;
		if (((reg >>12) & 0x000F) == 0x000E) {
			ret_reg = ((reg & 0x0FFF) | 0x4000);
		} else {
			ret_reg = reg;
		}
		ret = mira016_read(mira016, ret_reg, &ret_val);
		printk(KERN_INFO "[MIRA016]: Write reg 0x%4.4x, Read ret_reg 0x%4.4x, val = 0x%x.\n",
				reg, ret_reg, ret_val);
		if (val != ret_val) {
			printk(KERN_INFO "[MIRA016]: WARNING Write reg 0x%4.4x, val = 0x%x, read ret_reg = 0x%4.4x, ret_val = 0x%x.\n",
				reg, val, ret_reg, ret_val);
		}
	}
	*/

	return ret;
}

/*
 * mira016 is big-endian: msb of val goes to lower reg addr
 */
static int mira016_write_be16(struct mira016 *mira016, u16 reg, u16 val)
{
	int ret;
	unsigned char data[4] = {reg >> 8, reg & 0xff, (val >> 8) & 0xff, val & 0xff};
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);

	ret = i2c_master_send(client, data, 4);
	/*
	 * Writing the wrong number of bytes also needs to be flagged as an
	 * error. Success needs to produce a 0 return code.
	 */
	if (ret == 4)
	{
		ret = 0;
	}
	else
	{
		dev_dbg(&client->dev, "%s: i2c write error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
	}

	return ret;
}

/*
 * mira016 is big-endian: msb of val goes to lower reg addr
 */
static int mira016_write_be24(struct mira016 *mira016, u16 reg, u32 val)
{
	int ret;
	unsigned char data[5] = {reg >> 8, reg & 0xff, (val >> 16) & 0xff, (val >> 8) & 0xff, val & 0xff};
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);

	ret = i2c_master_send(client, data, 5);
	/*
	 * Writing the wrong number of bytes also needs to be flagged as an
	 * error. Success needs to produce a 0 return code.
	 */
	if (ret == 5)
	{
		ret = 0;
	}
	else
	{
		dev_dbg(&client->dev, "%s: i2c write error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
	}

	return ret;
}

/*
 * mira016 is big-endian: msb of val goes to lower reg addr
 */
static int mira016_write_be32(struct mira016 *mira016, u16 reg, u32 val)
{
	int ret;
	unsigned char data[6] = {reg >> 8, reg & 0xff, (val >> 24) & 0xff, (val >> 16) & 0xff, (val >> 8) & 0xff, val & 0xff};
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);

	ret = i2c_master_send(client, data, 6);
	/*
	 * Writing the wrong number of bytes also needs to be flagged as an
	 * error. Success needs to produce a 0 return code.
	 */
	if (ret == 6)
	{
		ret = 0;
	}
	else
	{
		dev_dbg(&client->dev, "%s: i2c write error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
	}

	return ret;
}

/*
 * mira016 OTP 32-bit val on I2C is big-endian. However, val content can be little-endian.
 */
static int mira016_read_be32(struct mira016 *mira016, u16 reg, u32 *val)
{
	int ret;
	unsigned char data_w[2] = {reg >> 8, reg & 0xff};
	/* Big-endian 32-bit buffer. */
	unsigned char data_r[4];
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);

	ret = i2c_master_send(client, data_w, 2);
	/*
	 * A negative return code, or sending the wrong number of bytes, both
	 * count as an error.
	 */
	if (ret != 2)
	{
		dev_dbg(&client->dev, "%s: i2c write error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
		return ret;
	}

	ret = i2c_master_recv(client, data_r, 4);
	*val = (u32)((data_r[0] << 24) | (data_r[1] << 16) | (data_r[2] << 8) | data_r[3]);
	/*
	 * The only return value indicating success is 4. Anything else, even
	 * a non-negative value, indicates something went wrong.
	 */
	if (ret == 4)
	{
		ret = 0;
	}
	else
	{
		dev_dbg(&client->dev, "%s: i2c read error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
	}

	return ret;
}

/* Write a list of registers */
static int mira016_write_regs(struct mira016 *mira016,
							  const struct mira016_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++)
	{
		ret = mira016_write(mira016, regs[i].address, regs[i].val);
		if (ret)
		{
			dev_err_ratelimited(&client->dev,
								"Failed to write reg 0x%4.4x. error = %d\n",
								regs[i].address, ret);

			return ret;
		}
		else
		{
			// Debug code below
			// u8 val;
			// ret = mira016_read(mira016, regs[i].address, &val);
			// printk(KERN_INFO "[MIRA016]: Read reg 0x%4.4x, val = 0x%x.\n",
			// 		regs[i].address, val);
		}
	}

	return 0;
}

/*
 * Read OTP memory: 8-bit addr and 32-bit value
 */
static int mira016_otp_read(struct mira016 *mira016, u8 addr, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);
	u8 busy_status = 1;
	int poll_cnt = 0;
	int poll_cnt_max = 10;
	int ret;
	mira016_write(mira016, MIRA016_BANK_SEL_REG, 0);
	mira016_write(mira016, MIRA016_OTP_COMMAND, 0);
	mira016_write(mira016, MIRA016_OTP_ADDR, addr);
	mira016_write(mira016, MIRA016_OTP_START, 1);
	usleep_range(15, 50);
	mira016_write(mira016, MIRA016_OTP_START, 0);
	for (poll_cnt = 0; poll_cnt < poll_cnt_max; poll_cnt++)
	{
		mira016_read(mira016, MIRA016_OTP_BUSY, &busy_status);
		if (busy_status == 0)
		{
			break;
		}
		else
		{
			usleep_range(5, 10);
		}
	}
	if (poll_cnt < poll_cnt_max && busy_status == 0)
	{
		usleep_range(15, 50);
		ret = mira016_read_be32(mira016, MIRA016_OTP_DOUT, val);
		printk(KERN_INFO "[MIRA016]: Read OTP 0x%x, val = 0x%x.\n",
		 		addr,*val);
	}
	else
	{
		dev_dbg(&client->dev, "%s: OTP memory busy, skip raeding addr: 0x%X\n",
				__func__, addr);
		ret = -EINVAL;
	}

	return ret;
}

/* Write PMIC registers, and can be reused to write microcontroller reg. */
static int mira016pmic_write(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	unsigned char data[2] = {reg & 0xff, val};

	ret = i2c_master_send(client, data, 2);
	/*
	 * Writing the wrong number of bytes also needs to be flagged as an
	 * error. Success needs to produce a 0 return code.
	 */
	if (ret == 2)
	{
		ret = 0;
	}
	else
	{
		dev_dbg(&client->dev, "%s: i2c write error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
	}

	return ret;
}

static int mira016pmic_read(struct i2c_client *client, u8 reg, u8 *val)
{
	struct i2c_msg msgs[2];
	u8 addr_buf[1] = {reg & 0xff};
	u8 data_buf[1] = {0};
	int ret;

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = 1;
	msgs[1].buf = &data_buf[0];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = (u8)(data_buf[0]);

	return 0;
}

/* Power/clock management functions */
static int mira016_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira016 *mira016 = to_mira016(sd);
	int ret = -EINVAL;

	ret = regulator_bulk_enable(MIRA016_NUM_SUPPLIES, mira016->supplies);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable regulators\n",
			__func__);
		goto reg_off;
	}
	ret = clk_prepare_enable(mira016->xclk);
	if (ret) {
		dev_err(&client->dev, "%s: failed to enable clock\n", __func__);
		goto clk_off;
	}
	fsleep(MIRA016_XCLR_MIN_DELAY_US);

	return 0;

clk_off:
	clk_disable_unprepare(mira016->xclk);
reg_off:
	ret = regulator_bulk_disable(MIRA016_NUM_SUPPLIES, mira016->supplies);
	return ret;
}


static int mira016_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira016 *mira016 = to_mira016(sd);
	(void)mira016;

	clk_disable_unprepare(mira016->xclk);
	regulator_bulk_disable(MIRA016_NUM_SUPPLIES, mira016->supplies);

	return 0;
}

static int mira016_write_illum_trig_regs(struct mira016 *mira016)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&mira016->sd);
	int ret = 0;
	u32 lps_time = 0;
	u32 width_adjust = 0;

	// Set context bank 1A or bank 1B
	ret = mira016_write(mira016, MIRA016_RW_CONTEXT_REG, 0);
	if (ret)
	{
		dev_err(&client->dev, "Error setting RW_CONTEXT.");
		return ret;
	}

	// Set conetxt bank 0 or 1
	ret = mira016_write(mira016, MIRA016_BANK_SEL_REG, 1);
	if (ret)
	{
		dev_err(&client->dev, "Error setting BANK_SEL_REG.");
		return ret;
	}

	// Enable or disable illumination trigger
	printk(KERN_INFO "[MIRA016]: Writing EN_TRIG_ILLUM to %d.\n", mira016->illum_enable);
	ret = mira016_write(mira016, MIRA016_EN_TRIG_ILLUM, mira016->illum_enable);
	if (ret)
	{
		dev_err(&client->dev, "Error setting EN_TRIG_ILLUM to %d.", mira016->illum_enable);
		return ret;
	}

	if (MIRA016_LPS_DISABLED)
	{
		// Set illumination width. Write 24 bits. All 24 bits are valid.
		printk(KERN_INFO "[MIRA016]: LPS DISABLED. Writing ILLUM_WIDTH to %u.\n", mira016->illum_width);
		ret = mira016_write_be24(mira016, MIRA016_ILLUM_WIDTH_REG, mira016->illum_width);
		if (ret)
		{
			dev_err(&client->dev, "LPS DISABLED. Error setting ILLUM_WIDTH to %u.", mira016->illum_width);
			return ret;
		}
	}
	else
	{
		// LSP active, adjust pulse with to compensate for dead time during exposure
		// INPUT PARAMS: LPS_CYCLE_TIME, EXP_TIME, FRAME_TIME, GLOB_TIME, READOUT_TIME
		//
		//
		// printk(KERN_INFO "[MIRA016]: LPS DISABLED. Exposure name is  to %u.\n", mira016->exposure->name);
		u32 cur_exposure = (mira016->exposure->val * MIRA016_DEFAULT_LINE_LENGTH);

		// printk(KERN_INFO "[MIRA016]: LPS ENABLED. Exposure cur is  to %u.\n", mira016->exposure->val);
		// printk(KERN_INFO "[MIRA016]: LPS ENABLED. Exposure cur IN US  is  to %u.\n", cur_exposure);

		u32 readout_time = (11 + MIRA016_PIXEL_ARRAY_HEIGHT) * mira016->row_length * 8 / MIRA016_DATA_RATE;

		// printk(KERN_INFO "[MIRA016]: LPS ENABLED. MIRA016_LPS_CYCLE_TIME is  to %u.\n", MIRA016_LPS_CYCLE_TIME);
		// printk(KERN_INFO "[MIRA016]: LPS ENABLED. MIRA016_GLOB_TIME is  to %u.\n", MIRA016_GLOB_TIME);
		// printk(KERN_INFO "[MIRA016]: LPS ENABLED. frame time is  to %u.\n", mira016->target_frame_time_us);
		// printk(KERN_INFO "[MIRA016]: LPS ENABLED. glob time is  to %u.\n", MIRA016_GLOB_TIME);
		// printk(KERN_INFO "[MIRA016]: LPS ENABLED. read time is  to %u.\n", MIRA016_READOUT_TIME);
		// printk(KERN_INFO "[MIRA016]: LPS ENABLED. new read time is  to %u.\n", readout_time);
		// printk(KERN_INFO "[MIRA016]: LPS ENABLED. mira016->target_frame_time_us - MIRA016_GLOB_TIME - readout_time is  to %u.\n", mira016->target_frame_time_us - MIRA016_GLOB_TIME - MIRA016_READOUT_TIME);

		// case 1: EXP_TIME < LPS_CYCLE_TIME
		if (cur_exposure < MIRA016_LPS_CYCLE_TIME)
		{
			printk(KERN_INFO "[MIRA016]: LPS CASE 1 to %u.\n", mira016->illum_width);
			lps_time = 0;
		}
		// case 2: LPS_ CYCLE_ TIME<EXP_ TIME≤FRAME_ TIME-GLOB_ TIME-READOUT_TIME
		else if ((MIRA016_LPS_CYCLE_TIME < cur_exposure) && (cur_exposure < (mira016->target_frame_time_us - MIRA016_GLOB_TIME - readout_time)))
		{
			lps_time = cur_exposure - MIRA016_LPS_CYCLE_TIME;
			printk(KERN_INFO "[MIRA016]: LPS CASE 2 - LPS TIME is %u.\n", lps_time);
		}
		// case 3: LPS_ CYCLE_ TIME≤FRAME_ TIME-GLOB_ TIME-READOUT_TIME<EXP_ TIME
		else if ((MIRA016_LPS_CYCLE_TIME < (mira016->target_frame_time_us - MIRA016_GLOB_TIME - readout_time)) && ((mira016->target_frame_time_us - MIRA016_GLOB_TIME - readout_time) < cur_exposure))
		{
			lps_time = (mira016->target_frame_time_us - MIRA016_GLOB_TIME - readout_time) - MIRA016_LPS_CYCLE_TIME;
			printk(KERN_INFO "[MIRA016]: LPS CASE 3 - LPS TIME is %u.\n", lps_time);
		}
		// case 4: FRAME_ TIME-GLOB_ TIME-READOUT_ TIME≤LPS_ CYCLE_ TIME<EXP_ TIME
		else if (((mira016->target_frame_time_us - MIRA016_GLOB_TIME - readout_time) < MIRA016_LPS_CYCLE_TIME) && (MIRA016_LPS_CYCLE_TIME < cur_exposure))
		{
			printk(KERN_INFO "[MIRA016]: LPS CASE 4 to %u.\n", mira016->illum_width);
			lps_time = 0;
		}
		else
		{
			printk(KERN_INFO "[MIRA016]: LPS CASE 5 invalid to %u.\n", mira016->illum_width);
		}

		width_adjust = (lps_time > 0 ? lps_time * 1500 / 8 - 30 : 0);
		printk(KERN_INFO "[MIRA016]: LPS ENABLE -s width adjust is  %u.\n", width_adjust);

		ret = mira016_write_be24(mira016, MIRA016_ILLUM_WIDTH_REG, mira016->illum_width - width_adjust);

		if (ret)
		{
			dev_err(&client->dev, "LPS ENABLED. Error setting ILLUM_WIDTH to %u.", mira016->illum_width - width_adjust);
			return ret;
		}
	}
	return ret;
}


// Returns the maximum exposure time in microseconds (reg value)
static u32 mira016_calculate_max_exposure_time(u32 row_length, u32 vsize,
											   u32 vblank)
{
	(void)(row_length);
	(void)(vsize);
	(void)(vblank);
	/* MIRA016 does not have a max exposure limit besides register bits */
	return MIRA016_EXPOSURE_MAX_LINES;
	return MIRA016_EXPOSURE_MAX_LINES;
}


static int mira016_write_exposure_reg(struct mira016 *mira016, u32 exposure_lines)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&mira016->sd);
	const u32 min_exposure = MIRA016_EXPOSURE_MIN_US;
	u32 max_exposure = mira016->exposure->maximum;
	u32 exposure = exposure_lines * MIRA016_DEFAULT_LINE_LENGTH;
	u32 ret = 0;

	if (exposure < min_exposure)
	{
		exposure = min_exposure;
	}
	if (exposure > max_exposure)
	{
		exposure = max_exposure;
	}

	/* Write Bank 1 context 0 */
	ret = mira016_write(mira016, MIRA016_RW_CONTEXT_REG, 0);
	ret = mira016_write(mira016, MIRA016_BANK_SEL_REG, 1);
	ret = mira016_write_be32(mira016, MIRA016_EXP_TIME_L_REG, exposure);
	/* Write Bank 1 context 1 */
	ret = mira016_write(mira016, MIRA016_RW_CONTEXT_REG, 1);
	ret = mira016_write_be32(mira016, MIRA016_EXP_TIME_L_REG, exposure);
	if (ret)
	{
		dev_err_ratelimited(&client->dev, "Error setting exposure time to %d", exposure);
		return -EINVAL;
	}
	if (mira016->illum_width_auto == 1)
	{
		mira016->illum_width = exposure * MIRA016_DATA_RATE / 8;
		mira016_write_illum_trig_regs(mira016);
	}

	return 0;
}


static int mira016_write_target_frame_time_reg(struct mira016 *mira016, u32 target_frame_time_us)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&mira016->sd);
	u32 ret = 0;

	/* Write Bank 1 context 0 */
	ret = mira016_write(mira016, MIRA016_RW_CONTEXT_REG, 0);
	ret = mira016_write(mira016, MIRA016_BANK_SEL_REG, 1);
	ret = mira016_write_be32(mira016, MIRA016_TARGET_FRAME_TIME_REG, target_frame_time_us);
	/* Write Bank 1 context 1 */
	ret = mira016_write(mira016, MIRA016_RW_CONTEXT_REG, 1);
	ret = mira016_write_be32(mira016, MIRA016_TARGET_FRAME_TIME_REG, target_frame_time_us);
	if (ret)
	{
		dev_err_ratelimited(&client->dev, "Error setting target frame time to %d", target_frame_time_us);
		return -EINVAL;
	}

	return 0;
}

static int mira016_write_start_streaming_regs(struct mira016 *mira016)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&mira016->sd);
	int ret = 0;

	// Set conetxt bank 0 or 1
	ret = mira016_write(mira016, MIRA016_BANK_SEL_REG, 0);
	if (ret)
	{
		dev_err(&client->dev, "Error setting BANK_SEL_REG.");
		return ret;
	}

	// Set context bank 1A or bank 1B
	ret = mira016_write(mira016, MIRA016_RW_CONTEXT_REG, 0);
	if (ret)
	{
		dev_err(&client->dev, "Error setting RW_CONTEXT.");
		return ret;
	}

	// Raising CMD_REQ_1 to 1 for REQ_EXP
	ret = mira016_write(mira016, MIRA016_CMD_REQ_1_REG,
						1);
	if (ret)
	{
		dev_err(&client->dev, "Error setting CMD_REQ_1 to 1 for REQ_EXP.");
		return ret;
	}

	usleep_range(10, 20);

	// Setting CMD_REQ_1 tp 0 for REQ_EXP
	ret = mira016_write(mira016, MIRA016_CMD_REQ_1_REG,
						0);
	if (ret)
	{
		dev_err(&client->dev, "Error setting CMD_REQ_1 to 0 for REQ_EXP.");
		return ret;
	}
	usleep_range(10, 20);

	return ret;
}

static int mira016_write_stop_streaming_regs(struct mira016 *mira016)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&mira016->sd);
	int ret = 0;
	printk(KERN_INFO "[MIRA016]: mira016_write_stop_streaming_regs  function.\n");

	// Set conetxt bank 0 or 1
	ret = mira016_write(mira016, MIRA016_BANK_SEL_REG, 0);
	if (ret)
	{
		dev_err(&client->dev, "Error setting BANK_SEL_REG.");
		return ret;
	}

	// Raising CMD_HALT_BLOCK to 1 to stop streaming
	ret = mira016_write(mira016, MIRA016_CMD_HALT_BLOCK_REG,
						1);
	if (ret)
	{
		dev_err(&client->dev, "Error setting CMD_HALT_BLOCK to 1.");
		return ret;
	}

	usleep_range(10, 20);

	// Setting CMD_HALT_BLOCK to 0 to stop streaming
	ret = mira016_write(mira016, MIRA016_CMD_HALT_BLOCK_REG,
						0);
	if (ret)
	{
		dev_err(&client->dev, "Error setting CMD_HALT_BLOCK to 0.");
		return ret;
	}
	usleep_range(10, 20);


	return ret;
}


static int mira016_write_analog_gain_reg(struct mira016 *mira016, u8 gain)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&mira016->sd);
	u32 num_of_regs;
	u32 ret = 0;
	u32 wait_us = 20000;
	u16 cds_offset = 1700;
	u16 dark_offset_100 = 1794; // noncont clock
	u16 scale_factor = 1;
	u16 preamp_gain_inv = 1;
	u16 preamp_gain = 1;

	u16 analog_gain = 1;
	u16 offset_clipping = 0;
	u16 scaled_offset = 0;
	printk(KERN_INFO "[MIRA016]: Write analog gain %u  mira016->bit_depth %u mira016->mode->bit_depth %u",gain, mira016->bit_depth, mira016->mode->bit_depth);

	// Select partial register sequence according to bit depth
	if (mira016->bit_depth == 12)
	{
		// Select register sequence according to gain value
		if (gain == 1)
		{
			mira016_write_stop_streaming_regs(mira016);
			usleep_range(wait_us, wait_us + 100);
			printk(KERN_INFO "[mira016]: Write reg sequence for analog gain x1 in 12 bit mode");
			num_of_regs = ARRAY_SIZE(partial_analog_gain_x1_12bit);
			ret = mira016_write_regs(mira016, partial_analog_gain_x1_12bit, num_of_regs);
			mira016_write_start_streaming_regs(mira016);
			mira016->row_length = 1504;
		}
		else if (gain == 2)
		{
			mira016_write_stop_streaming_regs(mira016);
			usleep_range(wait_us, wait_us + 100);
			printk(KERN_INFO "[mira016]: Write reg sequence for analog gain x2 in 12 bit mode");
			num_of_regs = ARRAY_SIZE(partial_analog_gain_x2_12bit);
			ret = mira016_write_regs(mira016, partial_analog_gain_x2_12bit, num_of_regs);
			mira016_write_start_streaming_regs(mira016);
			mira016->row_length = 2056;
		}
		else
		{
			// Other gains are not supported
			printk(KERN_INFO "[mira016]: Ignore analog gain %d in 12 bit mode", gain);
		}
	}
	else if (mira016->bit_depth == 10)
	{
		// Select register sequence according to gain value
		if (gain < ARRAY_SIZE(fine_gain_lut_10bit_hs_4x))
		{
			u32 analog_gain = fine_gain_lut_10bit_hs_4x[gain].analog_gain;
			u8 gdig_preamp = fine_gain_lut_10bit_hs_4x[gain].gdig_preamp;
			u8 rg_adcgain = fine_gain_lut_10bit_hs_4x[gain].rg_adcgain;
			u8 rg_mult = fine_gain_lut_10bit_hs_4x[gain].rg_mult;
			/* otp_cal_val should come from OTP, but OTP may have incorrect value. */
			u16 preamp_gain_inv = 16 / (gdig_preamp + 1); // invert because fixed point arithmetic

	
			/* Stop streaming and wait for frame data transmission done */
			mira016_write_stop_streaming_regs(mira016);
			usleep_range(wait_us, wait_us + 100);
			/* Write fine gain registers */
			printk(KERN_INFO "[MIRA016]: Write reg sequence for analog gain %u in 10 bit mode", gain);
			printk(KERN_INFO "[MIRA016]: analoggain: %u,gdig_preamp: %u rg_adcgain: %u, rg_mult: %u\n",
				   analog_gain, gdig_preamp, rg_adcgain, rg_mult );
			mira016_write(mira016, MIRA016_RW_CONTEXT_REG, 0);
			mira016_write(mira016, MIRA016_BANK_SEL_REG, 1);
			mira016_write(mira016, MIRA016_GDIG_PREAMP, gdig_preamp);
			mira016_write(mira016, MIRA016_BANK_SEL_REG, 0);
			mira016_write(mira016, MIRA016_BIAS_RG_ADCGAIN, rg_adcgain);
			mira016_write(mira016, MIRA016_BIAS_RG_MULT, rg_mult);
			/* Resume streaming */
			mira016_write_start_streaming_regs(mira016);
		}



		else
		{
			// Other gains are not supported
			printk(KERN_INFO "[mira016]: Ignore analog gain %d in 12 bit mode", gain);
		}
	}
	else if (mira016->bit_depth == 8)
	{
		dark_offset_100 = 72; // noncont clock
		scale_factor = 16;
		cds_offset = 1540;

		if (gain < ARRAY_SIZE(fine_gain_lut_8bit_16x))
		{
			u32 analog_gain = fine_gain_lut_8bit_16x[gain].analog_gain;
			u8 gdig_preamp = fine_gain_lut_8bit_16x[gain].gdig_preamp;
			u8 rg_adcgain = fine_gain_lut_8bit_16x[gain].rg_adcgain;
			u8 rg_mult = fine_gain_lut_8bit_16x[gain].rg_mult;
			/* otp_cal_val should come from OTP, but OTP may have incorrect value. */
			u16 preamp_gain_inv = 16 / (gdig_preamp + 1);

				//  = (int)(cds_offset - (target_black_level*digital_gain - offset_clipping)) < 0 ? 0 : (int)(cds_offset - (target_black_level*digital_gain - offset_clipping));

			// u16 offset_clipping = (offset_clipping_calc < 0) ? 0 : (int)(offset_clipping_calc);
			/* Stop streaming and wait for frame data transmission done */
			mira016_write_stop_streaming_regs(mira016);
			usleep_range(wait_us, wait_us + 100);
			/* Write fine gain registers */
			printk(KERN_INFO "[MIRA016]: Write reg sequence for analog gain %u in 8 bit mode", gain);
			printk(KERN_INFO "[MIRA016]: analoggain: %u,gdig_preamp: %u rg_adcgain: %u, rg_mult: %u\n",
				   analog_gain, gdig_preamp, rg_adcgain, rg_mult );
			mira016_write(mira016, MIRA016_RW_CONTEXT_REG, 0);
			mira016_write(mira016, MIRA016_BANK_SEL_REG, 1);
			mira016_write(mira016, MIRA016_GDIG_PREAMP, gdig_preamp);
			mira016_write(mira016, MIRA016_BANK_SEL_REG, 0);
			mira016_write(mira016, MIRA016_BIAS_RG_ADCGAIN, rg_adcgain);
			mira016_write(mira016, MIRA016_BIAS_RG_MULT, rg_mult);
			/* Resume streaming */
			mira016_write_start_streaming_regs(mira016);
		}
		else
		{
			// Other gains are not supported
			printk(KERN_INFO "[mira016]: Ignore analog gain %d in 8 bit mode", gain);
		}
	}
	else
	{
		// Other bit depths are not supported
		printk(KERN_INFO "[mira016]: Ignore analog gain in %u bit mode", mira016->mode->bit_depth);
	}

	if (ret)
	{
		dev_err(&client->dev, "%s failed to set mode because wrong gain\n", __func__);
	}

	// Always return 0 even if it fails
	return 0;
}


/* Get bayer order based on flip setting. */
static u32 mira016_get_format_code(struct mira016 *mira016, u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mira016_mbus_formats); i++)
		if (mira016_mbus_formats[i] == code)
			break;

	if (i >= ARRAY_SIZE(mira016_mbus_formats))
		i = 0;

	i = (i & ~3) | (mira016->vflip->val ? 2 : 0) | (mira016->hflip->val ? 0 : 1);


	return mira016_mbus_formats[i];
}


static void mira016_update_pad_format(struct mira016 *mira016,
	const struct mira016_mode *mode,
	struct v4l2_mbus_framefmt *fmt, u32 code)
{
	/* Bayer order varies with flips */
	fmt->code = mira016_get_format_code(mira016, code);
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int mira016_set_pad_format(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_format *fmt)
{
	struct mira016 *mira016 = to_mira016(sd);
	const struct mira016_mode *mode;
	struct v4l2_mbus_framefmt *format;
	struct v4l2_rect *crop;

	u32 max_exposure = 0, default_exp = 0;

	// /* Validate format or use default */


	mode = v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes), width,
				      height, fmt->format.width,
				      fmt->format.height);

	mira016_update_pad_format(mira016, mode, &fmt->format, fmt->format.code);

	format = v4l2_subdev_state_get_format(state, 0);
	*format = fmt->format;

	crop = v4l2_subdev_state_get_crop(state, 0);
	crop->width = format->width * 1;
	crop->height = format->height * 1;
	crop->left = MIRA016_PIXEL_ARRAY_LEFT;
	crop->top = MIRA016_PIXEL_ARRAY_TOP;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		// mira016->fmt = fmt->format;
		// mira016->mode = mode;

		// Update controls based on new mode (range and current value).
		max_exposure = mira016_calculate_max_exposure_time(
			mira016->mode->height, mira016->mode->min_vblank,
			mira016->mode->row_length);
		default_exp = (max_exposure < MIRA016_DEFAULT_EXPOSURE_LINES) ?
				      max_exposure :
				      MIRA016_DEFAULT_EXPOSURE_LINES;
		__v4l2_ctrl_modify_range(mira016->exposure,
					 MIRA016_EXPOSURE_MIN_LINES, max_exposure, 1,
					 default_exp);

		// Update pixel rate based on new mode.
		__v4l2_ctrl_modify_range(mira016->pixel_rate,
					MIRA016_PIXEL_RATE,
					MIRA016_PIXEL_RATE, 1,
					MIRA016_PIXEL_RATE);

					 			
		// Update hblank based on new mode.
		__v4l2_ctrl_modify_range(mira016->hblank, mira016->mode->hblank,
					 mira016->mode->hblank, 1,
					 mira016->mode->hblank);

		__v4l2_ctrl_modify_range(mira016->vblank,
					 mira016->mode->min_vblank,
					 mira016->mode->max_vblank, 1,
					 mira016->mode->min_vblank);

		__v4l2_ctrl_s_ctrl(mira016->vblank, mira016->mode->min_vblank);
	}

	return 0;
}


static int mira016_set_framefmt(struct mira016 *mira016,
	struct v4l2_subdev_state *state){
	const struct v4l2_mbus_framefmt *format;
	const struct v4l2_rect *crop;
	int ret = 0;
	format = v4l2_subdev_state_get_format(state, 0);
	crop = v4l2_subdev_state_get_crop(state, 0);
	// TODO: There is no easy way to change frame format
	switch (format->code) {
	case MEDIA_BUS_FMT_Y8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	printk(KERN_INFO "[MIRA016]: mira016_set_framefmt() selects 8 bit mode.\n");
		mira016->mode = &supported_modes[0];
		mira016->bit_depth = 8;
		__v4l2_ctrl_modify_range(mira016->gain,
								 0, ARRAY_SIZE(fine_gain_lut_8bit_16x) - 1, 1, 0);
		return 0;
	case MEDIA_BUS_FMT_Y10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SBGGR10_1X10:		printk(KERN_INFO "[MIRA016]: mira016_set_framefmt() selects 10 bit mode.\n");
		mira016->mode = &supported_modes[1];
		mira016->bit_depth = 10;
		__v4l2_ctrl_modify_range(mira016->gain,
								 0, ARRAY_SIZE(fine_gain_lut_10bit_hs_4x) - 1, 1, 0);
		return 0;
	case MEDIA_BUS_FMT_Y12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:	
		printk(KERN_INFO "[MIRA016]: mira016_set_framefmt() selects 12 bit mode.\n");
		mira016->mode = &supported_modes[2];
		mira016->bit_depth = 12;
		__v4l2_ctrl_modify_range(mira016->gain,
								 mira016->mode->gain_min, mira016->mode->gain_max,
								 mira016->mode->gain_step, mira016->mode->gain_min);
		return 0;
	default:
		printk(KERN_ERR "Unknown format requested %d", mira016->fmt.code);
	}

	return -EINVAL;
}


static int mira016_get_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		sel->r = *v4l2_subdev_state_get_crop(state, 0);
		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = MIRA016_NATIVE_WIDTH;
		sel->r.height = MIRA016_NATIVE_HEIGHT;
		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = MIRA016_PIXEL_ARRAY_TOP;
		sel->r.left = MIRA016_PIXEL_ARRAY_LEFT;
		sel->r.width = MIRA016_PIXEL_ARRAY_WIDTH;
		sel->r.height = MIRA016_PIXEL_ARRAY_HEIGHT;
		return 0;
	}

	return -EINVAL;
}


static int mira016_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.code = MEDIA_BUS_FMT_SGRBG8_1X8,
			.width = supported_modes[0].width,
			.height = supported_modes[0].height,
		},
	};

	mira016_set_pad_format(sd, state, &fmt);

	return 0;
}

static int mira016_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mira016 *mira016 =
		container_of(ctrl->handler, struct mira016, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);
	int ret = 0;
	u32 target_frame_time_us;

	// Debug print
	// printk(KERN_INFO "[MIRA016]: mira016_set_ctrl() id: 0x%X value: 0x%X.\n", ctrl->id, ctrl->val);

	if (ctrl->id == V4L2_CID_VBLANK)
	{
		int exposure_max, exposure_def;

		/* Update max exposure while meeting expected vblanking */
		exposure_max = mira016_calculate_max_exposure_time(MIRA016_MIN_ROW_LENGTH,
														   mira016->mode->height,
														   ctrl->val);
		exposure_def = (exposure_max < MIRA016_DEFAULT_EXPOSURE_LINES) ? exposure_max : MIRA016_DEFAULT_EXPOSURE_LINES;
		exposure_def = (exposure_max < MIRA016_DEFAULT_EXPOSURE_LINES) ? exposure_max : MIRA016_DEFAULT_EXPOSURE_LINES;
		__v4l2_ctrl_modify_range(mira016->exposure,
								 mira016->exposure->minimum,
								 (int)( exposure_max ), mira016->exposure->step,
								 (int)( exposure_def ));
	}

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(&client->dev) == 0)
	{
		dev_info(&client->dev,
				 "device in use, ctrl(id:0x%x,val:0x%x) is not handled\n",
				 ctrl->id, ctrl->val);
		return 0;
	}

	if (mira016->skip_reg_upload == 0)
	{
		switch (ctrl->id)
		{
		case V4L2_CID_ANALOGUE_GAIN:
			printk(KERN_INFO "[MIRA016]: V4L2_CID_ANALOGUE_GAIN: = %u !!!!!!!!!!!!!\n",
					ctrl->val);
			ret = mira016_write_analog_gain_reg(mira016, ctrl->val);
			break;
		case V4L2_CID_EXPOSURE:
			printk(KERN_INFO "[MIRA016]: V4L2_CID_EXPOSURE: exp line = %u \n",
					ctrl->val);
			ret = mira016_write_exposure_reg(mira016, ctrl->val);
			break;
		case V4L2_CID_TEST_PATTERN:

			break;
		case V4L2_CID_HFLIP:
			// TODO: HFLIP requires multiple register writes
			// ret = mira016_write(mira016, MIRA016_HFLIP_REG,
			//		        ctrl->val);
			printk(KERN_ERR "[MIRA016]: HFLIP: set %d.\n", ctrl->val);

			if (ctrl->val == 0)
			{
				printk(KERN_ERR "[MIRA016]: HFLIP: disable %d.\n", ctrl->val);
				ret = mira016_write(mira016, MIRA016_BANK_SEL_REG, 0x01);
				ret = mira016_write(mira016, MIRA016_XMIRROR_REG, 0);

			}
			else
			{
				printk(KERN_ERR "[MIRA016]: HFLIP: enable %d.\n", ctrl->val);
				ret = mira016_write(mira016, MIRA016_BANK_SEL_REG, 0x01);
				ret = mira016_write(mira016, MIRA016_XMIRROR_REG, 1);
			}
			break;
		case V4L2_CID_VFLIP:
			// {0x0029, 0x1},	// None
			// {0x002A, 0x90}, // None
			// {0x002B, 0x0},	// None
			// {0x002C, 0xE},	// None
			// TODO: VFLIP seems not supported in MIRA016
			printk(KERN_ERR "[MIRA016]: VFLIP: set %d.\n", ctrl->val);
			ret = mira016_write(mira016, MIRA016_BANK_SEL_REG, 0x00);

			if (ctrl->val == 0)
			{
				printk(KERN_ERR "[MIRA016]: VFLIP: disable %d.\n", ctrl->val);
				ret = mira016_write(mira016, MIRA016_YWIN_DIR_REG, 0x0);
				ret = mira016_write_be16(mira016, MIRA016_YWIN_START_REG, 14);

			}
			else
			{
				printk(KERN_ERR "[MIRA016]: VFLIP: enable %d.\n", ctrl->val);
				ret = mira016_write(mira016, MIRA016_YWIN_DIR_REG, 0x1);
				ret = mira016_write_be16(mira016, MIRA016_YWIN_START_REG, 413);
			}

			break;
		case V4L2_CID_VBLANK:
			/*
			 * In libcamera, frame time (== 1/framerate) is controlled by VBLANK:
			 * TARGET_FRAME_TIME (us) = 1000000 * ((1/PIXEL_RATE)*(WIDTH+HBLANK)*(HEIGHT+VBLANK))
			 */
			mira016->target_frame_time_us = (u32)((u64)(1000000 * (u64)(mira016->mode->width + mira016->mode->hblank) * (u64)(mira016->mode->height + ctrl->val)) / MIRA016_PIXEL_RATE);
			// Debug print
			printk(KERN_INFO "[MIRA016]: mira016_write_target_frame_time_reg target_frame_time_us = %u.\n",
				   mira016->target_frame_time_us);
			printk(KERN_INFO "[MIRA016]: width %d, hblank %d, vblank %d, height %d, ctrl->val %d.\n",
				   mira016->mode->width, mira016->mode->hblank, mira016->mode->min_vblank, mira016->mode->height, ctrl->val);
			ret = mira016_write_target_frame_time_reg(mira016, mira016->target_frame_time_us);
			break;
		case V4L2_CID_HBLANK:
			printk(KERN_INFO "[MIRA016]: V4L2_CID_HBLANK CALLED = %d.\n",
				   ctrl->val);
			break;
		default:
			dev_info(&client->dev,
					 "ctrl(id:0x%x,val:0x%x) is not handled\n",
					 ctrl->id, ctrl->val);
			ret = -EINVAL;
			break;
		}
	}

	pm_runtime_put(&client->dev);

	// TODO: FIXIT
	return ret;
}

static const struct v4l2_ctrl_ops mira016_ctrl_ops = {
	.s_ctrl = mira016_set_ctrl,
};

// This function should enumerate all the media bus formats for the requested pads. If the requested
// format index is beyond the number of avaialble formats it shall return -EINVAL;
static int mira016_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct mira016 *mira016 = to_mira016(sd);

	if (code->index >= (ARRAY_SIZE(mira016_mbus_formats) / 4))
		return -EINVAL;

	code->code = mira016_get_format_code(
		mira016, mira016_mbus_formats[code->index * 4]);

	return 0;
}

static int mira016_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	struct mira016 *mira016 = to_mira016(sd);
	u32 code;

	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	code = mira016_get_format_code(mira016, fse->code);
	if (fse->code != code)
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = supported_modes[fse->index].height;
	fse->max_height = fse->min_height;


	return 0;
}


static int mira016_start_streaming(struct mira016 *mira016,
				struct v4l2_subdev_state *state){
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);
	const struct mira016_reg_list *reg_list;

	int ret;

	printk(KERN_INFO "[MIRA016]: Entering start streaming function.\n");

	/* Follow examples of other camera driver, here use pm_runtime_resume_and_get */
	ret = pm_runtime_resume_and_get(&client->dev);

	if (ret < 0)
	{
		printk(KERN_INFO "[MIRA016]: get_sync failed, but continue.\n");
		pm_runtime_put_noidle(&client->dev);
		return ret;
	}

	/* Set current mode according to frame format bit depth */
	ret = mira016_set_framefmt(mira016,state);
	if (ret)
	{
		dev_err(&client->dev, "%s failed to set frame format: %d\n",
				__func__, ret);
		goto err_rpm_put;
	}
	printk(KERN_INFO "[MIRA016]: Register sequence for %d bit mode will be used.\n", mira016->mode->bit_depth);
	usleep_range(100000, 150000);

	if (mira016->skip_reg_upload == 0)
	{
		/* Apply pre soft reset default values of current mode */
		reg_list = &mira016->mode->reg_list_pre_soft_reset;
		printk(KERN_INFO "[MIRA016]: Write %d regs.\n", reg_list->num_of_regs);
		ret = mira016_write_regs(mira016, reg_list->regs, reg_list->num_of_regs);
		if (ret)
		{
			dev_err(&client->dev, "%s failed to set mode\n", __func__);
			goto err_rpm_put;
		}
	}
	else
	{
		printk(KERN_INFO "[MIRA016]: Skip base register sequence upload, due to mira016->skip_reg_upload=%u.\n", mira016->skip_reg_upload);
	}

	printk(KERN_INFO "[MIRA016]: Entering v4l2 ctrl handler setup function.\n");

	/* Apply customized values from user */
	ret = __v4l2_ctrl_handler_setup(mira016->sd.ctrl_handler);
	printk(KERN_INFO "[MIRA016]: __v4l2_ctrl_handler_setup ret = %d.\n", ret);
	if (ret)
		goto err_rpm_put;

	usleep_range(8000, 10000);


	printk(KERN_INFO "[MIRA016]: Writing start streaming regs.\n");
	ret = mira016_write_start_streaming_regs(mira016);
	if (ret)
	{
		dev_err(&client->dev, "Could not write stream-on sequence");
		goto err_rpm_put;
	}

	/* vflip and hflip cannot change during streaming */
	// printk(KERN_INFO "[MIRA016]: Entering v4l2 ctrl grab vflip grab vflip.\n");
	__v4l2_ctrl_grab(mira016->vflip, true);
	// printk(KERN_INFO "[MIRA016]: Entering v4l2 ctrl grab vflip grab hflip.\n");
	__v4l2_ctrl_grab(mira016->hflip, true);

	// printk(KERN_INFO "[MIRA016]: %s Enable illumination trigger.\n", __func__);
	mira016->illum_enable = 1;
	mira016_write_illum_trig_regs(mira016);

	return 0;

err_rpm_put:
	pm_runtime_put(&client->dev);
	return ret;
}

static void mira016_stop_streaming(struct mira016 *mira016)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);
	int ret = 0;

	ret = mira016_write_stop_streaming_regs(mira016);
	if (ret) {
		dev_err(&client->dev,
			"Could not write the stream-off sequence");
	}
	__v4l2_ctrl_grab(mira016->hflip, false);
	__v4l2_ctrl_grab(mira016->vflip, false);
	pm_runtime_put(&client->dev);
}

static int mira016_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct mira016 *mira016 = to_mira016(sd);
	struct v4l2_subdev_state *state;
	int ret = 0;

	state = v4l2_subdev_lock_and_get_active_state(sd);

	if (enable)
		ret = mira016_start_streaming(mira016, state);
	else
		mira016_stop_streaming(mira016);

	v4l2_subdev_unlock_state(state);
	return ret;
}



static int mira016_get_regulators(struct mira016 *mira016)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);
	unsigned int i;

	for (i = 0; i < MIRA016_NUM_SUPPLIES; i++)
	{		
		mira016->supplies[i].supply = mira016_supply_name[i];
		printk(KERN_INFO "[MIRA016]: %s get_regulators %s.\n", __func__, mira016_supply_name[i]);
	}
	return devm_regulator_bulk_get(&client->dev,
								   MIRA016_NUM_SUPPLIES,
								   mira016->supplies);
}

/* Verify chip ID */
static int mira016_identify_module(struct mira016 *mira016)
{
	int ret;
	u8 val;

	ret = mira016_read(mira016, 0x25, &val);
	printk(KERN_INFO "[MIRA016]: Read reg 0x%4.4x, val = 0x%x.\n",
		   0x25, val);
	ret = mira016_read(mira016, 0x3, &val);
	printk(KERN_INFO "[MIRA016]: Read reg 0x%4.4x, val = 0x%x.\n",
		   0x3, val);
	ret = mira016_read(mira016, 0x4, &val);
	printk(KERN_INFO "[MIRA016]: Read reg 0x%4.4x, val = 0x%x.\n",
		   0x4, val);

	return 0;
}


static const struct v4l2_subdev_core_ops mira016_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops mira016_video_ops = {
	.s_stream = mira016_set_stream,
};

static const struct v4l2_subdev_pad_ops mira016_pad_ops = {
	.enum_mbus_code = mira016_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = mira016_set_pad_format,
	.get_selection = mira016_get_selection,
	.enum_frame_size = mira016_enum_frame_size,
};

static const struct v4l2_subdev_ops mira016_subdev_ops = {
	.core = &mira016_core_ops,
	.video = &mira016_video_ops,
	.pad = &mira016_pad_ops,
};

static const struct v4l2_subdev_internal_ops mira016_internal_ops = {
	.init_state = mira016_init_state,
};


/* Initialize control handlers */
static int mira016_init_controls(struct mira016 *mira016)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira016->sd);
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct v4l2_fwnode_device_properties props;
	int ret;


	ctrl_hdlr = &mira016->ctrl_handler;
	/* v4l2_ctrl_handler_init gives a hint/guess of the number of v4l2_ctrl_new */
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 16);
	if (ret)
		return ret;

	mutex_init(&mira016->mutex);
	ctrl_hdlr->lock = &mira016->mutex;

	printk(KERN_INFO "[MIRA016]: %s V4L2_CID_PIXEL_RATE %X.\n", __func__, V4L2_CID_PIXEL_RATE);
	printk(KERN_INFO "[MIRA016]: %s INIT_CONTROLS bitmode %X.\n", __func__, mira016->mode->bit_depth);

	/* By default, PIXEL_RATE is read only */
	mira016->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &mira016_ctrl_ops,
											V4L2_CID_PIXEL_RATE,
											MIRA016_PIXEL_RATE,
											MIRA016_PIXEL_RATE, 1,
											MIRA016_PIXEL_RATE);

	printk(KERN_INFO "[MIRA016]: %s V4L2_CID_VBLANK %X.\n", __func__, V4L2_CID_VBLANK);

	mira016->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &mira016_ctrl_ops,
										V4L2_CID_VBLANK, mira016->mode->min_vblank,
										mira016->mode->max_vblank, 1,
										MIRA016_MIN_VBLANK_60);

	printk(KERN_INFO "[MIRA016]: %s V4L2_CID_HBLANK %X.\n", __func__, V4L2_CID_HBLANK);

	mira016->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &mira016_ctrl_ops,
										V4L2_CID_HBLANK, mira016->mode->hblank,
										mira016->mode->hblank, 1,
										mira016->mode->hblank);

	// Make the vblank control read only. This could be changed to allow changing framerate in
	// runtime, but would require adapting other settings
	// mira016->vblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	// Exposure is indicated in number of lines here
	// Max is determined by vblank + vsize and Tglob.
	printk(KERN_INFO "[MIRA016]: %s V4L2_CID_EXPOSURE %X.\n", __func__, V4L2_CID_EXPOSURE);
	mira016->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &mira016_ctrl_ops,
										  V4L2_CID_EXPOSURE,
										  MIRA016_EXPOSURE_MIN_LINES, MIRA016_EXPOSURE_MAX_LINES,
										  1,
										  MIRA016_DEFAULT_EXPOSURE_LINES);
	printk(KERN_INFO "[MIRA016]: %s V4L2_CID_ANALOGUE_GAIN %X.\n", __func__, V4L2_CID_ANALOGUE_GAIN);

	mira016->gain = v4l2_ctrl_new_std(ctrl_hdlr, &mira016_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
									  mira016->mode->gain_min, mira016->mode->gain_max,
									  mira016->mode->gain_step,mira016->mode->gain_min);

	printk(KERN_INFO "[MIRA016]: %s V4L2_CID_HFLIP new %X.\n", __func__, V4L2_CID_HFLIP);

	mira016->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &mira016_ctrl_ops,
									   V4L2_CID_HFLIP, 0, 1, 1, 0);
	//if (mira016->hflip)
	//	mira016->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	printk(KERN_INFO "[MIRA016]: %s V4L2_CID_VFLIP %X.\n", __func__, V4L2_CID_VFLIP);

	mira016->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &mira016_ctrl_ops,
									   V4L2_CID_VFLIP, 0, 1, 1, 0);
	//if (mira016->vflip)
        //		mira016->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;



	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &mira016_ctrl_ops,
										  &props);
	if (ret)
		goto error;

	mira016->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&mira016->mutex);

	return ret;
}

static void mira016_free_controls(struct mira016 *mira016)
{
	v4l2_ctrl_handler_free(mira016->sd.ctrl_handler);
	mutex_destroy(&mira016->mutex);
}



static int mira016_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct mira016 *mira016;
	int ret;

	printk(KERN_INFO "[MIRA016]: probing v4l2 sensor.\n");
	printk(KERN_INFO "[MIRA016]: Driver Version 0.0.\n");

	dev_err(dev, "[MIRA016] name: %s.\n", client->name);

	mira016 = devm_kzalloc(&client->dev, sizeof(*mira016), GFP_KERNEL);
	if (!mira016)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&mira016->sd, client, &mira016_subdev_ops);


	/* Parse device tree to check if dtoverlay has param skip-reg-upload=1 */
	device_property_read_u32(dev, "skip-reg-upload", &mira016->skip_reg_upload);
	printk(KERN_INFO "[MIRA016]: skip-reg-upload %d.\n", mira016->skip_reg_upload);
	/* Set default TBD I2C device address to LED I2C Address*/
	mira016->tbd_client_i2c_addr = MIRA016LED_I2C_ADDR;
	printk(KERN_INFO "[MIRA016]: User defined I2C device address defaults to LED driver I2C address 0x%X.\n", mira016->tbd_client_i2c_addr);

	/* Get system clock (xclk) */
	mira016->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(mira016->xclk))
	{
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(mira016->xclk);
	}

	mira016->xclk_freq = clk_get_rate(mira016->xclk);
	if (mira016->xclk_freq != MIRA016_SUPPORTED_XCLK_FREQ)
	{
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
				mira016->xclk_freq);
		return -EINVAL;
	}

	ret = mira016_get_regulators(mira016);
	if (ret)
	{
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	// {
	// 	printk(KERN_INFO "[MIRA016]: Init PMIC and uC and led driver.\n");
	// 	mira016->pmic_client = i2c_new_dummy_device(client->adapter,
	// 												MIRA016PMIC_I2C_ADDR);
	// 	if (IS_ERR(mira016->pmic_client))
	// 		return PTR_ERR(mira016->pmic_client);
	// 	mira016->uc_client = i2c_new_dummy_device(client->adapter,
	// 											  MIRA016UC_I2C_ADDR);
	// 	if (IS_ERR(mira016->uc_client))
	// 		return PTR_ERR(mira016->uc_client);
	// 	mira016->led_client = i2c_new_dummy_device(client->adapter,
	// 											   MIRA016LED_I2C_ADDR);
	// 	if (IS_ERR(mira016->led_client))
	// 		return PTR_ERR(mira016->led_client);
	// }

	dev_err(dev, "[MIRA016] Sleep for 1 second to let PMIC driver complete init.\n");

	/*
	 * The sensor must be powered for mira016_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = mira016_power_on(dev);
	if (ret)
		return ret;

	fsleep(100000);
	printk(KERN_INFO "[MIRA016]: Entering identify function.\n");

	ret = mira016_identify_module(mira016);
	if (ret)
		goto error_power_off;

	printk(KERN_INFO "[MIRA016]: Setting support function.\n");


	/* Initialize default illumination trigger parameters */
	/* ILLUM_WIDTH is in unit of SEQ_TIME_BASE, equal to (8/MIRA016_DATA_RATE) us. */
	mira016->illum_width = MIRA016_ILLUM_WIDTH_DEFAULT;
	/* ILLUM_WIDTH AUTO will match illum to exposure pulse width*/
	mira016->illum_width_auto = MIRA016_ILLUM_SYNC_DEFAULT;
	/* ILLUM_ENABLE is True or False, enabling it will activate illum trig. */
	mira016->illum_enable = MIRA016_ILLUM_ENABLE_DEFAULT;
	/* ILLUM_DELAY is in unit of TIME_UNIT, equal to 1 us. In continuous stream mode, zero delay is 1<<19. */
	mira016->illum_delay = MIRA016_ILLUM_DELAY_DEFAULT;
	/* Set default mode to max resolution */
	mira016->mode = &supported_modes[0];
	/* Set default mode to max resolution */

	printk(KERN_INFO "[MIRA016]: Entering init controls function.\n");

	ret = mira016_init_controls(mira016);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	mira016->sd.internal_ops = &mira016_internal_ops;
	mira016->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			     V4L2_SUBDEV_FL_HAS_EVENTS;
	mira016->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pads */
	mira016->pad.flags = MEDIA_PAD_FL_SOURCE;


	ret = media_entity_pads_init(&mira016->sd.entity, 1, &mira016->pad);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init entity pads\n");
		goto error_handler_free;
	}

	mira016->sd.state_lock = mira016->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&mira016->sd);
	if (ret < 0) {
		dev_err_probe(dev, ret, "subdev init error\n");
		goto error_media_entity;
	}

	ret = v4l2_async_register_subdev_sensor(&mira016->sd);
	if (ret < 0) {
		dev_err_probe(dev, ret,
			      "failed to register sensor sub-device\n");
		goto error_subdev_cleanup;
	}

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;


error_subdev_cleanup:
	v4l2_subdev_cleanup(&mira016->sd);

error_media_entity:
	media_entity_cleanup(&mira016->sd.entity);

error_handler_free:
	mira016_free_controls(mira016);

error_power_off:
	mira016_power_off(dev);

	return ret;
}

static void mira016_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira016 *mira016 = to_mira016(sd);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	mira016_free_controls(mira016);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		mira016_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static const struct dev_pm_ops mira016_pm_ops = {
	SET_RUNTIME_PM_OPS(mira016_power_off, mira016_power_on, NULL)};


static const struct of_device_id mira016_dt_ids[] = {
	{ .compatible = "ams,mira016" },
	{ /* sentinel */ }};
MODULE_DEVICE_TABLE(of, mira016_dt_ids);

static struct i2c_driver mira016_i2c_driver = {
	.driver = {
		.name = "mira016",
		.of_match_table	= mira016_dt_ids,
		.pm = &mira016_pm_ops,
	},
	.probe = mira016_probe,
	.remove = mira016_remove,
};

module_i2c_driver(mira016_i2c_driver);

MODULE_AUTHOR("Philippe Baetens <philippe.baetens@ams-osram.com>");
MODULE_DESCRIPTION("ams MIRA016 sensor driver");
MODULE_LICENSE("GPL v2");

