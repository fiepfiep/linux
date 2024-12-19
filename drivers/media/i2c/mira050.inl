// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for ams MIRA050 cameras.
 * Copyright (C) 2022, ams-OSRAM
 *
 * Based on Sony IMX219 camera driver
 * Copyright (C) 2019, Raspberry Pi (Trading) Ltd
 */

#ifndef __MIRA050_INL__
#define __MIRA050_INL__

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
#include <asm/unaligned.h>

/*
 * Introduce new v4l2 control
 */
#include <linux/v4l2-controls.h>
#define AMS_CAMERA_CID_BASE (V4L2_CTRL_CLASS_CAMERA | 0x2000)
#define AMS_CAMERA_CID_MIRA_REG_W (AMS_CAMERA_CID_BASE + 0)
#define AMS_CAMERA_CID_MIRA_REG_R (AMS_CAMERA_CID_BASE + 1)

/* Most significant Byte is flag, and most significant bit is unused. */
#define AMS_CAMERA_CID_MIRA050_REG_FLAG_FOR_READ 0b00000001
#define AMS_CAMERA_CID_MIRA050_REG_FLAG_USE_BANK 0b00000010
#define AMS_CAMERA_CID_MIRA050_REG_FLAG_BANK 0b00000100
#define AMS_CAMERA_CID_MIRA050_REG_FLAG_CONTEXT 0b00001000
/* Use bit 5 to indicate special command, bit 1,2,3,4 for command. */
#define AMS_CAMERA_CID_MIRA050_REG_FLAG_CMD_SEL 0b00010000
/* Special command for sleep. The other 3 Bytes (addr+val) is sleep values in us. */
#define AMS_CAMERA_CID_MIRA050_REG_FLAG_SLEEP_US 0b00010000
/* Special command to enable power on (/off) when stream on (/off). */
#define AMS_CAMERA_CID_MIRA050_REG_FLAG_RESET_ON 0b00010010
/* Special command to disable power on (/off) when stream on (/off). */
#define AMS_CAMERA_CID_MIRA050_REG_FLAG_RESET_OFF 0b00010100
/* Special command to enable base register sequence upload, overwrite skip-reg-upload in dtoverlay */
#define AMS_CAMERA_CID_MIRA050_REG_FLAG_REG_UP_ON 0b00010110
/* Special command to disable base register sequence upload, overwrite skip-reg-upload in dtoverlay */
#define AMS_CAMERA_CID_MIRA050_REG_FLAG_REG_UP_OFF 0b00011000
/* Special command to manually power on */
#define AMS_CAMERA_CID_MIRA050_REG_FLAG_POWER_ON 0b00011010
/* Special command to manually power off */
#define AMS_CAMERA_CID_MIRA050_REG_FLAG_POWER_OFF 0b00011100
/* Special command to turn illumination trigger on */
#define AMS_CAMERA_CID_MIRA050_REG_FLAG_ILLUM_TRIG_ON 0b00011110
/* Special command to turn illumination trigger off */
#define AMS_CAMERA_CID_MIRA050_REG_FLAG_ILLUM_TRIG_OFF 0b00010001
/* Special command to set ILLUM_WIDTH. The other 3 Bytes (addr+val) is width value. */
#define AMS_CAMERA_CID_MIRA050_REG_FLAG_ILLUM_WIDTH 0b00010011
/* Special command to set ILLUM_DELAY. The other 3 Bytes (addr+val) is width value. */
#define AMS_CAMERA_CID_MIRA050_REG_FLAG_ILLUM_DELAY 0b00010101
/* Special command to enable ILLUM_WIDTH automatically tracking exposure time */
#define AMS_CAMERA_CID_MIRA050_REG_FLAG_ILLUM_EXP_T_ON 0b00010111
/* Special command to disable ILLUM_WIDTH automatically tracking exposure time */
#define AMS_CAMERA_CID_MIRA050_REG_FLAG_ILLUM_EXP_T_OFF 0b00011001
/* Special command to enable force_stream_ctrl */
#define AMS_CAMERA_CID_MIRA050_REG_FLAG_STREAM_CTRL_ON 0b00011011
/* Special command to disable force_stream_ctrl */
#define AMS_CAMERA_CID_MIRA050_REG_FLAG_STREAM_CTRL_OFF 0b00011101

/*
 * Bit 6&7 of flag are combined to specify I2C dev (default is Mira).
 * If bit 6&7 is 0b01, the reg_addr and reg_val are for a TBD I2C address.
 * The TBD I2C address is default to MIRA050LED_I2C_ADDR.
 * To change the TBD I2C address, set bit 6&7 to 0b10,
 * then the reg_val will become TBD I2C address.
 * The TBD I2C address is stored in mira050->tbd_client_i2c_addr.
 */
#define AMS_CAMERA_CID_MIRA050_REG_FLAG_I2C_SEL 0b01100000
#define AMS_CAMERA_CID_MIRA050_REG_FLAG_I2C_MIRA 0b00000000
#define AMS_CAMERA_CID_MIRA050_REG_FLAG_I2C_TBD 0b00100000
#define AMS_CAMERA_CID_MIRA050_REG_FLAG_I2C_SET_TBD 0b01000000

/* Pre-allocated i2c_client */
#define MIRA050PMIC_I2C_ADDR 0x2D
#define MIRA050UC_I2C_ADDR 0x0A
#define MIRA050LED_I2C_ADDR 0x53

#define MIRA050_NATIVE_WIDTH 576U
#define MIRA050_NATIVE_HEIGHT 768U

#define MIRA050_PIXEL_ARRAY_LEFT 0U
#define MIRA050_PIXEL_ARRAY_TOP 0U
#define MIRA050_PIXEL_ARRAY_WIDTH 576U
#define MIRA050_PIXEL_ARRAY_HEIGHT 768U

#define MIRA050_ANALOG_GAIN_MAX 2
#define MIRA050_ANALOG_GAIN_MIN 0
#define MIRA050_ANALOG_GAIN_STEP 1
#define MIRA050_ANALOG_GAIN_DEFAULT MIRA050_ANALOG_GAIN_MIN

#define MIRA050_BANK_SEL_REG 0xE000
#define MIRA050_RW_CONTEXT_REG 0xE004
#define MIRA050_CMD_REQ_1_REG 0x000A
#define MIRA050_CMD_HALT_BLOCK_REG 0x000C

// Exposure time is indicated in us
#define MIRA050_EXP_TIME_L_REG 0x000E
#define MIRA050_EXP_TIME_S_REG 0x0012

// Target frame time is indicated in us
#define MIRA050_TARGET_FRAME_TIME_REG 0x0008
#define MIRA050_GLOB_NUM_CLK_CYCLES 1928

#define MIRA050_SUPPORTED_XCLK_FREQ 24000000

// Some timings
#define MIRA050_DATA_RATE 1500 // Mbit/s
#define MIRA050_SEQ_TIME_BASE 8 / MIRA050_DATA_RATE
#define MIRA050_LUT_DEL_008 66 // for 12bit, #TODO
#define MIRA050_GRAN_TG 1500 * 50 / MIRA050_DATA_RATE
#define MIRA050_LPS_CYCLE_TIME 12600 // 12500 + 100
#define MIRA050_GLOB_TIME (int)((190 + MIRA050_LUT_DEL_008) * MIRA050_GRAN_TG * MIRA050_SEQ_TIME_BASE)
#define MIRA050_ROW_LENGTH 1842 // 12b
#define MIRA050_LPS_DISABLED 0
#define MIRA050_TROW_US MIRA050_ROW_LENGTH * 8 / MIRA050_DATA_RATE

// Default exposure is adjusted to 1 ms

#define MIRA050_MIN_ROW_LENGTH MIRA050_ROW_LENGTH // 1042 for 8 bit
#define MIRA050_MIN_ROW_LENGTH_US (MIRA050_MIN_ROW_LENGTH * 8 / MIRA050_DATA_RATE)
#define MIRA050_EXPOSURE_MIN_US (int)(1 + (151 + MIRA050_LUT_DEL_008) * MIRA050_GRAN_TG * 8 / MIRA050_DATA_RATE)
#define MIRA050_EXPOSURE_MAX_US (1000000)
#define MIRA050_EXPOSURE_MIN_LINES (MIRA050_EXPOSURE_MIN_US/MIRA050_DEFAULT_LINE_LENGTH)
#define MIRA050_EXPOSURE_MAX_LINES (MIRA050_EXPOSURE_MAX_US/MIRA050_DEFAULT_LINE_LENGTH)

#define MIRA050_DEFAULT_EXPOSURE_LINES 1000
#define MIRA050_DEFAULT_EXPOSURE_US MIRA050_DEFAULT_EXPOSURE_LINES*MIRA050_DEFAULT_LINE_LENGTH
// Default exposure for V4L2 is in row time

// #define MIRA050_MIN_VBLANK 11 // for 10b or 8b, 360fps
// 50 fps
#define MIRA050_MIN_VBLANK_60 900

// 50 fps
#define MIRA050_MIN_VBLANK_120 65

// 120 fps

#define MIRA050_MAX_VBLANK 100000

// Default exposure is adjusted to 1 ms
// #define MIRA050_LUT_DEL_008			66
// #define MIRA050_GRAN_TG				34
// #define MIRA050_DATA_RATE			1000 // Mbit/s
// #define MIRA050_MIN_ROW_LENGTH			1842
// #define MIRA050_MIN_ROW_LENGTH_US		(MIRA050_MIN_ROW_LENGTH * 8 / MIRA050_DATA_RATE)
// #define MIRA050_EXPOSURE_MIN_US			(int)(1 + (151 + MIRA050_LUT_DEL_008) * MIRA050_GRAN_TG * 8 / MIRA050_DATA_RATE)
// #define MIRA050_EXPOSURE_MIN_RT			(int)(1 + (151 + MIRA050_LUT_DEL_008) * MIRA050_GRAN_TG / MIRA050_MIN_ROW_LENGTH)
// #define MIRA050_EXPOSURE_MAX_US			(16601)
// #define MIRA050_EXPOSURE_MAX_RT			(int)(1 + MIRA050_EXPOSURE_MAX_US / MIRA050_MIN_ROW_LENGTH_US)
// #define MIRA050_DEFAULT_EXPOSURE_US		1000
// #define MIRA050_DEFAULT_EXPOSURE_RT		(int)(1 + MIRA050_DEFAULT_EXPOSURE_US / MIRA050_MIN_ROW_LENGTH_US)

// Power on function timing
#define MIRA050_XCLR_MIN_DELAY_US 150000
#define MIRA050_XCLR_DELAY_RANGE_US 3000

// set pixel rate equal to width. such that 1 row time is 1 us.
// pixel_rate = link_freq * 2 * nr_of_lanes / bits_per_sample
// 1.0Gb/s * 2 * 1 / 12 = 178956970
#define MIRA050_PIXEL_RATE (57600000) //reduce by factor 10, otherwise driver complains...
/* Should match device tree link freq */
#define MIRA050_DEFAULT_LINK_FREQ 456000000
#define MIRA050_DEFAULT_LINE_LENGTH (10) // pixel rate / (HSIZE+HBLANK)

/* Trick the libcamera with achievable fps via hblank */

/* Formular in libcamera to derive TARGET_FPS:
 * TARGET_FPS=1/((1/MIRA050_PIXEL_RATE)*(WIDTH+HBLANK)*(HEIGHT+MIRA050_MIN_VBLANK))
 * Example with HBLANK=0 and MIRA050_MIN_VBLANK=12
 * TARGET_FPS=1/((1/178956970)*576*(768+12))=398
 *
 * Inverse the above formula to derive HBLANK from TARGET_FPS:
 * HBLANK=1/((1/MIRA050_PIXEL_RATE)*TARGET_FPS*(HEIGHT+MIRA050_MIN_VBLANK))-WIDTH
 * Example with TARGET_FPS of 50 fps
 * HBLANK=1/((1/178956970)*50*(768+12))-576=4013
 */

// For test pattern with fixed data
#define MIRA050_TRAINING_WORD_REG 0x0060
// For test pattern with 2D gradiant
#define MIRA050_DELTA_TEST_IMG_REG 0x0056
// For setting test pattern type
#define MIRA050_TEST_PATTERN_REG 0x0062
#define MIRA050_TEST_PATTERN_DISABLE 0x00
#define MIRA050_TEST_PATTERN_FIXED_DATA 0x01
#define MIRA050_TEST_PATTERN_2D_GRADIENT 0x02

/* Embedded metadata stream structure */
#define MIRA050_EMBEDDED_LINE_WIDTH 16384
#define MIRA050_NUM_EMBEDDED_LINES 1


#define MIRA050_GDIG_PREAMP 0x0024
#define MIRA050_BIAS_RG_ADCGAIN 0x01F0
#define MIRA050_BIAS_RG_MULT 0x01F3
#define MIRA050_OFFSET_CLIPPING 0x0193

#define MIRA050_OTP_COMMAND 0x0066
#define MIRA050_OTP_ADDR 0x0067
#define MIRA050_OTP_START 0x0064
#define MIRA050_OTP_BUSY 0x0065
#define MIRA050_OTP_DOUT 0x006C
#define MIRA050_OTP_CAL_VALUE_DEFAULT 2250
#define MIRA050_OTP_CAL_FINE_VALUE_DEFAULT 35
#define MIRA050_OTP_CAL_FINE_VALUE_MIN 1
#define MIRA050_OTP_CAL_FINE_VALUE_MAX 60 // TODO

/* Illumination trigger */
#define MIRA050_EN_TRIG_SYNC 0x001D		  // bank 1
#define MIRA050_TRIG_SYNC_DELAY 0x001A	  // bank 0
#define MIRA050_DMUX0_SEL 0x00F3		  // bank 0
#define MIRA050_TRIG_SYNC_ON_REQ_1 0x001D // bank 0

#define MIRA050_EN_TRIG_ILLUM 0x001C
#define MIRA050_ILLUM_WIDTH_REG 0x0019
#define MIRA050_ILLUM_DELAY_REG 0x0016
#define MIRA050_ILLUM_WIDTH_DEFAULT (MIRA050_DEFAULT_EXPOSURE_US * MIRA050_DATA_RATE / 8)
#define MIRA050_ILLUM_DELAY_DEFAULT (1 << 19)
#define MIRA050_ILLUM_WIDTH_AUTO_DEFAULT 1;
#define MIRA050_ILLUM_ENABLE_DEFAULT 1;
enum pad_types
{
	IMAGE_PAD,
	METADATA_PAD,
	NUM_PADS
};

struct mira050_reg
{
	u16 address;
	u8 val;
};

// struct mira050_fine_gain_lut
// {
// 	u8 gdig_preamp;
// 	u8 rg_adcgain;
// 	u8 rg_mult;
// };
struct mira050_fine_gain_lut_new
{
	u32 analog_gain;
	u8 gdig_preamp;
	u8 rg_adcgain;
	u8 rg_mult;
};
struct mira050_reg_list
{
	unsigned int num_of_regs;
	const struct mira050_reg *regs;
};

struct mira050_v4l2_reg
{
	u32 val;
};

/* Mode : resolution and related config&values */
struct mira050_mode
{
	/* Frame width */
	unsigned int width;
	/* Frame height */
	unsigned int height;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* Default register values */
	struct mira050_reg_list reg_list_pre_soft_reset;
	struct mira050_reg_list reg_list_post_soft_reset;
	u32 gain_min;
	u32 gain_max;
	u32 min_vblank;
	u32 max_vblank;
	u32 hblank;
	u32 row_length;

	/* Format code */
	u32 code;

	/* bit_depth needed for analog gain selection */
	u8 bit_depth;
};

// 576_768_50fps_12b_1lane
// Taken from generated_api\CSP_orig_12-bit mode_anagain1_60fps_exp2000us_datarate_1500.0_mclk_24.0.txt
static const struct mira050_reg full_576_768_50fps_12b_1lane_reg_pre_soft_reset[] = {
{0xE000,0x0},//,Base Configuration.BANK_SEL
{0x01E4,0x0},//,Base Configuration
{0x01E5,0x13},//,Base Configuration
{0x01E2,0x17},//,Base Configuration
{0x01E3,0x88},//,Base Configuration
{0x01E6,0x0},//,Base Configuration
{0x01E7,0xCA},//,Base Configuration
{0x016C,0x1},//,Base Configuration
{0x016B,0x1},//,Base Configuration
{0x0208,0x1},//,Base Configuration
{0x0209,0xF0},//,Base Configuration
{0x020A,0x3},//,Base Configuration
{0x020B,0x4D},//,Base Configuration
{0x020C,0x2},//,Base Configuration
{0x020D,0x10},//,Base Configuration
{0x020E,0x3},//,Base Configuration
{0x020F,0x1},//,Base Configuration
{0x0210,0x0},//,Base Configuration
{0x0211,0x13},//,Base Configuration
{0x0212,0x0},//,Base Configuration
{0x0213,0x3},//,Base Configuration
{0x0214,0x3},//,Base Configuration
{0x0215,0xEF},//,Base Configuration
{0x0216,0x3},//,Base Configuration
{0x0217,0xF3},//,Base Configuration
{0x0218,0x3},//,Base Configuration
{0x0219,0xF4},//,Base Configuration
{0x021A,0x1},//,Base Configuration
{0x021B,0xF1},//,Base Configuration
{0x021C,0x3},//,Base Configuration
{0x021D,0x24},//,Base Configuration
{0x021E,0x0},//,Base Configuration
{0x021F,0x2},//,Base Configuration
{0x0220,0x1},//,Base Configuration
{0x0221,0xF2},//,Base Configuration
{0x0222,0x3},//,Base Configuration
{0x0223,0x2F},//,Base Configuration
{0x0224,0x0},//,Base Configuration
{0x0225,0x21},//,Base Configuration
{0x0226,0x3},//,Base Configuration
{0x0227,0xF0},//,Base Configuration
{0x0228,0x3},//,Base Configuration
{0x0229,0xF1},//,Base Configuration
{0x022A,0x3},//,Base Configuration
{0x022B,0xF2},//,Base Configuration
{0x022C,0x3},//,Base Configuration
{0x022D,0xF5},//,Base Configuration
{0x022E,0x3},//,Base Configuration
{0x022F,0xF6},//,Base Configuration
{0x0230,0x0},//,Base Configuration
{0x0231,0xC1},//,Base Configuration
{0x0232,0x0},//,Base Configuration
{0x0233,0x2},//,Base Configuration
{0x0234,0x1},//,Base Configuration
{0x0235,0xF2},//,Base Configuration
{0x0236,0x3},//,Base Configuration
{0x0237,0x6B},//,Base Configuration
{0x0238,0x3},//,Base Configuration
{0x0239,0xFF},//,Base Configuration
{0x023A,0x3},//,Base Configuration
{0x023B,0x31},//,Base Configuration
{0x023C,0x1},//,Base Configuration
{0x023D,0xF0},//,Base Configuration
{0x023E,0x3},//,Base Configuration
{0x023F,0x87},//,Base Configuration
{0x0240,0x2},//,Base Configuration
{0x0241,0x3A},//,Base Configuration
{0x0242,0x0},//,Base Configuration
{0x0243,0xB},//,Base Configuration
{0x0244,0x1},//,Base Configuration
{0x0245,0xF9},//,Base Configuration
{0x0246,0x3},//,Base Configuration
{0x0247,0xD},//,Base Configuration
{0x0248,0x0},//,Base Configuration
{0x0249,0x7},//,Base Configuration
{0x024A,0x3},//,Base Configuration
{0x024B,0xEF},//,Base Configuration
{0x024C,0x3},//,Base Configuration
{0x024D,0xF3},//,Base Configuration
{0x024E,0x3},//,Base Configuration
{0x024F,0xF4},//,Base Configuration
{0x0250,0x3},//,Base Configuration
{0x0251,0x0},//,Base Configuration
{0x0252,0x0},//,Base Configuration
{0x0253,0x7},//,Base Configuration
{0x0254,0x0},//,Base Configuration
{0x0255,0xC},//,Base Configuration
{0x0256,0x1},//,Base Configuration
{0x0257,0xF1},//,Base Configuration
{0x0258,0x3},//,Base Configuration
{0x0259,0x43},//,Base Configuration
{0x025A,0x1},//,Base Configuration
{0x025B,0xF8},//,Base Configuration
{0x025C,0x3},//,Base Configuration
{0x025D,0x10},//,Base Configuration
{0x025E,0x0},//,Base Configuration
{0x025F,0x7},//,Base Configuration
{0x0260,0x3},//,Base Configuration
{0x0261,0xF0},//,Base Configuration
{0x0262,0x3},//,Base Configuration
{0x0263,0xF1},//,Base Configuration
{0x0264,0x3},//,Base Configuration
{0x0265,0xF2},//,Base Configuration
{0x0266,0x3},//,Base Configuration
{0x0267,0xF5},//,Base Configuration
{0x0268,0x3},//,Base Configuration
{0x0269,0xF6},//,Base Configuration
{0x026A,0x3},//,Base Configuration
{0x026B,0x0},//,Base Configuration
{0x026C,0x2},//,Base Configuration
{0x026D,0x87},//,Base Configuration
{0x026E,0x2},//,Base Configuration
{0x026F,0x31},//,Base Configuration
{0x0270,0x3},//,Base Configuration
{0x0271,0xFF},//,Base Configuration
{0x0272,0x3},//,Base Configuration
{0x0273,0x0},//,Base Configuration
{0x0274,0x3},//,Base Configuration
{0x0275,0xFF},//,Base Configuration
{0x0276,0x2},//,Base Configuration
{0x0277,0x87},//,Base Configuration
{0x0278,0x3},//,Base Configuration
{0x0279,0x2},//,Base Configuration
{0x027A,0x3},//,Base Configuration
{0x027B,0x9},//,Base Configuration
{0x027C,0x3},//,Base Configuration
{0x027D,0xF7},//,Base Configuration
{0x027E,0x0},//,Base Configuration
{0x027F,0x16},//,Base Configuration
{0x0280,0x0},//,Base Configuration
{0x0281,0x33},//,Base Configuration
{0x0282,0x0},//,Base Configuration
{0x0283,0x4},//,Base Configuration
{0x0284,0x0},//,Base Configuration
{0x0285,0x11},//,Base Configuration
{0x0286,0x3},//,Base Configuration
{0x0287,0x9},//,Base Configuration
{0x0288,0x0},//,Base Configuration
{0x0289,0x2},//,Base Configuration
{0x028A,0x0},//,Base Configuration
{0x028B,0x20},//,Base Configuration
{0x028C,0x0},//,Base Configuration
{0x028D,0xB5},//,Base Configuration
{0x028E,0x1},//,Base Configuration
{0x028F,0x5},//,Base Configuration
{0x0290,0x0},//,Base Configuration
{0x0291,0x12},//,Base Configuration
{0x0292,0x0},//,Base Configuration
{0x0293,0xB5},//,Base Configuration
{0x0294,0x1},//,Base Configuration
{0x0295,0x5},//,Base Configuration
{0x0296,0x0},//,Base Configuration
{0x0297,0x0},//,Base Configuration
{0x0298,0x0},//,Base Configuration
{0x0299,0x12},//,Base Configuration
{0x029A,0x0},//,Base Configuration
{0x029B,0x12},//,Base Configuration
{0x029C,0x0},//,Base Configuration
{0x029D,0x20},//,Base Configuration
{0x029E,0x0},//,Base Configuration
{0x029F,0xB5},//,Base Configuration
{0x02A0,0x1},//,Base Configuration
{0x02A1,0x5},//,Base Configuration
{0x02A2,0x0},//,Base Configuration
{0x02A3,0x0},//,Base Configuration
{0x02A4,0x0},//,Base Configuration
{0x02A5,0x12},//,Base Configuration
{0x02A6,0x0},//,Base Configuration
{0x02A7,0x12},//,Base Configuration
{0x02A8,0x0},//,Base Configuration
{0x02A9,0x20},//,Base Configuration
{0x02AA,0x0},//,Base Configuration
{0x02AB,0x47},//,Base Configuration
{0x02AC,0x0},//,Base Configuration
{0x02AD,0x27},//,Base Configuration
{0x02AE,0x0},//,Base Configuration
{0x02AF,0xB5},//,Base Configuration
{0x02B0,0x0},//,Base Configuration
{0x02B1,0xE5},//,Base Configuration
{0x02B2,0x0},//,Base Configuration
{0x02B3,0x0},//,Base Configuration
{0x02B4,0x0},//,Base Configuration
{0x02B5,0x4},//,Base Configuration
{0x02B6,0x0},//,Base Configuration
{0x02B7,0x43},//,Base Configuration
{0x02B8,0x0},//,Base Configuration
{0x02B9,0x1},//,Base Configuration
{0x02BA,0x3},//,Base Configuration
{0x02BB,0x2},//,Base Configuration
{0x02BC,0x0},//,Base Configuration
{0x02BD,0x8},//,Base Configuration
{0x02BE,0x3},//,Base Configuration
{0x02BF,0xFF},//,Base Configuration
{0x02C0,0x2},//,Base Configuration
{0x02C1,0x87},//,Base Configuration
{0x02C2,0x3},//,Base Configuration
{0x02C3,0x89},//,Base Configuration
{0x02C4,0x3},//,Base Configuration
{0x02C5,0xF7},//,Base Configuration
{0x02C6,0x0},//,Base Configuration
{0x02C7,0x77},//,Base Configuration
{0x02C8,0x0},//,Base Configuration
{0x02C9,0x17},//,Base Configuration
{0x02CA,0x0},//,Base Configuration
{0x02CB,0x8},//,Base Configuration
{0x02CC,0x3},//,Base Configuration
{0x02CD,0xFF},//,Base Configuration
{0x02CE,0x0},//,Base Configuration
{0x02CF,0x38},//,Base Configuration
{0x02D0,0x0},//,Base Configuration
{0x02D1,0x17},//,Base Configuration
{0x02D2,0x0},//,Base Configuration
{0x02D3,0x8},//,Base Configuration
{0x02D4,0x3},//,Base Configuration
{0x02D5,0xFF},//,Base Configuration
{0x02D6,0x3},//,Base Configuration
{0x02D7,0xFF},//,Base Configuration
{0x02D8,0x3},//,Base Configuration
{0x02D9,0xFF},//,Base Configuration
{0x02DA,0x3},//,Base Configuration
{0x02DB,0xFF},//,Base Configuration
{0x02DC,0x3},//,Base Configuration
{0x02DD,0xFF},//,Base Configuration
{0x02DE,0x3},//,Base Configuration
{0x02DF,0xFF},//,Base Configuration
{0x02E0,0x3},//,Base Configuration
{0x02E1,0xFF},//,Base Configuration
{0x02E2,0x3},//,Base Configuration
{0x02E3,0xFF},//,Base Configuration
{0x02E4,0x3},//,Base Configuration
{0x02E5,0xFF},//,Base Configuration
{0x02E6,0x3},//,Base Configuration
{0x02E7,0xFF},//,Base Configuration
{0x02E8,0x3},//,Base Configuration
{0x02E9,0xFF},//,Base Configuration
{0x02EA,0x3},//,Base Configuration
{0x02EB,0xFF},//,Base Configuration
{0x02EC,0x3},//,Base Configuration
{0x02ED,0xFF},//,Base Configuration
{0x02EE,0x3},//,Base Configuration
{0x02EF,0xFF},//,Base Configuration
{0x02F0,0x3},//,Base Configuration
{0x02F1,0xFF},//,Base Configuration
{0x02F2,0x3},//,Base Configuration
{0x02F3,0xFF},//,Base Configuration
{0x02F4,0x3},//,Base Configuration
{0x02F5,0xFF},//,Base Configuration
{0x02F6,0x3},//,Base Configuration
{0x02F7,0xFF},//,Base Configuration
{0x02F8,0x3},//,Base Configuration
{0x02F9,0xFF},//,Base Configuration
{0x02FA,0x3},//,Base Configuration
{0x02FB,0xFF},//,Base Configuration
{0x02FC,0x3},//,Base Configuration
{0x02FD,0xFF},//,Base Configuration
{0x02FE,0x3},//,Base Configuration
{0x02FF,0xFF},//,Base Configuration
{0x0300,0x3},//,Base Configuration
{0x0301,0xFF},//,Base Configuration
{0x0302,0x3},//,Base Configuration
{0x0303,0xFF},//,Base Configuration
{0x01E9,0x0},//,Base Configuration
{0x01E8,0x19},//,Base Configuration
{0x01EA,0x35},//,Base Configuration
{0x01EB,0x37},//,Base Configuration
{0x01EC,0x5C},//,Base Configuration
{0x01ED,0x63},//,Base Configuration
{0x01F8,0xF},//,Base Configuration
{0x005C,0x0},//,Base Configuration
{0x005D,0x0},//,Base Configuration
{0x01DA,0x1},//,Base Configuration
{0x01DC,0x1},//,Base Configuration
{0x01DE,0x1},//,Base Configuration
{0x0189,0x1},//,Base Configuration
{0x01B7,0x1},//,Base Configuration
{0x01C1,0xE},//,Base Configuration
{0x01C2,0xF6},//,Base Configuration
{0x01C3,0xFF},//,Base Configuration
{0x01B8,0x1},//,Base Configuration
{0x01BA,0x32},//,Base Configuration
{0x01BD,0x8},//,Base Configuration
{0x01CA,0x1E},//,Base Configuration
{0x01C9,0x1E},//,Base Configuration
{0x01BF,0x3C},//,Base Configuration
{0x01C0,0x5C},//,Base Configuration
{0x0071,0x1},//,Base Configuration
{0x01B4,0x1},//,Base Configuration
{0x01B5,0x1},//,Base Configuration
{0x01F1,0x1},//,Base Configuration
{0x01F4,0x1},//,Base Configuration
{0x01F5,0x1},//,Base Configuration
{0x0314,0x1},//,Base Configuration
{0x0315,0x1},//,Base Configuration
{0x0316,0x1},//,Base Configuration
{0x0207,0x0},//,Base Configuration
{0x4207,0x2},//,Base Configuration
{0x2207,0x2},//,Base Configuration
{0x209D,0x0},//,Base Configuration
{0x0063,0x1},//,Base Configuration
{0x01F7,0xF},//,Base Configuration
{0x00E9,0x3},//,Base Configuration
{0x00EA,0x28},//,Base Configuration
{0x0309,0x7},//,Base Configuration
{0x030A,0x4},//,Base Configuration
{0x030B,0xD},//,Base Configuration
{0x030C,0x7},//,Base Configuration
{0x030E,0x15},//,Base Configuration
{0x030D,0xA},//,Base Configuration
{0x030F,0x1},//,Base Configuration
{0x0310,0xF},//,Base Configuration
{0x01D0,0x1F},//,Base Configuration
{0x01D1,0x12},//,Base Configuration
{0x0016,0x0},//,Base Configuration
{0x0017,0x5},//,Base Configuration
{0x00E8,0x3},//,Base Configuration
{0xE0C0,0x0},//,Base Configuration
{0xE0C1,0x20},//,Base Configuration
{0xE0C2,0x0},//,Base Configuration
{0xE0C3,0x20},//,Base Configuration
{0x016A,0x2},//,Base Configuration.DPATH_BITMODE
{0x0168,0x2C},//,Base Configuration.CSI2_DATA_TYPE
{0xE000,0x0},//,PLL.BANK_SEL
{0x2077,0x0},//,PLL.PLL_DIV_N
{0x2076,0xBD},//,PLL.PLL_DIV_M
{0x00CE,0x1},//,PLL.CLKGEN_CP_DIV
{0x0070,0x9},//,PLL.OTP_GRANULARITY
{0x016D,0x32},//,PLL.GRAN_TG
{0x0176,0x0},//,PLL.LUT_DEL_008
{0x20C6,0x0},//,PLL.CLKGEN_TX_ESC_H
{0x20C7,0x0},//,PLL.CLKGEN_TX_ESC_H
{0x20C8,0x1},//,PLL.CLKGEN_TX_ESC_H
{0x20C9,0x0},//,PLL.CLKGEN_TX_ESC_L
{0x20CA,0x0},//,PLL.CLKGEN_TX_ESC_L
{0x20CB,0x1},//,PLL.CLKGEN_TX_ESC_L
{0x2075,0x0},//,PLL.PLL_PD
{0xE000,0x0},//,Low Power State.BANK_SEL
{0x001E,0x1},//,Low Power State.EN_AUTO_LPS_TRANS
{0xE000,0x0},//,MIPI.BANK_SEL
{0x207E,0x0},//,MIPI.MIPI_VC_ID
{0x207F,0x0},//,MIPI.MIPI_FRAME_MODE_PIX
{0x2080,0x0},//,MIPI.MIPI_FRAME_MODE_EMBED
{0x2081,0x3},//,MIPI.MIPI_LINE_COUNT_EN
{0x2082,0x0},//,MIPI.MIPI_FRAME_COUNT_WRAP
{0x2083,0x2},//,MIPI.MIPI_FRAME_COUNT_WRAP
{0x0090,0x0},//,MIPI.CSI2_CONT_CLOCK_MODE
{0x2097,0x0},//,MIPI.CSI2_SCRAMBLING_EN
{0xE000,0x0},//,Sensor Control Mode.BANK_SEL
{0x0011,0x3},//,Sensor Control Mode.CTRL_MODE
{0x011D,0x0},//,Sensor Control Mode.SINGLE_PIN_CTRL
{0xE000,0x0},//,Time Bases.BANK_SEL
{0x0012,0x0},//,Time Bases.CLOCKS_PER_TIME_UNIT
{0x0013,0x18},//,Time Bases.CLOCKS_PER_TIME_UNIT
{0x015A,0x0},//,Time Bases.GLOB_TIME
{0x015B,0x33},//,Time Bases.GLOB_TIME
{0x015C,0x0},//,Time Bases.GLOB_TIME_A
{0x015D,0x33},//,Time Bases.GLOB_TIME_A
{0x015E,0x0},//,Time Bases.GLOB_TIME_B
{0x015F,0x33},//,Time Bases.GLOB_TIME_B
{0x0162,0x0},//,Time Bases.ENTER_LPS_TIME
{0x0163,0x5},//,Time Bases.ENTER_LPS_TIME
{0x0164,0x4},//,Time Bases.EXIT_LPS_TIME
{0x0165,0x4C},//,Time Bases.EXIT_LPS_TIME
{0x0166,0x4},//,Time Bases.LPS_CYCLE_TIME
{0x0167,0x4C},//,Time Bases.LPS_CYCLE_TIME
{0xE000,0x0},//,Analog Gain.BANK_SEL
{0x01BB,0x99},//,Analog Gain
{0x01BC,0x91},//,Analog Gain
{0x00D0,0x0},//,Analog Gain
{0x01F0,0x8},//,Analog Gain
{0x01F3,0x0},//,Analog Gain
{0x016E,0xFF},//,Analog Gain.LUT_DEL_000
{0x0172,0xFF},//,Analog Gain.LUT_DEL_004
{0x0173,0x2E},//,Analog Gain.LUT_DEL_005
{0x016F,0xFF},//,Analog Gain.LUT_DEL_001
{0x0170,0xFF},//,Analog Gain.LUT_DEL_002
{0x0171,0xFF},//,Analog Gain.LUT_DEL_003
{0x0174,0xFF},//,Analog Gain.LUT_DEL_006
{0x0175,0xAB},//,Analog Gain.LUT_DEL_007
{0x018B,0x8},//,Analog Gain
{0x018C,0xCA},//,Analog Gain
{0x018D,0x2},//,Analog Gain
{0x018E,0x56},//,Analog Gain
{0x018F,0x12},//,Analog Gain
{0x0190,0xBE},//,Analog Gain
{0x01EE,0x14},//,Analog Gain.SHUTTER_LAG
{0x01EF,0xA2},//,Analog Gain.SHUTTER_LAG
{0x01A2,0x6},//,Analog Gain.POS_ANACOL_TRIGGER
{0x01A3,0xA5},//,Analog Gain.POS_ANACOL_TRIGGER
{0x031F,0x6},//,Analog Gain.POS_YADDR_TRIGGER
{0x0320,0xAE},//,Analog Gain.POS_YADDR_TRIGGER
{0x01A6,0x7},//,Analog Gain.POS_ADC_TRIGGER
{0x01A7,0x3C},//,Analog Gain.POS_ADC_TRIGGER
{0x01A4,0xF},//,Analog Gain.POS_ANACOL_YBIN_TRIGGER
{0x01A5,0x27},//,Analog Gain.POS_ANACOL_YBIN_TRIGGER
{0x0321,0xF},//,Analog Gain.POS_YADDR_YBIN_TRIGGER
{0x0322,0x30},//,Analog Gain.POS_YADDR_YBIN_TRIGGER
{0x01A8,0xF},//,Analog Gain.POS_ADC_YBIN_TRIGGER
{0x01A9,0xBE},//,Analog Gain.POS_ADC_YBIN_TRIGGER
{0x01A0,0x1},//,Analog Gain.POS_VIS_TRIGGER
{0x01A1,0x25},//,Analog Gain.POS_VIS_TRIGGER
{0x01B2,0x1},//,Analog Gain.POS_HSYNC_RISE
{0x01B3,0x3D},//,Analog Gain.POS_HSYNC_RISE
{0x01B0,0x1},//,Analog Gain.POS_STAT_TRIGGER
{0x01B1,0x38},//,Analog Gain.POS_STAT_TRIGGER
{0x01AC,0x1},//,Analog Gain.POS_SVAL_TRIGGER
{0x01AD,0x43},//,Analog Gain.POS_SVAL_TRIGGER
{0xE000,0x0},//,Black Level.BANK_SEL
{0x0193,0x38},//,Black Level.OFFSET_CLIPPING
{0x0194,0xA6},//,Black Level.OFFSET_CLIPPING
{0xE000,0x0},//,Release Soft Reset.BANK_SEL
{0xE009,0x1},//,Release Soft Reset
{0x212F,0x1},//,Release Soft Reset
{0x2130,0x1},//,Release Soft Reset
{0x2131,0x1},//,Release Soft Reset
{0x2132,0x1},//,Release Soft Reset
{0x2133,0x1},//,Release Soft Reset
{0x2134,0x1},//,Release Soft Reset
{0x2135,0x1},//,Release Soft Reset
{0xE0E1,0x1},//,Release Soft Reset
{0x018A,0x1},//,Release Soft Reset
{0x00E0,0x1},//,Release Soft Reset
{0xE004,0x0},//,Horizontal ROI.RW_CONTEXT
{0xE000,0x1},//,Horizontal ROI.BANK_SEL
{0xE02C,0x0},//,Horizontal ROI.XWIN_LEFT
{0xE02D,0xC},//,Horizontal ROI.XWIN_LEFT
{0xE02E,0x2},//,Horizontal ROI.XWIN_RIGHT
{0xE02F,0x4B},//,Horizontal ROI.XWIN_RIGHT
{0xE030,0x0},//,Horizontal ROI.XMIRROR
{0xE025,0x0},//,Horizontal ROI.XSUBSAMPLING
{0xE02A,0x0},//,Horizontal ROI.XBINNING
{0x2029,0x46},//,Horizontal ROI.MIPI_DATA_FIFO_THRESHOLD
{0x0034,0x1},//,Horizontal ROI.HSYNC_LENGTH
{0x0035,0x20},//,Horizontal ROI.HSYNC_LENGTH
{0xE004,0x1},//,Horizontal ROI.RW_CONTEXT
{0xE02C,0x0},//,Horizontal ROI.XWIN_LEFT
{0xE02D,0xC},//,Horizontal ROI.XWIN_LEFT
{0xE02E,0x2},//,Horizontal ROI.XWIN_RIGHT
{0xE02F,0x4B},//,Horizontal ROI.XWIN_RIGHT
{0xE030,0x0},//,Horizontal ROI.XMIRROR
{0xE025,0x0},//,Horizontal ROI.XSUBSAMPLING
{0xE02A,0x0},//,Horizontal ROI.XBINNING
{0x2029,0x46},//,Horizontal ROI.MIPI_DATA_FIFO_THRESHOLD
{0x0034,0x1},//,Horizontal ROI.HSYNC_LENGTH
{0x0035,0x20},//,Horizontal ROI.HSYNC_LENGTH
{0xE004,0x0},//,Vertical ROI.RW_CONTEXT
{0xE000,0x1},//,Vertical ROI.BANK_SEL
{0x001E,0x0},//,Vertical ROI.YWIN_ENA
{0x001F,0x1},//,Vertical ROI.YWIN_ENA
{0x002B,0x0},//,Vertical ROI.YBINNING
{0xE004,0x1},//,Vertical ROI.RW_CONTEXT
{0x001E,0x0},//,Vertical ROI.YWIN_ENA
{0x001F,0x1},//,Vertical ROI.YWIN_ENA
{0x002B,0x0},//,Vertical ROI.YBINNING
{0xE000,0x0},//,Vertical ROI.BANK_SEL
{0x001F,0x0},//,Vertical ROI.YWIN_BLACK
{0x0020,0x0},//,Vertical ROI.YWIN_BLACK
{0x0023,0x0},//,Vertical ROI.YWIN_DIR
{0x0024,0x3},//,Vertical ROI.YWIN0_SIZE
{0x0025,0x0},//,Vertical ROI.YWIN0_SIZE
{0x0026,0x0},//,Vertical ROI.YWIN0_START
{0x0027,0x18},//,Vertical ROI.YWIN0_START
{0x0028,0x0},//,Vertical ROI.YWIN0_SUBS
{0x0029,0x0},//,Vertical ROI.YWIN1_SIZE
{0x002A,0x0},//,Vertical ROI.YWIN1_SIZE
{0x002B,0x0},//,Vertical ROI.YWIN1_START
{0x002C,0x0},//,Vertical ROI.YWIN1_START
{0x002D,0x0},//,Vertical ROI.YWIN1_SUBS
{0x002E,0x0},//,Vertical ROI.YWIN2_SIZE
{0x002F,0x0},//,Vertical ROI.YWIN2_SIZE
{0x0030,0x0},//,Vertical ROI.YWIN2_START
{0x0031,0x0},//,Vertical ROI.YWIN2_START
{0x0032,0x0},//,Vertical ROI.YWIN2_SUBS
{0x0033,0x0},//,Vertical ROI.YWIN3_SIZE
{0x0034,0x0},//,Vertical ROI.YWIN3_SIZE
{0x0035,0x0},//,Vertical ROI.YWIN3_START
{0x0036,0x0},//,Vertical ROI.YWIN3_START
{0x0037,0x0},//,Vertical ROI.YWIN3_SUBS
{0x0038,0x0},//,Vertical ROI.YWIN4_SIZE
{0x0039,0x0},//,Vertical ROI.YWIN4_SIZE
{0x003A,0x0},//,Vertical ROI.YWIN4_START
{0x003B,0x0},//,Vertical ROI.YWIN4_START
{0x003C,0x0},//,Vertical ROI.YWIN4_SUBS
{0x003D,0x0},//,Vertical ROI.YWIN5_SIZE
{0x003E,0x0},//,Vertical ROI.YWIN5_SIZE
{0x003F,0x0},//,Vertical ROI.YWIN5_START
{0x0040,0x0},//,Vertical ROI.YWIN5_START
{0x0041,0x0},//,Vertical ROI.YWIN5_SUBS
{0x0042,0x0},//,Vertical ROI.YWIN6_SIZE
{0x0043,0x0},//,Vertical ROI.YWIN6_SIZE
{0x0044,0x0},//,Vertical ROI.YWIN6_START
{0x0045,0x0},//,Vertical ROI.YWIN6_START
{0x0046,0x0},//,Vertical ROI.YWIN6_SUBS
{0x0047,0x0},//,Vertical ROI.YWIN7_SIZE
{0x0048,0x0},//,Vertical ROI.YWIN7_SIZE
{0x0049,0x0},//,Vertical ROI.YWIN7_START
{0x004A,0x0},//,Vertical ROI.YWIN7_START
{0x004B,0x0},//,Vertical ROI.YWIN7_SUBS
{0x004C,0x0},//,Vertical ROI.YWIN8_SIZE
{0x004D,0x0},//,Vertical ROI.YWIN8_SIZE
{0x004E,0x0},//,Vertical ROI.YWIN8_START
{0x004F,0x0},//,Vertical ROI.YWIN8_START
{0x0050,0x0},//,Vertical ROI.YWIN8_SUBS
{0x0051,0x0},//,Vertical ROI.YWIN9_SIZE
{0x0052,0x0},//,Vertical ROI.YWIN9_SIZE
{0x0053,0x0},//,Vertical ROI.YWIN9_START
{0x0054,0x0},//,Vertical ROI.YWIN9_START
{0x0055,0x0},//,Vertical ROI.YWIN9_SUBS
{0xE004,0x0},//,Frame && Exposure Control.RW_CONTEXT
{0xE000,0x1},//,Frame && Exposure Control.BANK_SEL
{0x000E,0x0},//,Frame && Exposure Control.EXP_TIME_L
{0x000F,0x0},//,Frame && Exposure Control.EXP_TIME_L
{0x0010,0x3},//,Frame && Exposure Control.EXP_TIME_L
{0x0011,0xE8},//,Frame && Exposure Control.EXP_TIME_L
{0x0012,0x0},//,Frame && Exposure Control.EXP_TIME_S
{0x0013,0x0},//,Frame && Exposure Control.EXP_TIME_S
{0x0014,0x0},//,Frame && Exposure Control.EXP_TIME_S
{0x0015,0x0},//,Frame && Exposure Control.EXP_TIME_S
{0xE004,0x1},//,Frame && Exposure Control.RW_CONTEXT
{0x000E,0x0},//,Frame && Exposure Control.EXP_TIME_L
{0x000F,0x0},//,Frame && Exposure Control.EXP_TIME_L
{0x0010,0x3},//,Frame && Exposure Control.EXP_TIME_L
{0x0011,0xE8},//,Frame && Exposure Control.EXP_TIME_L
{0x0012,0x0},//,Frame && Exposure Control.EXP_TIME_S
{0x0013,0x0},//,Frame && Exposure Control.EXP_TIME_S
{0x0014,0x0},//,Frame && Exposure Control.EXP_TIME_S
{0x0015,0x0},//,Frame && Exposure Control.EXP_TIME_S
{0xE004,0x0},//,Frame && Exposure Control.RW_CONTEXT
{0x0032,0xB},//,Frame && Exposure Control.ROW_LENGTH
{0x0033,0xFD},//,Frame && Exposure Control.ROW_LENGTH
{0xE004,0x1},//,Frame && Exposure Control.RW_CONTEXT
{0x0032,0xB},//,Frame && Exposure Control.ROW_LENGTH
{0x0033,0xFD},//,Frame && Exposure Control.ROW_LENGTH
{0xE004,0x0},//,Frame && Exposure Control.RW_CONTEXT
{0x0007,0x1},//,Frame && Exposure Control.NROF_FRAMES
{0x0008,0x0},//,Frame && Exposure Control.TARGET_FRAME_TIME
{0x0009,0x0},//,Frame && Exposure Control.TARGET_FRAME_TIME
{0x000A,0x41},//,Frame && Exposure Control.TARGET_FRAME_TIME
{0x000B,0x1B},//,Frame && Exposure Control.TARGET_FRAME_TIME
{0xE004,0x1},//,Frame && Exposure Control.RW_CONTEXT
{0x0007,0x1},//,Frame && Exposure Control.NROF_FRAMES
{0x0008,0x0},//,Frame && Exposure Control.TARGET_FRAME_TIME
{0x0009,0x0},//,Frame && Exposure Control.TARGET_FRAME_TIME
{0x000A,0x41},//,Frame && Exposure Control.TARGET_FRAME_TIME
{0x000B,0x1B},//,Frame && Exposure Control.TARGET_FRAME_TIME
{0xE004,0x0},//,Digital Gain.RW_CONTEXT
{0xE000,0x1},//,Digital Gain.BANK_SEL
{0x0024,0xF},//,Digital Gain.GDIG_AMP
{0xE004,0x1},//,Digital Gain.RW_CONTEXT
{0x0024,0xF},//,Digital Gain.GDIG_AMP
{0xE000,0x0},//,Defect Pixel Correction.BANK_SEL
{0x0057,0x0},//,Defect Pixel Correction.DEFECT_ON
{0x0058,0x0},//,Defect Pixel Correction.DEFECT_MODE
{0x0059,0x2},//,Defect Pixel Correction.DEFECT_LIMIT_HIGH
{0x005A,0x2},//,Defect Pixel Correction.DEFECT_LIMIT_LOW
{0x005B,0x0},//,Defect Pixel Correction.DEFECT_LIMIT_HIGH_MODE
{0xE000,0x0},//,Context Switching.BANK_SEL
{0xE008,0x0},//,Context Switching.DISABLE_CONTEXTSYNC
{0x0006,0x1},//,Context Switching.PARAM_HOLD
{0xE003,0x0},//,Context Switching.NEXT_ACTIVE_CONTEXT
{0x0006,0x0},//,Context Switching.PARAM_HOLD
{0xE008,0x0},//,Context Switching.DISABLE_CONTEXTSYNC
{0xE004,0x0},//,Event Detection.RW_CONTEXT
{0xE000,0x1},//,Event Detection.BANK_SEL
{0x0031,0x0},//,Event Detection.EN_EVENT_DETECTION
{0xE004,0x1},//,Event Detection.RW_CONTEXT
{0x0031,0x0},//,Event Detection.EN_EVENT_DETECTION
{0xE000,0x0},//,Event Detection.BANK_SEL
{0x0138,0x0},//,Event Detection.EN_AUTO_CNTXT_SWITCH
{0xE005,0x0},//,Event Detection.AUTO_SWITCHTO_CNTXT
{0x0139,0x0},//,Event Detection.TILE_XSTART
{0x013A,0x0},//,Event Detection.TILE_XSTART
{0x013B,0x96},//,Event Detection.TILE_WIDTH
{0x013C,0x0},//,Event Detection.TILE_YSTART
{0x013D,0x0},//,Event Detection.TILE_YSTART
{0x013E,0xA0},//,Event Detection.TILE_HEIGHT
{0x013F,0x6},//,Event Detection.TILE_DISCARD_NR_BITS
{0x0140,0x1},//,Event Detection.MIN_FLAGGED_TILES
{0x0141,0x14},//,Event Detection.MAX_FLAGGED_TILES
{0x0142,0x1},//,Event Detection.TILE_THRESHOLD
{0x0143,0x0},//,Event Detection.TILE_MIN_THRESHOLD
{0x0144,0x0},//,Event Detection.TILE_MIN_THRESHOLD
{0x0145,0x0},//,Event Detection.TILE_DISABLE_MASK
{0x0146,0x0},//,Event Detection.TILE_DISABLE_MASK
{0x0147,0x0},//,Event Detection.TILE_DISABLE_MASK
{0x0148,0x0},//,Event Detection.CPY_TILES_INTERVAL
{0xE004,0x0},//,Image Statistics.RW_CONTEXT
{0xE000,0x1},//,Image Statistics.BANK_SEL
{0x0026,0x0},//,Image Statistics.STAT_NROF_HIST
{0xE004,0x1},//,Image Statistics.RW_CONTEXT
{0x0026,0x0},//,Image Statistics.STAT_NROF_HIST
{0xE000,0x0},//,Image Statistics.BANK_SEL
{0x0169,0x12},//,Image Statistics.CSI2_EMBED_TYPE
{0xE004,0x0},//,Illumination Trigger.RW_CONTEXT
{0xE000,0x1},//,Illumination Trigger.BANK_SEL
{0x001C,0x0},//,Illumination Trigger.EN_TRIG_ILLUM
{0x001D,0x0},//,Illumination Trigger.EN_TRIG_SYNC
{0x0019,0x0},//,Illumination Trigger.ILLUM_WIDTH
{0x001A,0x7},//,Illumination Trigger.ILLUM_WIDTH
{0x001B,0x53},//,Illumination Trigger.ILLUM_WIDTH
{0x0016,0x8},//,Illumination Trigger.ILLUM_DELAY
{0x0017,0x0},//,Illumination Trigger.ILLUM_DELAY
{0x0018,0x0},//,Illumination Trigger.ILLUM_DELAY
{0xE004,0x1},//,Illumination Trigger.RW_CONTEXT
{0x001C,0x0},//,Illumination Trigger.EN_TRIG_ILLUM
{0x001D,0x0},//,Illumination Trigger.EN_TRIG_SYNC
{0x0019,0x0},//,Illumination Trigger.ILLUM_WIDTH
{0x001A,0x7},//,Illumination Trigger.ILLUM_WIDTH
{0x001B,0x53},//,Illumination Trigger.ILLUM_WIDTH
{0x0016,0x8},//,Illumination Trigger.ILLUM_DELAY
{0x0017,0x0},//,Illumination Trigger.ILLUM_DELAY
{0x0018,0x0},//,Illumination Trigger.ILLUM_DELAY
{0xE000,0x0},//,Illumination Trigger.BANK_SEL
{0x001A,0x0},//,Illumination Trigger.TRIG_SYNC_DELAY
{0x001B,0x0},//,Illumination Trigger.TRIG_SYNC_DELAY
{0x001C,0x0},//,Illumination Trigger.TRIG_SYNC_DELAY
{0x00F3,0x27},//,Illumination Trigger.DMUX0_SEL
{0xE004,0x0},//,Synchronization Trigger.RW_CONTEXT
{0xE000,0x1},//,Synchronization Trigger.BANK_SEL
{0x001D,0x0},//,Synchronization Trigger.EN_TRIG_SYNC
{0xE004,0x1},//,Synchronization Trigger.RW_CONTEXT
{0x001D,0x0},//,Synchronization Trigger.EN_TRIG_SYNC
{0xE000,0x0},//,Synchronization Trigger.BANK_SEL
{0x001A,0x0},//,Synchronization Trigger.TRIG_SYNC_DELAY
{0x001B,0x0},//,Synchronization Trigger.TRIG_SYNC_DELAY
{0x001C,0x0},//,Synchronization Trigger.TRIG_SYNC_DELAY
{0x001D,0x0},//,Synchronization Trigger.TRIG_SYNC_ON_REQ_1



};

static const struct mira050_reg full_576_768_50fps_12b_1lane_reg_post_soft_reset[] = {

};
// from API 1.5.1
static const struct mira050_reg full_576_768_50fps_10b_hs_1lane_reg_pre_soft_reset[] = {
{0xE000,0x0},//,Base Configuration.BANK_SEL
{0x01E4,0x0},//,Base Configuration
{0x01E5,0x13},//,Base Configuration
{0x01E2,0x17},//,Base Configuration
{0x01E3,0x88},//,Base Configuration
{0x01E6,0x0},//,Base Configuration
{0x01E7,0xCA},//,Base Configuration
{0x016C,0x1},//,Base Configuration
{0x016B,0x1},//,Base Configuration
{0x0208,0x1},//,Base Configuration
{0x0209,0xF0},//,Base Configuration
{0x020A,0x3},//,Base Configuration
{0x020B,0x4D},//,Base Configuration
{0x020C,0x2},//,Base Configuration
{0x020D,0x10},//,Base Configuration
{0x020E,0x3},//,Base Configuration
{0x020F,0x1},//,Base Configuration
{0x0210,0x0},//,Base Configuration
{0x0211,0x13},//,Base Configuration
{0x0212,0x0},//,Base Configuration
{0x0213,0x3},//,Base Configuration
{0x0214,0x3},//,Base Configuration
{0x0215,0xEF},//,Base Configuration
{0x0216,0x3},//,Base Configuration
{0x0217,0xF3},//,Base Configuration
{0x0218,0x3},//,Base Configuration
{0x0219,0xF4},//,Base Configuration
{0x021A,0x1},//,Base Configuration
{0x021B,0xF1},//,Base Configuration
{0x021C,0x3},//,Base Configuration
{0x021D,0x24},//,Base Configuration
{0x021E,0x0},//,Base Configuration
{0x021F,0x2},//,Base Configuration
{0x0220,0x1},//,Base Configuration
{0x0221,0xF2},//,Base Configuration
{0x0222,0x3},//,Base Configuration
{0x0223,0x2F},//,Base Configuration
{0x0224,0x0},//,Base Configuration
{0x0225,0x21},//,Base Configuration
{0x0226,0x3},//,Base Configuration
{0x0227,0xF0},//,Base Configuration
{0x0228,0x3},//,Base Configuration
{0x0229,0xF1},//,Base Configuration
{0x022A,0x3},//,Base Configuration
{0x022B,0xF2},//,Base Configuration
{0x022C,0x3},//,Base Configuration
{0x022D,0xF5},//,Base Configuration
{0x022E,0x3},//,Base Configuration
{0x022F,0xF6},//,Base Configuration
{0x0230,0x0},//,Base Configuration
{0x0231,0xC1},//,Base Configuration
{0x0232,0x0},//,Base Configuration
{0x0233,0x2},//,Base Configuration
{0x0234,0x1},//,Base Configuration
{0x0235,0xF2},//,Base Configuration
{0x0236,0x3},//,Base Configuration
{0x0237,0x6B},//,Base Configuration
{0x0238,0x3},//,Base Configuration
{0x0239,0xFF},//,Base Configuration
{0x023A,0x3},//,Base Configuration
{0x023B,0x31},//,Base Configuration
{0x023C,0x1},//,Base Configuration
{0x023D,0xF0},//,Base Configuration
{0x023E,0x3},//,Base Configuration
{0x023F,0x87},//,Base Configuration
{0x0240,0x2},//,Base Configuration
{0x0241,0x3A},//,Base Configuration
{0x0242,0x0},//,Base Configuration
{0x0243,0xB},//,Base Configuration
{0x0244,0x1},//,Base Configuration
{0x0245,0xF9},//,Base Configuration
{0x0246,0x3},//,Base Configuration
{0x0247,0xD},//,Base Configuration
{0x0248,0x0},//,Base Configuration
{0x0249,0x7},//,Base Configuration
{0x024A,0x3},//,Base Configuration
{0x024B,0xEF},//,Base Configuration
{0x024C,0x3},//,Base Configuration
{0x024D,0xF3},//,Base Configuration
{0x024E,0x3},//,Base Configuration
{0x024F,0xF4},//,Base Configuration
{0x0250,0x3},//,Base Configuration
{0x0251,0x0},//,Base Configuration
{0x0252,0x0},//,Base Configuration
{0x0253,0x7},//,Base Configuration
{0x0254,0x0},//,Base Configuration
{0x0255,0xC},//,Base Configuration
{0x0256,0x1},//,Base Configuration
{0x0257,0xF1},//,Base Configuration
{0x0258,0x3},//,Base Configuration
{0x0259,0x43},//,Base Configuration
{0x025A,0x1},//,Base Configuration
{0x025B,0xF8},//,Base Configuration
{0x025C,0x3},//,Base Configuration
{0x025D,0x10},//,Base Configuration
{0x025E,0x0},//,Base Configuration
{0x025F,0x7},//,Base Configuration
{0x0260,0x3},//,Base Configuration
{0x0261,0xF0},//,Base Configuration
{0x0262,0x3},//,Base Configuration
{0x0263,0xF1},//,Base Configuration
{0x0264,0x3},//,Base Configuration
{0x0265,0xF2},//,Base Configuration
{0x0266,0x3},//,Base Configuration
{0x0267,0xF5},//,Base Configuration
{0x0268,0x3},//,Base Configuration
{0x0269,0xF6},//,Base Configuration
{0x026A,0x3},//,Base Configuration
{0x026B,0x0},//,Base Configuration
{0x026C,0x2},//,Base Configuration
{0x026D,0x87},//,Base Configuration
{0x026E,0x2},//,Base Configuration
{0x026F,0x31},//,Base Configuration
{0x0270,0x3},//,Base Configuration
{0x0271,0xFF},//,Base Configuration
{0x0272,0x3},//,Base Configuration
{0x0273,0x0},//,Base Configuration
{0x0274,0x3},//,Base Configuration
{0x0275,0xFF},//,Base Configuration
{0x0276,0x2},//,Base Configuration
{0x0277,0x87},//,Base Configuration
{0x0278,0x3},//,Base Configuration
{0x0279,0x2},//,Base Configuration
{0x027A,0x3},//,Base Configuration
{0x027B,0x9},//,Base Configuration
{0x027C,0x3},//,Base Configuration
{0x027D,0xF7},//,Base Configuration
{0x027E,0x0},//,Base Configuration
{0x027F,0x16},//,Base Configuration
{0x0280,0x0},//,Base Configuration
{0x0281,0x33},//,Base Configuration
{0x0282,0x0},//,Base Configuration
{0x0283,0x4},//,Base Configuration
{0x0284,0x0},//,Base Configuration
{0x0285,0x11},//,Base Configuration
{0x0286,0x3},//,Base Configuration
{0x0287,0x9},//,Base Configuration
{0x0288,0x0},//,Base Configuration
{0x0289,0x2},//,Base Configuration
{0x028A,0x0},//,Base Configuration
{0x028B,0x20},//,Base Configuration
{0x028C,0x0},//,Base Configuration
{0x028D,0xB5},//,Base Configuration
{0x028E,0x1},//,Base Configuration
{0x028F,0x5},//,Base Configuration
{0x0290,0x0},//,Base Configuration
{0x0291,0x12},//,Base Configuration
{0x0292,0x0},//,Base Configuration
{0x0293,0xB5},//,Base Configuration
{0x0294,0x1},//,Base Configuration
{0x0295,0x5},//,Base Configuration
{0x0296,0x0},//,Base Configuration
{0x0297,0x0},//,Base Configuration
{0x0298,0x0},//,Base Configuration
{0x0299,0x12},//,Base Configuration
{0x029A,0x0},//,Base Configuration
{0x029B,0x12},//,Base Configuration
{0x029C,0x0},//,Base Configuration
{0x029D,0x20},//,Base Configuration
{0x029E,0x0},//,Base Configuration
{0x029F,0xB5},//,Base Configuration
{0x02A0,0x1},//,Base Configuration
{0x02A1,0x5},//,Base Configuration
{0x02A2,0x0},//,Base Configuration
{0x02A3,0x0},//,Base Configuration
{0x02A4,0x0},//,Base Configuration
{0x02A5,0x12},//,Base Configuration
{0x02A6,0x0},//,Base Configuration
{0x02A7,0x12},//,Base Configuration
{0x02A8,0x0},//,Base Configuration
{0x02A9,0x20},//,Base Configuration
{0x02AA,0x0},//,Base Configuration
{0x02AB,0x47},//,Base Configuration
{0x02AC,0x0},//,Base Configuration
{0x02AD,0x27},//,Base Configuration
{0x02AE,0x0},//,Base Configuration
{0x02AF,0xB5},//,Base Configuration
{0x02B0,0x0},//,Base Configuration
{0x02B1,0xE5},//,Base Configuration
{0x02B2,0x0},//,Base Configuration
{0x02B3,0x0},//,Base Configuration
{0x02B4,0x0},//,Base Configuration
{0x02B5,0x4},//,Base Configuration
{0x02B6,0x0},//,Base Configuration
{0x02B7,0x43},//,Base Configuration
{0x02B8,0x0},//,Base Configuration
{0x02B9,0x1},//,Base Configuration
{0x02BA,0x3},//,Base Configuration
{0x02BB,0x2},//,Base Configuration
{0x02BC,0x0},//,Base Configuration
{0x02BD,0x8},//,Base Configuration
{0x02BE,0x3},//,Base Configuration
{0x02BF,0xFF},//,Base Configuration
{0x02C0,0x2},//,Base Configuration
{0x02C1,0x87},//,Base Configuration
{0x02C2,0x3},//,Base Configuration
{0x02C3,0x89},//,Base Configuration
{0x02C4,0x3},//,Base Configuration
{0x02C5,0xF7},//,Base Configuration
{0x02C6,0x0},//,Base Configuration
{0x02C7,0x77},//,Base Configuration
{0x02C8,0x0},//,Base Configuration
{0x02C9,0x17},//,Base Configuration
{0x02CA,0x0},//,Base Configuration
{0x02CB,0x8},//,Base Configuration
{0x02CC,0x3},//,Base Configuration
{0x02CD,0xFF},//,Base Configuration
{0x02CE,0x0},//,Base Configuration
{0x02CF,0x38},//,Base Configuration
{0x02D0,0x0},//,Base Configuration
{0x02D1,0x17},//,Base Configuration
{0x02D2,0x0},//,Base Configuration
{0x02D3,0x8},//,Base Configuration
{0x02D4,0x3},//,Base Configuration
{0x02D5,0xFF},//,Base Configuration
{0x02D6,0x3},//,Base Configuration
{0x02D7,0xFF},//,Base Configuration
{0x02D8,0x3},//,Base Configuration
{0x02D9,0xFF},//,Base Configuration
{0x02DA,0x3},//,Base Configuration
{0x02DB,0xFF},//,Base Configuration
{0x02DC,0x3},//,Base Configuration
{0x02DD,0xFF},//,Base Configuration
{0x02DE,0x3},//,Base Configuration
{0x02DF,0xFF},//,Base Configuration
{0x02E0,0x3},//,Base Configuration
{0x02E1,0xFF},//,Base Configuration
{0x02E2,0x3},//,Base Configuration
{0x02E3,0xFF},//,Base Configuration
{0x02E4,0x3},//,Base Configuration
{0x02E5,0xFF},//,Base Configuration
{0x02E6,0x3},//,Base Configuration
{0x02E7,0xFF},//,Base Configuration
{0x02E8,0x3},//,Base Configuration
{0x02E9,0xFF},//,Base Configuration
{0x02EA,0x3},//,Base Configuration
{0x02EB,0xFF},//,Base Configuration
{0x02EC,0x3},//,Base Configuration
{0x02ED,0xFF},//,Base Configuration
{0x02EE,0x3},//,Base Configuration
{0x02EF,0xFF},//,Base Configuration
{0x02F0,0x3},//,Base Configuration
{0x02F1,0xFF},//,Base Configuration
{0x02F2,0x3},//,Base Configuration
{0x02F3,0xFF},//,Base Configuration
{0x02F4,0x3},//,Base Configuration
{0x02F5,0xFF},//,Base Configuration
{0x02F6,0x3},//,Base Configuration
{0x02F7,0xFF},//,Base Configuration
{0x02F8,0x3},//,Base Configuration
{0x02F9,0xFF},//,Base Configuration
{0x02FA,0x3},//,Base Configuration
{0x02FB,0xFF},//,Base Configuration
{0x02FC,0x3},//,Base Configuration
{0x02FD,0xFF},//,Base Configuration
{0x02FE,0x3},//,Base Configuration
{0x02FF,0xFF},//,Base Configuration
{0x0300,0x3},//,Base Configuration
{0x0301,0xFF},//,Base Configuration
{0x0302,0x3},//,Base Configuration
{0x0303,0xFF},//,Base Configuration
{0x01E9,0x0},//,Base Configuration
{0x01E8,0x19},//,Base Configuration
{0x01EA,0x35},//,Base Configuration
{0x01EB,0x37},//,Base Configuration
{0x01EC,0x5C},//,Base Configuration
{0x01ED,0x63},//,Base Configuration
{0x01F8,0xF},//,Base Configuration
{0x005C,0x0},//,Base Configuration
{0x005D,0x0},//,Base Configuration
{0x01DA,0x1},//,Base Configuration
{0x01DC,0x1},//,Base Configuration
{0x01DE,0x1},//,Base Configuration
{0x0189,0x1},//,Base Configuration
{0x01B7,0x1},//,Base Configuration
{0x01C1,0xE},//,Base Configuration
{0x01C2,0xF6},//,Base Configuration
{0x01C3,0xFF},//,Base Configuration
{0x01B8,0x1},//,Base Configuration
{0x01BA,0x32},//,Base Configuration
{0x01BD,0x8},//,Base Configuration
{0x01CA,0x1E},//,Base Configuration
{0x01C9,0x1E},//,Base Configuration
{0x01BF,0x3C},//,Base Configuration
{0x01C0,0x5C},//,Base Configuration
{0x0071,0x1},//,Base Configuration
{0x01B4,0x1},//,Base Configuration
{0x01B5,0x1},//,Base Configuration
{0x01F1,0x1},//,Base Configuration
{0x01F4,0x1},//,Base Configuration
{0x01F5,0x1},//,Base Configuration
{0x0314,0x1},//,Base Configuration
{0x0315,0x1},//,Base Configuration
{0x0316,0x1},//,Base Configuration
{0x0207,0x0},//,Base Configuration
{0x4207,0x2},//,Base Configuration
{0x2207,0x2},//,Base Configuration
{0x209D,0x0},//,Base Configuration
{0x0063,0x1},//,Base Configuration
{0x01F7,0xF},//,Base Configuration
{0x00E9,0x3},//,Base Configuration
{0x00EA,0x28},//,Base Configuration
{0x0309,0x7},//,Base Configuration
{0x030A,0x4},//,Base Configuration
{0x030B,0xD},//,Base Configuration
{0x030C,0x7},//,Base Configuration
{0x030E,0x15},//,Base Configuration
{0x030D,0xA},//,Base Configuration
{0x030F,0x1},//,Base Configuration
{0x0310,0xF},//,Base Configuration
{0x01D0,0x1F},//,Base Configuration
{0x01D1,0x12},//,Base Configuration
{0x0016,0x0},//,Base Configuration
{0x0017,0x5},//,Base Configuration
{0x00E8,0x3},//,Base Configuration
{0xE0C0,0x0},//,Base Configuration
{0xE0C1,0x10},//,Base Configuration
{0xE0C2,0x0},//,Base Configuration
{0xE0C3,0x10},//,Base Configuration
{0x016A,0x1},//,Base Configuration.DPATH_BITMODE
{0x0168,0x2B},//,Base Configuration.CSI2_DATA_TYPE
{0xE000,0x0},//,PLL.BANK_SEL
{0x2077,0x0},//,PLL.PLL_DIV_N
{0x2076,0xBD},//,PLL.PLL_DIV_M
{0x00CE,0x1},//,PLL.CLKGEN_CP_DIV
{0x0070,0x9},//,PLL.OTP_GRANULARITY
{0x016D,0x32},//,PLL.GRAN_TG
{0x0176,0x0},//,PLL.LUT_DEL_008
{0x20C6,0x0},//,PLL.CLKGEN_TX_ESC_H
{0x20C7,0x0},//,PLL.CLKGEN_TX_ESC_H
{0x20C8,0x1},//,PLL.CLKGEN_TX_ESC_H
{0x20C9,0x0},//,PLL.CLKGEN_TX_ESC_L
{0x20CA,0x0},//,PLL.CLKGEN_TX_ESC_L
{0x20CB,0x1},//,PLL.CLKGEN_TX_ESC_L
{0x2075,0x0},//,PLL.PLL_PD
{0xE000,0x0},//,Low Power State.BANK_SEL
{0x001E,0x1},//,Low Power State.EN_AUTO_LPS_TRANS
{0xE000,0x0},//,MIPI.BANK_SEL
{0x207E,0x0},//,MIPI.MIPI_VC_ID
{0x207F,0x0},//,MIPI.MIPI_FRAME_MODE_PIX
{0x2080,0x0},//,MIPI.MIPI_FRAME_MODE_EMBED
{0x2081,0x3},//,MIPI.MIPI_LINE_COUNT_EN
{0x2082,0x0},//,MIPI.MIPI_FRAME_COUNT_WRAP
{0x2083,0x2},//,MIPI.MIPI_FRAME_COUNT_WRAP
{0x0090,0x0},//,MIPI.CSI2_CONT_CLOCK_MODE
{0x2097,0x0},//,MIPI.CSI2_SCRAMBLING_EN
{0xE000,0x0},//,Sensor Control Mode.BANK_SEL
{0x0011,0x3},//,Sensor Control Mode.CTRL_MODE
{0x011D,0x0},//,Sensor Control Mode.SINGLE_PIN_CTRL
{0xE000,0x0},//,Time Bases.BANK_SEL
{0x0012,0x0},//,Time Bases.CLOCKS_PER_TIME_UNIT
{0x0013,0x18},//,Time Bases.CLOCKS_PER_TIME_UNIT
{0x015A,0x0},//,Time Bases.GLOB_TIME
{0x015B,0x33},//,Time Bases.GLOB_TIME
{0x015C,0x0},//,Time Bases.GLOB_TIME_A
{0x015D,0x33},//,Time Bases.GLOB_TIME_A
{0x015E,0x0},//,Time Bases.GLOB_TIME_B
{0x015F,0x33},//,Time Bases.GLOB_TIME_B
{0x0162,0x0},//,Time Bases.ENTER_LPS_TIME
{0x0163,0x5},//,Time Bases.ENTER_LPS_TIME
{0x0164,0x4},//,Time Bases.EXIT_LPS_TIME
{0x0165,0x4C},//,Time Bases.EXIT_LPS_TIME
{0x0166,0x4},//,Time Bases.LPS_CYCLE_TIME
{0x0167,0x4C},//,Time Bases.LPS_CYCLE_TIME
{0xE000,0x0},//,Analog Gain.BANK_SEL
{0x01BB,0xC8},//,Analog Gain
{0x01BC,0xC0},//,Analog Gain
{0x00D0,0x0},//,Analog Gain
{0x016E,0xBA},//,Analog Gain.LUT_DEL_000
{0x0172,0x0},//,Analog Gain.LUT_DEL_004
{0x0173,0x0},//,Analog Gain.LUT_DEL_005
{0x016F,0x7E},//,Analog Gain.LUT_DEL_001
{0x0170,0x0},//,Analog Gain.LUT_DEL_002
{0x0171,0xBA},//,Analog Gain.LUT_DEL_003
{0x0174,0x0},//,Analog Gain.LUT_DEL_006
{0x0175,0x20},//,Analog Gain.LUT_DEL_007
{0x018B,0x3},//,Analog Gain
{0x018C,0x2},//,Analog Gain
{0x018D,0x2},//,Analog Gain
{0x018E,0x56},//,Analog Gain
{0x018F,0x5},//,Analog Gain
{0x0190,0x7F},//,Analog Gain
{0x01EE,0x15},//,Analog Gain.SHUTTER_LAG
{0x01EF,0xD8},//,Analog Gain.SHUTTER_LAG
{0x01A2,0x5},//,Analog Gain.POS_ANACOL_TRIGGER
{0x01A3,0x6F},//,Analog Gain.POS_ANACOL_TRIGGER
{0x031F,0x5},//,Analog Gain.POS_YADDR_TRIGGER
{0x0320,0x78},//,Analog Gain.POS_YADDR_TRIGGER
{0x01A6,0x6},//,Analog Gain.POS_ADC_TRIGGER
{0x01A7,0x6},//,Analog Gain.POS_ADC_TRIGGER
{0x01A4,0x9},//,Analog Gain.POS_ANACOL_YBIN_TRIGGER
{0x01A5,0x30},//,Analog Gain.POS_ANACOL_YBIN_TRIGGER
{0x0321,0x9},//,Analog Gain.POS_YADDR_YBIN_TRIGGER
{0x0322,0x39},//,Analog Gain.POS_YADDR_YBIN_TRIGGER
{0x01A8,0x9},//,Analog Gain.POS_ADC_YBIN_TRIGGER
{0x01A9,0xC7},//,Analog Gain.POS_ADC_YBIN_TRIGGER
{0x01A0,0x0},//,Analog Gain.POS_VIS_TRIGGER
{0x01A1,0xCC},//,Analog Gain.POS_VIS_TRIGGER
{0x01B2,0x0},//,Analog Gain.POS_HSYNC_RISE
{0x01B3,0xE4},//,Analog Gain.POS_HSYNC_RISE
{0x01B0,0x0},//,Analog Gain.POS_STAT_TRIGGER
{0x01B1,0xDF},//,Analog Gain.POS_STAT_TRIGGER
{0x01AC,0x0},//,Analog Gain.POS_SVAL_TRIGGER
{0x01AD,0xEA},//,Analog Gain.POS_SVAL_TRIGGER
{0x01F0,0x24},//,Analog Gain
{0x01F3,0x1},//,Analog Gain
{0xE000,0x0},//,Black Level.BANK_SEL
{0x0193,0xF},//,Black Level.OFFSET_CLIPPING
{0x0194,0xA8},//,Black Level.OFFSET_CLIPPING
{0xE000,0x0},//,Release Soft Reset.BANK_SEL
{0xE009,0x1},//,Release Soft Reset
{0x212F,0x1},//,Release Soft Reset
{0x2130,0x1},//,Release Soft Reset
{0x2131,0x1},//,Release Soft Reset
{0x2132,0x1},//,Release Soft Reset
{0x2133,0x1},//,Release Soft Reset
{0x2134,0x1},//,Release Soft Reset
{0x2135,0x1},//,Release Soft Reset
{0xE0E1,0x1},//,Release Soft Reset
{0x018A,0x1},//,Release Soft Reset
{0x00E0,0x1},//,Release Soft Reset
{0xE004,0x0},//,Horizontal ROI.RW_CONTEXT
{0xE000,0x1},//,Horizontal ROI.BANK_SEL
{0xE02C,0x0},//,Horizontal ROI.XWIN_LEFT
{0xE02D,0xC},//,Horizontal ROI.XWIN_LEFT
{0xE02E,0x2},//,Horizontal ROI.XWIN_RIGHT
{0xE02F,0x4B},//,Horizontal ROI.XWIN_RIGHT
{0xE030,0x0},//,Horizontal ROI.XMIRROR
{0xE025,0x0},//,Horizontal ROI.XSUBSAMPLING
{0xE02A,0x0},//,Horizontal ROI.XBINNING
{0x2029,0x46},//,Horizontal ROI.MIPI_DATA_FIFO_THRESHOLD
{0x0034,0x1},//,Horizontal ROI.HSYNC_LENGTH
{0x0035,0x20},//,Horizontal ROI.HSYNC_LENGTH
{0xE004,0x1},//,Horizontal ROI.RW_CONTEXT
{0xE02C,0x0},//,Horizontal ROI.XWIN_LEFT
{0xE02D,0x0},//,Horizontal ROI.XWIN_LEFT
{0xE02E,0x2},//,Horizontal ROI.XWIN_RIGHT
{0xE02F,0x57},//,Horizontal ROI.XWIN_RIGHT
{0xE030,0x0},//,Horizontal ROI.XMIRROR
{0xE025,0x0},//,Horizontal ROI.XSUBSAMPLING
{0xE02A,0x0},//,Horizontal ROI.XBINNING
{0x2029,0x46},//,Horizontal ROI.MIPI_DATA_FIFO_THRESHOLD
{0x0034,0x1},//,Horizontal ROI.HSYNC_LENGTH
{0x0035,0x2C},//,Horizontal ROI.HSYNC_LENGTH
{0xE004,0x0},//,Vertical ROI.RW_CONTEXT
{0xE000,0x1},//,Vertical ROI.BANK_SEL
{0x001E,0x0},//,Vertical ROI.YWIN_ENA
{0x001F,0x1},//,Vertical ROI.YWIN_ENA
{0x002B,0x0},//,Vertical ROI.YBINNING
{0xE004,0x1},//,Vertical ROI.RW_CONTEXT
{0x001E,0x0},//,Vertical ROI.YWIN_ENA
{0x001F,0x1},//,Vertical ROI.YWIN_ENA
{0x002B,0x0},//,Vertical ROI.YBINNING
{0xE000,0x0},//,Vertical ROI.BANK_SEL
{0x001F,0x0},//,Vertical ROI.YWIN_BLACK
{0x0020,0x0},//,Vertical ROI.YWIN_BLACK
{0x0023,0x0},//,Vertical ROI.YWIN_DIR
{0x0024,0x3},//,Vertical ROI.YWIN0_SIZE
{0x0025,0x0},//,Vertical ROI.YWIN0_SIZE
{0x0026,0x0},//,Vertical ROI.YWIN0_START
{0x0027,0x18},//,Vertical ROI.YWIN0_START
{0x0028,0x0},//,Vertical ROI.YWIN0_SUBS
{0x0029,0x0},//,Vertical ROI.YWIN1_SIZE
{0x002A,0x0},//,Vertical ROI.YWIN1_SIZE
{0x002B,0x0},//,Vertical ROI.YWIN1_START
{0x002C,0x0},//,Vertical ROI.YWIN1_START
{0x002D,0x0},//,Vertical ROI.YWIN1_SUBS
{0x002E,0x0},//,Vertical ROI.YWIN2_SIZE
{0x002F,0x0},//,Vertical ROI.YWIN2_SIZE
{0x0030,0x0},//,Vertical ROI.YWIN2_START
{0x0031,0x0},//,Vertical ROI.YWIN2_START
{0x0032,0x0},//,Vertical ROI.YWIN2_SUBS
{0x0033,0x0},//,Vertical ROI.YWIN3_SIZE
{0x0034,0x0},//,Vertical ROI.YWIN3_SIZE
{0x0035,0x0},//,Vertical ROI.YWIN3_START
{0x0036,0x0},//,Vertical ROI.YWIN3_START
{0x0037,0x0},//,Vertical ROI.YWIN3_SUBS
{0x0038,0x0},//,Vertical ROI.YWIN4_SIZE
{0x0039,0x0},//,Vertical ROI.YWIN4_SIZE
{0x003A,0x0},//,Vertical ROI.YWIN4_START
{0x003B,0x0},//,Vertical ROI.YWIN4_START
{0x003C,0x0},//,Vertical ROI.YWIN4_SUBS
{0x003D,0x0},//,Vertical ROI.YWIN5_SIZE
{0x003E,0x0},//,Vertical ROI.YWIN5_SIZE
{0x003F,0x0},//,Vertical ROI.YWIN5_START
{0x0040,0x0},//,Vertical ROI.YWIN5_START
{0x0041,0x0},//,Vertical ROI.YWIN5_SUBS
{0x0042,0x0},//,Vertical ROI.YWIN6_SIZE
{0x0043,0x0},//,Vertical ROI.YWIN6_SIZE
{0x0044,0x0},//,Vertical ROI.YWIN6_START
{0x0045,0x0},//,Vertical ROI.YWIN6_START
{0x0046,0x0},//,Vertical ROI.YWIN6_SUBS
{0x0047,0x0},//,Vertical ROI.YWIN7_SIZE
{0x0048,0x0},//,Vertical ROI.YWIN7_SIZE
{0x0049,0x0},//,Vertical ROI.YWIN7_START
{0x004A,0x0},//,Vertical ROI.YWIN7_START
{0x004B,0x0},//,Vertical ROI.YWIN7_SUBS
{0x004C,0x0},//,Vertical ROI.YWIN8_SIZE
{0x004D,0x0},//,Vertical ROI.YWIN8_SIZE
{0x004E,0x0},//,Vertical ROI.YWIN8_START
{0x004F,0x0},//,Vertical ROI.YWIN8_START
{0x0050,0x0},//,Vertical ROI.YWIN8_SUBS
{0x0051,0x0},//,Vertical ROI.YWIN9_SIZE
{0x0052,0x0},//,Vertical ROI.YWIN9_SIZE
{0x0053,0x0},//,Vertical ROI.YWIN9_START
{0x0054,0x0},//,Vertical ROI.YWIN9_START
{0x0055,0x0},//,Vertical ROI.YWIN9_SUBS
{0xE004,0x0},//,Frame && Exposure Control.RW_CONTEXT
{0xE000,0x1},//,Frame && Exposure Control.BANK_SEL
{0x000E,0x0},//,Frame && Exposure Control.EXP_TIME_L
{0x000F,0x0},//,Frame && Exposure Control.EXP_TIME_L
{0x0010,0x3},//,Frame && Exposure Control.EXP_TIME_L
{0x0011,0xE8},//,Frame && Exposure Control.EXP_TIME_L
{0x0012,0x0},//,Frame && Exposure Control.EXP_TIME_S
{0x0013,0x0},//,Frame && Exposure Control.EXP_TIME_S
{0x0014,0x0},//,Frame && Exposure Control.EXP_TIME_S
{0x0015,0x0},//,Frame && Exposure Control.EXP_TIME_S
{0xE004,0x1},//,Frame && Exposure Control.RW_CONTEXT
{0x000E,0x0},//,Frame && Exposure Control.EXP_TIME_L
{0x000F,0x0},//,Frame && Exposure Control.EXP_TIME_L
{0x0010,0x3},//,Frame && Exposure Control.EXP_TIME_L
{0x0011,0xE8},//,Frame && Exposure Control.EXP_TIME_L
{0x0012,0x0},//,Frame && Exposure Control.EXP_TIME_S
{0x0013,0x0},//,Frame && Exposure Control.EXP_TIME_S
{0x0014,0x0},//,Frame && Exposure Control.EXP_TIME_S
{0x0015,0x0},//,Frame && Exposure Control.EXP_TIME_S
{0xE004,0x0},//,Frame && Exposure Control.RW_CONTEXT
{0x0032,0x7},//,Frame && Exposure Control.ROW_LENGTH
{0x0033,0x78},//,Frame && Exposure Control.ROW_LENGTH
{0xE004,0x1},//,Frame && Exposure Control.RW_CONTEXT
{0x0032,0x7},//,Frame && Exposure Control.ROW_LENGTH
{0x0033,0x78},//,Frame && Exposure Control.ROW_LENGTH
{0xE004,0x0},//,Frame && Exposure Control.RW_CONTEXT
{0x0007,0x1},//,Frame && Exposure Control.NROF_FRAMES
{0x0008,0x0},//,Frame && Exposure Control.TARGET_FRAME_TIME
{0x0009,0x0},//,Frame && Exposure Control.TARGET_FRAME_TIME
{0x000A,0x41},//,Frame && Exposure Control.TARGET_FRAME_TIME
{0x000B,0x1B},//,Frame && Exposure Control.TARGET_FRAME_TIME
{0xE004,0x1},//,Frame && Exposure Control.RW_CONTEXT
{0x0007,0x1},//,Frame && Exposure Control.NROF_FRAMES
{0x0008,0x0},//,Frame && Exposure Control.TARGET_FRAME_TIME
{0x0009,0x0},//,Frame && Exposure Control.TARGET_FRAME_TIME
{0x000A,0x41},//,Frame && Exposure Control.TARGET_FRAME_TIME
{0x000B,0x1B},//,Frame && Exposure Control.TARGET_FRAME_TIME
{0xE004,0x0},//,Digital Gain.RW_CONTEXT
{0xE000,0x1},//,Digital Gain.BANK_SEL
{0x0024,0xF},//,Digital Gain.GDIG_AMP
{0xE004,0x1},//,Digital Gain.RW_CONTEXT
{0x0024,0xF},//,Digital Gain.GDIG_AMP
{0xE000,0x0},//,Defect Pixel Correction.BANK_SEL
{0x0057,0x0},//,Defect Pixel Correction.DEFECT_ON
{0x0058,0x0},//,Defect Pixel Correction.DEFECT_MODE
{0x0059,0x2},//,Defect Pixel Correction.DEFECT_LIMIT_HIGH
{0x005A,0x2},//,Defect Pixel Correction.DEFECT_LIMIT_LOW
{0x005B,0x0},//,Defect Pixel Correction.DEFECT_LIMIT_HIGH_MODE
{0xE000,0x0},//,Context Switching.BANK_SEL
{0xE008,0x0},//,Context Switching.DISABLE_CONTEXTSYNC
{0x0006,0x1},//,Context Switching.PARAM_HOLD
{0xE003,0x0},//,Context Switching.NEXT_ACTIVE_CONTEXT
{0x0006,0x0},//,Context Switching.PARAM_HOLD
{0xE008,0x0},//,Context Switching.DISABLE_CONTEXTSYNC
{0xE004,0x0},//,Event Detection.RW_CONTEXT
{0xE000,0x1},//,Event Detection.BANK_SEL
{0x0031,0x0},//,Event Detection.EN_EVENT_DETECTION
{0xE004,0x1},//,Event Detection.RW_CONTEXT
{0x0031,0x0},//,Event Detection.EN_EVENT_DETECTION
{0xE000,0x0},//,Event Detection.BANK_SEL
{0x0138,0x0},//,Event Detection.EN_AUTO_CNTXT_SWITCH
{0xE005,0x0},//,Event Detection.AUTO_SWITCHTO_CNTXT
{0x0139,0x0},//,Event Detection.TILE_XSTART
{0x013A,0x0},//,Event Detection.TILE_XSTART
{0x013B,0x96},//,Event Detection.TILE_WIDTH
{0x013C,0x0},//,Event Detection.TILE_YSTART
{0x013D,0x0},//,Event Detection.TILE_YSTART
{0x013E,0xA0},//,Event Detection.TILE_HEIGHT
{0x013F,0x6},//,Event Detection.TILE_DISCARD_NR_BITS
{0x0140,0x1},//,Event Detection.MIN_FLAGGED_TILES
{0x0141,0x14},//,Event Detection.MAX_FLAGGED_TILES
{0x0142,0x1},//,Event Detection.TILE_THRESHOLD
{0x0143,0x0},//,Event Detection.TILE_MIN_THRESHOLD
{0x0144,0x0},//,Event Detection.TILE_MIN_THRESHOLD
{0x0145,0x0},//,Event Detection.TILE_DISABLE_MASK
{0x0146,0x0},//,Event Detection.TILE_DISABLE_MASK
{0x0147,0x0},//,Event Detection.TILE_DISABLE_MASK
{0x0148,0x0},//,Event Detection.CPY_TILES_INTERVAL
{0xE004,0x0},//,Image Statistics.RW_CONTEXT
{0xE000,0x1},//,Image Statistics.BANK_SEL
{0x0026,0x0},//,Image Statistics.STAT_NROF_HIST
{0xE004,0x1},//,Image Statistics.RW_CONTEXT
{0x0026,0x0},//,Image Statistics.STAT_NROF_HIST
{0xE000,0x0},//,Image Statistics.BANK_SEL
{0x0169,0x12},//,Image Statistics.CSI2_EMBED_TYPE
{0xE004,0x0},//,Illumination Trigger.RW_CONTEXT
{0xE000,0x1},//,Illumination Trigger.BANK_SEL
{0x001C,0x0},//,Illumination Trigger.EN_TRIG_ILLUM
{0x001D,0x0},//,Illumination Trigger.EN_TRIG_SYNC
{0x0019,0x0},//,Illumination Trigger.ILLUM_WIDTH
{0x001A,0x7},//,Illumination Trigger.ILLUM_WIDTH
{0x001B,0x53},//,Illumination Trigger.ILLUM_WIDTH
{0x0016,0x8},//,Illumination Trigger.ILLUM_DELAY
{0x0017,0x0},//,Illumination Trigger.ILLUM_DELAY
{0x0018,0x0},//,Illumination Trigger.ILLUM_DELAY
{0xE004,0x1},//,Illumination Trigger.RW_CONTEXT
{0x001C,0x0},//,Illumination Trigger.EN_TRIG_ILLUM
{0x001D,0x0},//,Illumination Trigger.EN_TRIG_SYNC
{0x0019,0x0},//,Illumination Trigger.ILLUM_WIDTH
{0x001A,0x7},//,Illumination Trigger.ILLUM_WIDTH
{0x001B,0x53},//,Illumination Trigger.ILLUM_WIDTH
{0x0016,0x8},//,Illumination Trigger.ILLUM_DELAY
{0x0017,0x0},//,Illumination Trigger.ILLUM_DELAY
{0x0018,0x0},//,Illumination Trigger.ILLUM_DELAY
{0xE000,0x0},//,Illumination Trigger.BANK_SEL
{0x001A,0x0},//,Illumination Trigger.TRIG_SYNC_DELAY
{0x001B,0x0},//,Illumination Trigger.TRIG_SYNC_DELAY
{0x001C,0x0},//,Illumination Trigger.TRIG_SYNC_DELAY
{0x00F3,0x27},//,Illumination Trigger.DMUX0_SEL
{0xE004,0x0},//,Synchronization Trigger.RW_CONTEXT
{0xE000,0x1},//,Synchronization Trigger.BANK_SEL
{0x001D,0x0},//,Synchronization Trigger.EN_TRIG_SYNC
{0xE004,0x1},//,Synchronization Trigger.RW_CONTEXT
{0x001D,0x0},//,Synchronization Trigger.EN_TRIG_SYNC
{0xE000,0x0},//,Synchronization Trigger.BANK_SEL
{0x001A,0x0},//,Synchronization Trigger.TRIG_SYNC_DELAY
{0x001B,0x0},//,Synchronization Trigger.TRIG_SYNC_DELAY
{0x001C,0x0},//,Synchronization Trigger.TRIG_SYNC_DELAY
{0x001D,0x0},//,Synchronization Trigger.TRIG_SYNC_ON_REQ_1


};

static const struct mira050_reg full_576_768_50fps_10b_hs_1lane_reg_post_soft_reset[] = {

};

// api 1.5.1, 1500Mbit/s 60 fps
static const struct mira050_reg full_576_768_50fps_8b_1lane_reg_pre_soft_reset[] = {
	{0xE000,0x0},//,Base Configuration.BANK_SEL
{0x01E4,0x0},//,Base Configuration
{0x01E5,0x13},//,Base Configuration
{0x01E2,0x17},//,Base Configuration
{0x01E3,0x88},//,Base Configuration
{0x01E6,0x0},//,Base Configuration
{0x01E7,0xCA},//,Base Configuration
{0x016C,0x1},//,Base Configuration
{0x016B,0x1},//,Base Configuration
{0x0208,0x1},//,Base Configuration
{0x0209,0xF0},//,Base Configuration
{0x020A,0x3},//,Base Configuration
{0x020B,0x4D},//,Base Configuration
{0x020C,0x2},//,Base Configuration
{0x020D,0x10},//,Base Configuration
{0x020E,0x3},//,Base Configuration
{0x020F,0x1},//,Base Configuration
{0x0210,0x0},//,Base Configuration
{0x0211,0x13},//,Base Configuration
{0x0212,0x0},//,Base Configuration
{0x0213,0x3},//,Base Configuration
{0x0214,0x3},//,Base Configuration
{0x0215,0xEF},//,Base Configuration
{0x0216,0x3},//,Base Configuration
{0x0217,0xF3},//,Base Configuration
{0x0218,0x3},//,Base Configuration
{0x0219,0xF4},//,Base Configuration
{0x021A,0x1},//,Base Configuration
{0x021B,0xF1},//,Base Configuration
{0x021C,0x3},//,Base Configuration
{0x021D,0x24},//,Base Configuration
{0x021E,0x0},//,Base Configuration
{0x021F,0x2},//,Base Configuration
{0x0220,0x1},//,Base Configuration
{0x0221,0xF2},//,Base Configuration
{0x0222,0x3},//,Base Configuration
{0x0223,0x2F},//,Base Configuration
{0x0224,0x0},//,Base Configuration
{0x0225,0x21},//,Base Configuration
{0x0226,0x3},//,Base Configuration
{0x0227,0xF0},//,Base Configuration
{0x0228,0x3},//,Base Configuration
{0x0229,0xF1},//,Base Configuration
{0x022A,0x3},//,Base Configuration
{0x022B,0xF2},//,Base Configuration
{0x022C,0x3},//,Base Configuration
{0x022D,0xF5},//,Base Configuration
{0x022E,0x3},//,Base Configuration
{0x022F,0xF6},//,Base Configuration
{0x0230,0x0},//,Base Configuration
{0x0231,0xC1},//,Base Configuration
{0x0232,0x0},//,Base Configuration
{0x0233,0x2},//,Base Configuration
{0x0234,0x1},//,Base Configuration
{0x0235,0xF2},//,Base Configuration
{0x0236,0x3},//,Base Configuration
{0x0237,0x6B},//,Base Configuration
{0x0238,0x3},//,Base Configuration
{0x0239,0xFF},//,Base Configuration
{0x023A,0x3},//,Base Configuration
{0x023B,0x31},//,Base Configuration
{0x023C,0x1},//,Base Configuration
{0x023D,0xF0},//,Base Configuration
{0x023E,0x3},//,Base Configuration
{0x023F,0x87},//,Base Configuration
{0x0240,0x2},//,Base Configuration
{0x0241,0x3A},//,Base Configuration
{0x0242,0x0},//,Base Configuration
{0x0243,0xB},//,Base Configuration
{0x0244,0x1},//,Base Configuration
{0x0245,0xF9},//,Base Configuration
{0x0246,0x3},//,Base Configuration
{0x0247,0xD},//,Base Configuration
{0x0248,0x0},//,Base Configuration
{0x0249,0x7},//,Base Configuration
{0x024A,0x3},//,Base Configuration
{0x024B,0xEF},//,Base Configuration
{0x024C,0x3},//,Base Configuration
{0x024D,0xF3},//,Base Configuration
{0x024E,0x3},//,Base Configuration
{0x024F,0xF4},//,Base Configuration
{0x0250,0x3},//,Base Configuration
{0x0251,0x0},//,Base Configuration
{0x0252,0x0},//,Base Configuration
{0x0253,0x7},//,Base Configuration
{0x0254,0x0},//,Base Configuration
{0x0255,0xC},//,Base Configuration
{0x0256,0x1},//,Base Configuration
{0x0257,0xF1},//,Base Configuration
{0x0258,0x3},//,Base Configuration
{0x0259,0x43},//,Base Configuration
{0x025A,0x1},//,Base Configuration
{0x025B,0xF8},//,Base Configuration
{0x025C,0x3},//,Base Configuration
{0x025D,0x10},//,Base Configuration
{0x025E,0x0},//,Base Configuration
{0x025F,0x7},//,Base Configuration
{0x0260,0x3},//,Base Configuration
{0x0261,0xF0},//,Base Configuration
{0x0262,0x3},//,Base Configuration
{0x0263,0xF1},//,Base Configuration
{0x0264,0x3},//,Base Configuration
{0x0265,0xF2},//,Base Configuration
{0x0266,0x3},//,Base Configuration
{0x0267,0xF5},//,Base Configuration
{0x0268,0x3},//,Base Configuration
{0x0269,0xF6},//,Base Configuration
{0x026A,0x3},//,Base Configuration
{0x026B,0x0},//,Base Configuration
{0x026C,0x2},//,Base Configuration
{0x026D,0x87},//,Base Configuration
{0x026E,0x2},//,Base Configuration
{0x026F,0x31},//,Base Configuration
{0x0270,0x3},//,Base Configuration
{0x0271,0xFF},//,Base Configuration
{0x0272,0x3},//,Base Configuration
{0x0273,0x0},//,Base Configuration
{0x0274,0x3},//,Base Configuration
{0x0275,0xFF},//,Base Configuration
{0x0276,0x2},//,Base Configuration
{0x0277,0x87},//,Base Configuration
{0x0278,0x3},//,Base Configuration
{0x0279,0x2},//,Base Configuration
{0x027A,0x3},//,Base Configuration
{0x027B,0x9},//,Base Configuration
{0x027C,0x3},//,Base Configuration
{0x027D,0xF7},//,Base Configuration
{0x027E,0x0},//,Base Configuration
{0x027F,0x16},//,Base Configuration
{0x0280,0x0},//,Base Configuration
{0x0281,0x33},//,Base Configuration
{0x0282,0x0},//,Base Configuration
{0x0283,0x4},//,Base Configuration
{0x0284,0x0},//,Base Configuration
{0x0285,0x11},//,Base Configuration
{0x0286,0x3},//,Base Configuration
{0x0287,0x9},//,Base Configuration
{0x0288,0x0},//,Base Configuration
{0x0289,0x2},//,Base Configuration
{0x028A,0x0},//,Base Configuration
{0x028B,0x20},//,Base Configuration
{0x028C,0x0},//,Base Configuration
{0x028D,0xB5},//,Base Configuration
{0x028E,0x1},//,Base Configuration
{0x028F,0x5},//,Base Configuration
{0x0290,0x0},//,Base Configuration
{0x0291,0x12},//,Base Configuration
{0x0292,0x0},//,Base Configuration
{0x0293,0xB5},//,Base Configuration
{0x0294,0x1},//,Base Configuration
{0x0295,0x5},//,Base Configuration
{0x0296,0x0},//,Base Configuration
{0x0297,0x0},//,Base Configuration
{0x0298,0x0},//,Base Configuration
{0x0299,0x12},//,Base Configuration
{0x029A,0x0},//,Base Configuration
{0x029B,0x12},//,Base Configuration
{0x029C,0x0},//,Base Configuration
{0x029D,0x20},//,Base Configuration
{0x029E,0x0},//,Base Configuration
{0x029F,0xB5},//,Base Configuration
{0x02A0,0x1},//,Base Configuration
{0x02A1,0x5},//,Base Configuration
{0x02A2,0x0},//,Base Configuration
{0x02A3,0x0},//,Base Configuration
{0x02A4,0x0},//,Base Configuration
{0x02A5,0x12},//,Base Configuration
{0x02A6,0x0},//,Base Configuration
{0x02A7,0x12},//,Base Configuration
{0x02A8,0x0},//,Base Configuration
{0x02A9,0x20},//,Base Configuration
{0x02AA,0x0},//,Base Configuration
{0x02AB,0x47},//,Base Configuration
{0x02AC,0x0},//,Base Configuration
{0x02AD,0x27},//,Base Configuration
{0x02AE,0x0},//,Base Configuration
{0x02AF,0xB5},//,Base Configuration
{0x02B0,0x0},//,Base Configuration
{0x02B1,0xE5},//,Base Configuration
{0x02B2,0x0},//,Base Configuration
{0x02B3,0x0},//,Base Configuration
{0x02B4,0x0},//,Base Configuration
{0x02B5,0x4},//,Base Configuration
{0x02B6,0x0},//,Base Configuration
{0x02B7,0x43},//,Base Configuration
{0x02B8,0x0},//,Base Configuration
{0x02B9,0x1},//,Base Configuration
{0x02BA,0x3},//,Base Configuration
{0x02BB,0x2},//,Base Configuration
{0x02BC,0x0},//,Base Configuration
{0x02BD,0x8},//,Base Configuration
{0x02BE,0x3},//,Base Configuration
{0x02BF,0xFF},//,Base Configuration
{0x02C0,0x2},//,Base Configuration
{0x02C1,0x87},//,Base Configuration
{0x02C2,0x3},//,Base Configuration
{0x02C3,0x89},//,Base Configuration
{0x02C4,0x3},//,Base Configuration
{0x02C5,0xF7},//,Base Configuration
{0x02C6,0x0},//,Base Configuration
{0x02C7,0x77},//,Base Configuration
{0x02C8,0x0},//,Base Configuration
{0x02C9,0x17},//,Base Configuration
{0x02CA,0x0},//,Base Configuration
{0x02CB,0x8},//,Base Configuration
{0x02CC,0x3},//,Base Configuration
{0x02CD,0xFF},//,Base Configuration
{0x02CE,0x0},//,Base Configuration
{0x02CF,0x38},//,Base Configuration
{0x02D0,0x0},//,Base Configuration
{0x02D1,0x17},//,Base Configuration
{0x02D2,0x0},//,Base Configuration
{0x02D3,0x8},//,Base Configuration
{0x02D4,0x3},//,Base Configuration
{0x02D5,0xFF},//,Base Configuration
{0x02D6,0x3},//,Base Configuration
{0x02D7,0xFF},//,Base Configuration
{0x02D8,0x3},//,Base Configuration
{0x02D9,0xFF},//,Base Configuration
{0x02DA,0x3},//,Base Configuration
{0x02DB,0xFF},//,Base Configuration
{0x02DC,0x3},//,Base Configuration
{0x02DD,0xFF},//,Base Configuration
{0x02DE,0x3},//,Base Configuration
{0x02DF,0xFF},//,Base Configuration
{0x02E0,0x3},//,Base Configuration
{0x02E1,0xFF},//,Base Configuration
{0x02E2,0x3},//,Base Configuration
{0x02E3,0xFF},//,Base Configuration
{0x02E4,0x3},//,Base Configuration
{0x02E5,0xFF},//,Base Configuration
{0x02E6,0x3},//,Base Configuration
{0x02E7,0xFF},//,Base Configuration
{0x02E8,0x3},//,Base Configuration
{0x02E9,0xFF},//,Base Configuration
{0x02EA,0x3},//,Base Configuration
{0x02EB,0xFF},//,Base Configuration
{0x02EC,0x3},//,Base Configuration
{0x02ED,0xFF},//,Base Configuration
{0x02EE,0x3},//,Base Configuration
{0x02EF,0xFF},//,Base Configuration
{0x02F0,0x3},//,Base Configuration
{0x02F1,0xFF},//,Base Configuration
{0x02F2,0x3},//,Base Configuration
{0x02F3,0xFF},//,Base Configuration
{0x02F4,0x3},//,Base Configuration
{0x02F5,0xFF},//,Base Configuration
{0x02F6,0x3},//,Base Configuration
{0x02F7,0xFF},//,Base Configuration
{0x02F8,0x3},//,Base Configuration
{0x02F9,0xFF},//,Base Configuration
{0x02FA,0x3},//,Base Configuration
{0x02FB,0xFF},//,Base Configuration
{0x02FC,0x3},//,Base Configuration
{0x02FD,0xFF},//,Base Configuration
{0x02FE,0x3},//,Base Configuration
{0x02FF,0xFF},//,Base Configuration
{0x0300,0x3},//,Base Configuration
{0x0301,0xFF},//,Base Configuration
{0x0302,0x3},//,Base Configuration
{0x0303,0xFF},//,Base Configuration
{0x01E9,0x0},//,Base Configuration
{0x01E8,0x19},//,Base Configuration
{0x01EA,0x35},//,Base Configuration
{0x01EB,0x37},//,Base Configuration
{0x01EC,0x5C},//,Base Configuration
{0x01ED,0x63},//,Base Configuration
{0x01F8,0xF},//,Base Configuration
{0x005C,0x0},//,Base Configuration
{0x005D,0x0},//,Base Configuration
{0x01DA,0x1},//,Base Configuration
{0x01DC,0x1},//,Base Configuration
{0x01DE,0x1},//,Base Configuration
{0x0189,0x1},//,Base Configuration
{0x01B7,0x1},//,Base Configuration
{0x01C1,0xE},//,Base Configuration
{0x01C2,0xF6},//,Base Configuration
{0x01C3,0xFF},//,Base Configuration
{0x01B8,0x1},//,Base Configuration
{0x01BA,0x32},//,Base Configuration
{0x01BD,0x8},//,Base Configuration
{0x01CA,0x1E},//,Base Configuration
{0x01C9,0x1E},//,Base Configuration
{0x01BF,0x3C},//,Base Configuration
{0x01C0,0x5C},//,Base Configuration
{0x0071,0x1},//,Base Configuration
{0x01B4,0x1},//,Base Configuration
{0x01B5,0x1},//,Base Configuration
{0x01F1,0x1},//,Base Configuration
{0x01F4,0x1},//,Base Configuration
{0x01F5,0x1},//,Base Configuration
{0x0314,0x1},//,Base Configuration
{0x0315,0x1},//,Base Configuration
{0x0316,0x1},//,Base Configuration
{0x0207,0x0},//,Base Configuration
{0x4207,0x2},//,Base Configuration
{0x2207,0x2},//,Base Configuration
{0x209D,0x0},//,Base Configuration
{0x0063,0x1},//,Base Configuration
{0x01F7,0xF},//,Base Configuration
{0x00E9,0x3},//,Base Configuration
{0x00EA,0x28},//,Base Configuration
{0x0309,0x7},//,Base Configuration
{0x030A,0x4},//,Base Configuration
{0x030B,0xD},//,Base Configuration
{0x030C,0x7},//,Base Configuration
{0x030E,0x15},//,Base Configuration
{0x030D,0xA},//,Base Configuration
{0x030F,0x1},//,Base Configuration
{0x0310,0xF},//,Base Configuration
{0x01D0,0x1F},//,Base Configuration
{0x01D1,0x12},//,Base Configuration
{0x0016,0x0},//,Base Configuration
{0x0017,0x5},//,Base Configuration
{0x00E8,0x3},//,Base Configuration
{0xE0C0,0x0},//,Base Configuration
{0xE0C1,0x8},//,Base Configuration
{0xE0C2,0x0},//,Base Configuration
{0xE0C3,0x8},//,Base Configuration
{0x016A,0x0},//,Base Configuration.DPATH_BITMODE
{0x0168,0x2A},//,Base Configuration.CSI2_DATA_TYPE
{0xE000,0x0},//,PLL.BANK_SEL
{0x2077,0x0},//,PLL.PLL_DIV_N
{0x2076,0xBD},//,PLL.PLL_DIV_M
{0x00CE,0x1},//,PLL.CLKGEN_CP_DIV
{0x0070,0x9},//,PLL.OTP_GRANULARITY
{0x016D,0x32},//,PLL.GRAN_TG
{0x0176,0x0},//,PLL.LUT_DEL_008
{0x20C6,0x0},//,PLL.CLKGEN_TX_ESC_H
{0x20C7,0x0},//,PLL.CLKGEN_TX_ESC_H
{0x20C8,0x1},//,PLL.CLKGEN_TX_ESC_H
{0x20C9,0x0},//,PLL.CLKGEN_TX_ESC_L
{0x20CA,0x0},//,PLL.CLKGEN_TX_ESC_L
{0x20CB,0x1},//,PLL.CLKGEN_TX_ESC_L
{0x2075,0x0},//,PLL.PLL_PD
{0xE000,0x0},//,Low Power State.BANK_SEL
{0x001E,0x1},//,Low Power State.EN_AUTO_LPS_TRANS
{0xE000,0x0},//,MIPI.BANK_SEL
{0x207E,0x0},//,MIPI.MIPI_VC_ID
{0x207F,0x0},//,MIPI.MIPI_FRAME_MODE_PIX
{0x2080,0x0},//,MIPI.MIPI_FRAME_MODE_EMBED
{0x2081,0x3},//,MIPI.MIPI_LINE_COUNT_EN
{0x2082,0x0},//,MIPI.MIPI_FRAME_COUNT_WRAP
{0x2083,0x2},//,MIPI.MIPI_FRAME_COUNT_WRAP
{0x0090,0x0},//,MIPI.CSI2_CONT_CLOCK_MODE
{0x2097,0x0},//,MIPI.CSI2_SCRAMBLING_EN
{0xE000,0x0},//,Sensor Control Mode.BANK_SEL
{0x0011,0x3},//,Sensor Control Mode.CTRL_MODE
{0x011D,0x0},//,Sensor Control Mode.SINGLE_PIN_CTRL
{0xE000,0x0},//,Time Bases.BANK_SEL
{0x0012,0x0},//,Time Bases.CLOCKS_PER_TIME_UNIT
{0x0013,0x18},//,Time Bases.CLOCKS_PER_TIME_UNIT
{0x015A,0x0},//,Time Bases.GLOB_TIME
{0x015B,0x33},//,Time Bases.GLOB_TIME
{0x015C,0x0},//,Time Bases.GLOB_TIME_A
{0x015D,0x33},//,Time Bases.GLOB_TIME_A
{0x015E,0x0},//,Time Bases.GLOB_TIME_B
{0x015F,0x33},//,Time Bases.GLOB_TIME_B
{0x0162,0x0},//,Time Bases.ENTER_LPS_TIME
{0x0163,0x5},//,Time Bases.ENTER_LPS_TIME
{0x0164,0x4},//,Time Bases.EXIT_LPS_TIME
{0x0165,0x4C},//,Time Bases.EXIT_LPS_TIME
{0x0166,0x4},//,Time Bases.LPS_CYCLE_TIME
{0x0167,0x4C},//,Time Bases.LPS_CYCLE_TIME
{0xE000,0x0},//,Analog Gain.BANK_SEL
{0x01BB,0xC8},//,Analog Gain
{0x01BC,0xC0},//,Analog Gain
{0x00D0,0x0},//,Analog Gain
{0x016E,0xBA},//,Analog Gain.LUT_DEL_000
{0x0172,0x0},//,Analog Gain.LUT_DEL_004
{0x0173,0x0},//,Analog Gain.LUT_DEL_005
{0x016F,0x7E},//,Analog Gain.LUT_DEL_001
{0x0170,0x0},//,Analog Gain.LUT_DEL_002
{0x0171,0xBA},//,Analog Gain.LUT_DEL_003
{0x0174,0x0},//,Analog Gain.LUT_DEL_006
{0x0175,0x20},//,Analog Gain.LUT_DEL_007
{0x018B,0x3},//,Analog Gain
{0x018C,0x2},//,Analog Gain
{0x018D,0x2},//,Analog Gain
{0x018E,0x56},//,Analog Gain
{0x018F,0x5},//,Analog Gain
{0x0190,0x7F},//,Analog Gain
{0x01EE,0x16},//,Analog Gain.SHUTTER_LAG
{0x01EF,0x6E},//,Analog Gain.SHUTTER_LAG
{0x01A2,0x4},//,Analog Gain.POS_ANACOL_TRIGGER
{0x01A3,0xD9},//,Analog Gain.POS_ANACOL_TRIGGER
{0x031F,0x4},//,Analog Gain.POS_YADDR_TRIGGER
{0x0320,0xE2},//,Analog Gain.POS_YADDR_TRIGGER
{0x01A6,0x5},//,Analog Gain.POS_ADC_TRIGGER
{0x01A7,0x70},//,Analog Gain.POS_ADC_TRIGGER
{0x01A4,0x8},//,Analog Gain.POS_ANACOL_YBIN_TRIGGER
{0x01A5,0x9A},//,Analog Gain.POS_ANACOL_YBIN_TRIGGER
{0x0321,0x8},//,Analog Gain.POS_YADDR_YBIN_TRIGGER
{0x0322,0xA3},//,Analog Gain.POS_YADDR_YBIN_TRIGGER
{0x01A8,0x9},//,Analog Gain.POS_ADC_YBIN_TRIGGER
{0x01A9,0x31},//,Analog Gain.POS_ADC_YBIN_TRIGGER
{0x01A0,0x0},//,Analog Gain.POS_VIS_TRIGGER
{0x01A1,0xFF},//,Analog Gain.POS_VIS_TRIGGER
{0x01B2,0x1},//,Analog Gain.POS_HSYNC_RISE
{0x01B3,0x17},//,Analog Gain.POS_HSYNC_RISE
{0x01B0,0x1},//,Analog Gain.POS_STAT_TRIGGER
{0x01B1,0x12},//,Analog Gain.POS_STAT_TRIGGER
{0x01AC,0x1},//,Analog Gain.POS_SVAL_TRIGGER
{0x01AD,0x1D},//,Analog Gain.POS_SVAL_TRIGGER
{0x01F0,0x24},//,Analog Gain
{0x01F3,0x1},//,Analog Gain
{0xE195,0xF},//,Analog Gain
{0xE000,0x0},//,Black Level.BANK_SEL
{0x0193,0xF},//,Black Level.OFFSET_CLIPPING
{0x0194,0xA8},//,Black Level.OFFSET_CLIPPING
{0xE000,0x0},//,Release Soft Reset.BANK_SEL
{0xE009,0x1},//,Release Soft Reset
{0x212F,0x1},//,Release Soft Reset
{0x2130,0x1},//,Release Soft Reset
{0x2131,0x1},//,Release Soft Reset
{0x2132,0x1},//,Release Soft Reset
{0x2133,0x1},//,Release Soft Reset
{0x2134,0x1},//,Release Soft Reset
{0x2135,0x1},//,Release Soft Reset
{0xE0E1,0x1},//,Release Soft Reset
{0x018A,0x1},//,Release Soft Reset
{0x00E0,0x1},//,Release Soft Reset
{0xE004,0x0},//,Horizontal ROI.RW_CONTEXT
{0xE000,0x1},//,Horizontal ROI.BANK_SEL
{0xE02C,0x0},//,Horizontal ROI.XWIN_LEFT
{0xE02D,0xC},//,Horizontal ROI.XWIN_LEFT
{0xE02E,0x2},//,Horizontal ROI.XWIN_RIGHT
{0xE02F,0x4B},//,Horizontal ROI.XWIN_RIGHT
{0xE030,0x0},//,Horizontal ROI.XMIRROR
{0xE025,0x0},//,Horizontal ROI.XSUBSAMPLING
{0xE02A,0x0},//,Horizontal ROI.XBINNING
{0x2029,0x46},//,Horizontal ROI.MIPI_DATA_FIFO_THRESHOLD
{0x0034,0x1},//,Horizontal ROI.HSYNC_LENGTH
{0x0035,0x20},//,Horizontal ROI.HSYNC_LENGTH
{0xE004,0x1},//,Horizontal ROI.RW_CONTEXT
{0xE02C,0x0},//,Horizontal ROI.XWIN_LEFT
{0xE02D,0x0},//,Horizontal ROI.XWIN_LEFT
{0xE02E,0x2},//,Horizontal ROI.XWIN_RIGHT
{0xE02F,0x57},//,Horizontal ROI.XWIN_RIGHT
{0xE030,0x0},//,Horizontal ROI.XMIRROR
{0xE025,0x0},//,Horizontal ROI.XSUBSAMPLING
{0xE02A,0x0},//,Horizontal ROI.XBINNING
{0x2029,0x46},//,Horizontal ROI.MIPI_DATA_FIFO_THRESHOLD
{0x0034,0x1},//,Horizontal ROI.HSYNC_LENGTH
{0x0035,0x2C},//,Horizontal ROI.HSYNC_LENGTH
{0xE004,0x0},//,Vertical ROI.RW_CONTEXT
{0xE000,0x1},//,Vertical ROI.BANK_SEL
{0x001E,0x0},//,Vertical ROI.YWIN_ENA
{0x001F,0x1},//,Vertical ROI.YWIN_ENA
{0x002B,0x0},//,Vertical ROI.YBINNING
{0xE004,0x1},//,Vertical ROI.RW_CONTEXT
{0x001E,0x0},//,Vertical ROI.YWIN_ENA
{0x001F,0x1},//,Vertical ROI.YWIN_ENA
{0x002B,0x0},//,Vertical ROI.YBINNING
{0xE000,0x0},//,Vertical ROI.BANK_SEL
{0x001F,0x0},//,Vertical ROI.YWIN_BLACK
{0x0020,0x0},//,Vertical ROI.YWIN_BLACK
{0x0023,0x0},//,Vertical ROI.YWIN_DIR
{0x0024,0x3},//,Vertical ROI.YWIN0_SIZE
{0x0025,0x0},//,Vertical ROI.YWIN0_SIZE
{0x0026,0x0},//,Vertical ROI.YWIN0_START
{0x0027,0x18},//,Vertical ROI.YWIN0_START
{0x0028,0x0},//,Vertical ROI.YWIN0_SUBS
{0x0029,0x0},//,Vertical ROI.YWIN1_SIZE
{0x002A,0x0},//,Vertical ROI.YWIN1_SIZE
{0x002B,0x0},//,Vertical ROI.YWIN1_START
{0x002C,0x0},//,Vertical ROI.YWIN1_START
{0x002D,0x0},//,Vertical ROI.YWIN1_SUBS
{0x002E,0x0},//,Vertical ROI.YWIN2_SIZE
{0x002F,0x0},//,Vertical ROI.YWIN2_SIZE
{0x0030,0x0},//,Vertical ROI.YWIN2_START
{0x0031,0x0},//,Vertical ROI.YWIN2_START
{0x0032,0x0},//,Vertical ROI.YWIN2_SUBS
{0x0033,0x0},//,Vertical ROI.YWIN3_SIZE
{0x0034,0x0},//,Vertical ROI.YWIN3_SIZE
{0x0035,0x0},//,Vertical ROI.YWIN3_START
{0x0036,0x0},//,Vertical ROI.YWIN3_START
{0x0037,0x0},//,Vertical ROI.YWIN3_SUBS
{0x0038,0x0},//,Vertical ROI.YWIN4_SIZE
{0x0039,0x0},//,Vertical ROI.YWIN4_SIZE
{0x003A,0x0},//,Vertical ROI.YWIN4_START
{0x003B,0x0},//,Vertical ROI.YWIN4_START
{0x003C,0x0},//,Vertical ROI.YWIN4_SUBS
{0x003D,0x0},//,Vertical ROI.YWIN5_SIZE
{0x003E,0x0},//,Vertical ROI.YWIN5_SIZE
{0x003F,0x0},//,Vertical ROI.YWIN5_START
{0x0040,0x0},//,Vertical ROI.YWIN5_START
{0x0041,0x0},//,Vertical ROI.YWIN5_SUBS
{0x0042,0x0},//,Vertical ROI.YWIN6_SIZE
{0x0043,0x0},//,Vertical ROI.YWIN6_SIZE
{0x0044,0x0},//,Vertical ROI.YWIN6_START
{0x0045,0x0},//,Vertical ROI.YWIN6_START
{0x0046,0x0},//,Vertical ROI.YWIN6_SUBS
{0x0047,0x0},//,Vertical ROI.YWIN7_SIZE
{0x0048,0x0},//,Vertical ROI.YWIN7_SIZE
{0x0049,0x0},//,Vertical ROI.YWIN7_START
{0x004A,0x0},//,Vertical ROI.YWIN7_START
{0x004B,0x0},//,Vertical ROI.YWIN7_SUBS
{0x004C,0x0},//,Vertical ROI.YWIN8_SIZE
{0x004D,0x0},//,Vertical ROI.YWIN8_SIZE
{0x004E,0x0},//,Vertical ROI.YWIN8_START
{0x004F,0x0},//,Vertical ROI.YWIN8_START
{0x0050,0x0},//,Vertical ROI.YWIN8_SUBS
{0x0051,0x0},//,Vertical ROI.YWIN9_SIZE
{0x0052,0x0},//,Vertical ROI.YWIN9_SIZE
{0x0053,0x0},//,Vertical ROI.YWIN9_START
{0x0054,0x0},//,Vertical ROI.YWIN9_START
{0x0055,0x0},//,Vertical ROI.YWIN9_SUBS
{0xE004,0x0},//,Frame && Exposure Control.RW_CONTEXT
{0xE000,0x1},//,Frame && Exposure Control.BANK_SEL
{0x000E,0x0},//,Frame && Exposure Control.EXP_TIME_L
{0x000F,0x0},//,Frame && Exposure Control.EXP_TIME_L
{0x0010,0x3},//,Frame && Exposure Control.EXP_TIME_L
{0x0011,0xE8},//,Frame && Exposure Control.EXP_TIME_L
{0x0012,0x0},//,Frame && Exposure Control.EXP_TIME_S
{0x0013,0x0},//,Frame && Exposure Control.EXP_TIME_S
{0x0014,0x0},//,Frame && Exposure Control.EXP_TIME_S
{0x0015,0x0},//,Frame && Exposure Control.EXP_TIME_S
{0xE004,0x1},//,Frame && Exposure Control.RW_CONTEXT
{0x000E,0x0},//,Frame && Exposure Control.EXP_TIME_L
{0x000F,0x0},//,Frame && Exposure Control.EXP_TIME_L
{0x0010,0x3},//,Frame && Exposure Control.EXP_TIME_L
{0x0011,0xE8},//,Frame && Exposure Control.EXP_TIME_L
{0x0012,0x0},//,Frame && Exposure Control.EXP_TIME_S
{0x0013,0x0},//,Frame && Exposure Control.EXP_TIME_S
{0x0014,0x0},//,Frame && Exposure Control.EXP_TIME_S
{0x0015,0x0},//,Frame && Exposure Control.EXP_TIME_S
{0xE004,0x0},//,Frame && Exposure Control.RW_CONTEXT
{0x0032,0x6},//,Frame && Exposure Control.ROW_LENGTH
{0x0033,0xE2},//,Frame && Exposure Control.ROW_LENGTH
{0xE004,0x1},//,Frame && Exposure Control.RW_CONTEXT
{0x0032,0x6},//,Frame && Exposure Control.ROW_LENGTH
{0x0033,0xE2},//,Frame && Exposure Control.ROW_LENGTH
{0xE004,0x0},//,Frame && Exposure Control.RW_CONTEXT
{0x0007,0x1},//,Frame && Exposure Control.NROF_FRAMES
{0x0008,0x0},//,Frame && Exposure Control.TARGET_FRAME_TIME
{0x0009,0x0},//,Frame && Exposure Control.TARGET_FRAME_TIME
{0x000A,0x41},//,Frame && Exposure Control.TARGET_FRAME_TIME
{0x000B,0x1B},//,Frame && Exposure Control.TARGET_FRAME_TIME
{0xE004,0x1},//,Frame && Exposure Control.RW_CONTEXT
{0x0007,0x1},//,Frame && Exposure Control.NROF_FRAMES
{0x0008,0x0},//,Frame && Exposure Control.TARGET_FRAME_TIME
{0x0009,0x0},//,Frame && Exposure Control.TARGET_FRAME_TIME
{0x000A,0x41},//,Frame && Exposure Control.TARGET_FRAME_TIME
{0x000B,0x1B},//,Frame && Exposure Control.TARGET_FRAME_TIME
{0xE004,0x0},//,Digital Gain.RW_CONTEXT
{0xE000,0x1},//,Digital Gain.BANK_SEL
{0x0024,0xF},//,Digital Gain.GDIG_AMP
{0xE004,0x1},//,Digital Gain.RW_CONTEXT
{0x0024,0xF},//,Digital Gain.GDIG_AMP
{0xE000,0x0},//,Defect Pixel Correction.BANK_SEL
{0x0057,0x0},//,Defect Pixel Correction.DEFECT_ON
{0x0058,0x0},//,Defect Pixel Correction.DEFECT_MODE
{0x0059,0x2},//,Defect Pixel Correction.DEFECT_LIMIT_HIGH
{0x005A,0x2},//,Defect Pixel Correction.DEFECT_LIMIT_LOW
{0x005B,0x0},//,Defect Pixel Correction.DEFECT_LIMIT_HIGH_MODE
{0xE000,0x0},//,Context Switching.BANK_SEL
{0xE008,0x0},//,Context Switching.DISABLE_CONTEXTSYNC
{0x0006,0x1},//,Context Switching.PARAM_HOLD
{0xE003,0x0},//,Context Switching.NEXT_ACTIVE_CONTEXT
{0x0006,0x0},//,Context Switching.PARAM_HOLD
{0xE008,0x0},//,Context Switching.DISABLE_CONTEXTSYNC
{0xE004,0x0},//,Event Detection.RW_CONTEXT
{0xE000,0x1},//,Event Detection.BANK_SEL
{0x0031,0x0},//,Event Detection.EN_EVENT_DETECTION
{0xE004,0x1},//,Event Detection.RW_CONTEXT
{0x0031,0x0},//,Event Detection.EN_EVENT_DETECTION
{0xE000,0x0},//,Event Detection.BANK_SEL
{0x0138,0x0},//,Event Detection.EN_AUTO_CNTXT_SWITCH
{0xE005,0x0},//,Event Detection.AUTO_SWITCHTO_CNTXT
{0x0139,0x0},//,Event Detection.TILE_XSTART
{0x013A,0x0},//,Event Detection.TILE_XSTART
{0x013B,0x96},//,Event Detection.TILE_WIDTH
{0x013C,0x0},//,Event Detection.TILE_YSTART
{0x013D,0x0},//,Event Detection.TILE_YSTART
{0x013E,0xA0},//,Event Detection.TILE_HEIGHT
{0x013F,0x6},//,Event Detection.TILE_DISCARD_NR_BITS
{0x0140,0x1},//,Event Detection.MIN_FLAGGED_TILES
{0x0141,0x14},//,Event Detection.MAX_FLAGGED_TILES
{0x0142,0x1},//,Event Detection.TILE_THRESHOLD
{0x0143,0x0},//,Event Detection.TILE_MIN_THRESHOLD
{0x0144,0x0},//,Event Detection.TILE_MIN_THRESHOLD
{0x0145,0x0},//,Event Detection.TILE_DISABLE_MASK
{0x0146,0x0},//,Event Detection.TILE_DISABLE_MASK
{0x0147,0x0},//,Event Detection.TILE_DISABLE_MASK
{0x0148,0x0},//,Event Detection.CPY_TILES_INTERVAL
{0xE004,0x0},//,Image Statistics.RW_CONTEXT
{0xE000,0x1},//,Image Statistics.BANK_SEL
{0x0026,0x0},//,Image Statistics.STAT_NROF_HIST
{0xE004,0x1},//,Image Statistics.RW_CONTEXT
{0x0026,0x0},//,Image Statistics.STAT_NROF_HIST
{0xE000,0x0},//,Image Statistics.BANK_SEL
{0x0169,0x12},//,Image Statistics.CSI2_EMBED_TYPE
{0xE004,0x0},//,Illumination Trigger.RW_CONTEXT
{0xE000,0x1},//,Illumination Trigger.BANK_SEL
{0x001C,0x0},//,Illumination Trigger.EN_TRIG_ILLUM
{0x001D,0x0},//,Illumination Trigger.EN_TRIG_SYNC
{0x0019,0x0},//,Illumination Trigger.ILLUM_WIDTH
{0x001A,0x7},//,Illumination Trigger.ILLUM_WIDTH
{0x001B,0x53},//,Illumination Trigger.ILLUM_WIDTH
{0x0016,0x8},//,Illumination Trigger.ILLUM_DELAY
{0x0017,0x0},//,Illumination Trigger.ILLUM_DELAY
{0x0018,0x0},//,Illumination Trigger.ILLUM_DELAY
{0xE004,0x1},//,Illumination Trigger.RW_CONTEXT
{0x001C,0x0},//,Illumination Trigger.EN_TRIG_ILLUM
{0x001D,0x0},//,Illumination Trigger.EN_TRIG_SYNC
{0x0019,0x0},//,Illumination Trigger.ILLUM_WIDTH
{0x001A,0x7},//,Illumination Trigger.ILLUM_WIDTH
{0x001B,0x53},//,Illumination Trigger.ILLUM_WIDTH
{0x0016,0x8},//,Illumination Trigger.ILLUM_DELAY
{0x0017,0x0},//,Illumination Trigger.ILLUM_DELAY
{0x0018,0x0},//,Illumination Trigger.ILLUM_DELAY
{0xE000,0x0},//,Illumination Trigger.BANK_SEL
{0x001A,0x0},//,Illumination Trigger.TRIG_SYNC_DELAY
{0x001B,0x0},//,Illumination Trigger.TRIG_SYNC_DELAY
{0x001C,0x0},//,Illumination Trigger.TRIG_SYNC_DELAY
{0x00F3,0x27},//,Illumination Trigger.DMUX0_SEL
{0xE004,0x0},//,Synchronization Trigger.RW_CONTEXT
{0xE000,0x1},//,Synchronization Trigger.BANK_SEL
{0x001D,0x0},//,Synchronization Trigger.EN_TRIG_SYNC
{0xE004,0x1},//,Synchronization Trigger.RW_CONTEXT
{0x001D,0x0},//,Synchronization Trigger.EN_TRIG_SYNC
{0xE000,0x0},//,Synchronization Trigger.BANK_SEL
{0x001A,0x0},//,Synchronization Trigger.TRIG_SYNC_DELAY
{0x001B,0x0},//,Synchronization Trigger.TRIG_SYNC_DELAY
{0x001C,0x0},//,Synchronization Trigger.TRIG_SYNC_DELAY
{0x001D,0x0},//,Synchronization Trigger.TRIG_SYNC_ON_REQ_1


};

static const struct mira050_reg full_576_768_50fps_8b_1lane_reg_post_soft_reset[] = {

};

static const struct mira050_reg partial_analog_gain_x1_12bit[] = {
	{0xE000,0x0}, //,Analog Gain.BANK_SEL
	{0x01BB,0xC8}, //,Analog Gain
	{0x01BC,0xC0}, //,Analog Gain
	{0x00D0,0x0}, //,Analog Gain
	{0x01F0,0x8}, //,Analog Gain
	{0x01F3,0x2}, //,Analog Gain
	{0x016E,0xCE}, //,Analog Gain.LUT_DEL_000
	{0x0172,0x0}, //,Analog Gain.LUT_DEL_004
	{0x0173,0x0}, //,Analog Gain.LUT_DEL_005
	{0x016F,0xFF}, //,Analog Gain.LUT_DEL_001
	{0x0170,0xFF}, //,Analog Gain.LUT_DEL_002
	{0x0171,0xCE}, //,Analog Gain.LUT_DEL_003
	{0x0174,0x0}, //,Analog Gain.LUT_DEL_006
	{0x0175,0x20}, //,Analog Gain.LUT_DEL_007
	{0x018B,0x3}, //,Analog Gain
	{0x018C,0x52}, //,Analog Gain
	{0x018D,0x2}, //,Analog Gain
	{0x018E,0x56}, //,Analog Gain
	{0x018F,0xB}, //,Analog Gain
	{0x0190,0xCF}, //,Analog Gain
	{0x01EE,0x15}, //,Analog Gain.SHUTTER_LAG
	{0x01EF,0x6A}, //,Analog Gain.SHUTTER_LAG
	{0x01A2,0x5}, //,Analog Gain.POS_ANACOL_TRIGGER
	{0x01A3,0xDD}, //,Analog Gain.POS_ANACOL_TRIGGER
	{0x031F,0x5}, //,Analog Gain.POS_YADDR_TRIGGER
	{0x0320,0xE6}, //,Analog Gain.POS_YADDR_TRIGGER
	{0x01A6,0x6}, //,Analog Gain.POS_ADC_TRIGGER
	{0x01A7,0x74}, //,Analog Gain.POS_ADC_TRIGGER
	{0x01A4,0xB}, //,Analog Gain.POS_ANACOL_YBIN_TRIGGER
	{0x01A5,0x46}, //,Analog Gain.POS_ANACOL_YBIN_TRIGGER
	{0x0321,0xB}, //,Analog Gain.POS_YADDR_YBIN_TRIGGER
	{0x0322,0x4F}, //,Analog Gain.POS_YADDR_YBIN_TRIGGER
	{0x01A8,0xB}, //,Analog Gain.POS_ADC_YBIN_TRIGGER
	{0x01A9,0xDD}, //,Analog Gain.POS_ADC_YBIN_TRIGGER
	{0x01A0,0x0}, //,Analog Gain.POS_VIS_TRIGGER
	{0x01A1,0xB1}, //,Analog Gain.POS_VIS_TRIGGER
	{0x01B2,0x0}, //,Analog Gain.POS_HSYNC_RISE
	{0x01B3,0xC9}, //,Analog Gain.POS_HSYNC_RISE
	{0x01B0,0x0}, //,Analog Gain.POS_STAT_TRIGGER
	{0x01B1,0xC4}, //,Analog Gain.POS_STAT_TRIGGER
	{0x01AC,0x0}, //,Analog Gain.POS_SVAL_TRIGGER
	{0x01AD,0xCF}, //,Analog Gain.POS_SVAL_TRIGGER

};

static const struct mira050_reg partial_analog_gain_x2_12bit[] = {
	{0xE000,0x0},//,Analog Gain.BANK_SEL
	{0x01BB,0xAF},//,Analog Gain
	{0x01BC,0xA7},//,Analog Gain
	{0x00D0,0x0},//,Analog Gain
	{0x01F0,0x8},//,Analog Gain
	{0x01F3,0x1},//,Analog Gain
	{0x016E,0xFF},//,Analog Gain.LUT_DEL_000
	{0x0172,0x4D},//,Analog Gain.LUT_DEL_004
	{0x0173,0x0},//,Analog Gain.LUT_DEL_005
	{0x016F,0xFF},//,Analog Gain.LUT_DEL_001
	{0x0170,0xFF},//,Analog Gain.LUT_DEL_002
	{0x0171,0xFF},//,Analog Gain.LUT_DEL_003
	{0x0174,0x4D},//,Analog Gain.LUT_DEL_006
	{0x0175,0x3F},//,Analog Gain.LUT_DEL_007
	{0x018B,0x5},//,Analog Gain
	{0x018C,0x4A},//,Analog Gain
	{0x018D,0x2},//,Analog Gain
	{0x018E,0x56},//,Analog Gain
	{0x018F,0xE},//,Analog Gain
	{0x0190,0x44},//,Analog Gain
	{0x01EE,0x15},//,Analog Gain.SHUTTER_LAG
	{0x01EF,0x6},//,Analog Gain.SHUTTER_LAG
	{0x01A2,0x6},//,Analog Gain.POS_ANACOL_TRIGGER
	{0x01A3,0x41},//,Analog Gain.POS_ANACOL_TRIGGER
	{0x031F,0x6},//,Analog Gain.POS_YADDR_TRIGGER
	{0x0320,0x4A},//,Analog Gain.POS_YADDR_TRIGGER
	{0x01A6,0x6},//,Analog Gain.POS_ADC_TRIGGER
	{0x01A7,0xD8},//,Analog Gain.POS_ADC_TRIGGER
	{0x01A4,0xC},//,Analog Gain.POS_ANACOL_YBIN_TRIGGER
	{0x01A5,0xC5},//,Analog Gain.POS_ANACOL_YBIN_TRIGGER
	{0x0321,0xC},//,Analog Gain.POS_YADDR_YBIN_TRIGGER
	{0x0322,0xCE},//,Analog Gain.POS_YADDR_YBIN_TRIGGER
	{0x01A8,0xD},//,Analog Gain.POS_ADC_YBIN_TRIGGER
	{0x01A9,0x5C},//,Analog Gain.POS_ADC_YBIN_TRIGGER
	{0x01A0,0x0},//,Analog Gain.POS_VIS_TRIGGER
	{0x01A1,0xDB},//,Analog Gain.POS_VIS_TRIGGER
	{0x01B2,0x0},//,Analog Gain.POS_HSYNC_RISE
	{0x01B3,0xF3},//,Analog Gain.POS_HSYNC_RISE
	{0x01B0,0x0},//,Analog Gain.POS_STAT_TRIGGER
	{0x01B1,0xEE},//,Analog Gain.POS_STAT_TRIGGER
	{0x01AC,0x0},//,Analog Gain.POS_SVAL_TRIGGER
	{0x01AD,0xF9},//,Analog Gain.POS_SVAL_TRIGGER

};

static const struct mira050_reg partial_analog_gain_x4_12bit[] = {
	// # Analog Gain
	{0xE000, 0x00},
	{0x01BB, 0x99},
	{0x01BC, 0x91},
	{0x00D0, 0x00},
	{0x01F0, 0x08},
	{0x01F3, 0x00},
	{0x016E, 0xFF},
	{0x0172, 0xFF},
	{0x0173, 0x2E},
	{0x016F, 0xFF},
	{0x0170, 0xFF},
	{0x0171, 0xFF},
	{0x0174, 0xFF},
	{0x0175, 0xAB},
	{0x018B, 0x08},
	{0x018C, 0xCA},
	{0x018D, 0x02},
	{0x018E, 0x56},
	{0x018F, 0x12},
	{0x0190, 0xBE},
	{0x01EE, 0x14},
	{0x01EF, 0xA2},
	{0x01A2, 0x06},
	{0x01A3, 0xA5},
	{0x031F, 0x06},
	{0x0320, 0xAE},
	{0x01A6, 0x07},
	{0x01A7, 0x3C},
	{0x01A4, 0x0F},
	{0x01A5, 0x27},
	{0x0321, 0x0F},
	{0x0322, 0x30},
	{0x01A8, 0x0F},
	{0x01A9, 0xBE},
	{0x01A0, 0x01},
	{0x01A1, 0x25},
	{0x01B2, 0x01},
	{0x01B3, 0x3D},
	{0x01B0, 0x01},
	{0x01B1, 0x38},
	{0x01AC, 0x01},
	{0x01AD, 0x43},
	// # Black Level
	{0xE000, 0x00},
	{0x0193, 0x34},
	{0x0194, 0x5E},

};

static const struct mira050_fine_gain_lut_new fine_gain_lut_10bit_hs_4x[] = {
	{256, 15, 36, 3},
	{261, 15, 35, 3},
	{270, 15, 33, 3},
	{275, 15, 32, 3},
	{286, 15, 30, 3},
	{292, 15, 29, 3},
	{304, 15, 27, 3},
	{310, 15, 26, 3},
	{324, 15, 24, 3},
	{332, 15, 23, 3},
	{339, 15, 22, 3},
	{352, 15, 62, 2},
	{365, 15, 59, 2},
	{374, 15, 57, 2},
	{384, 15, 55, 2},
	{394, 15, 53, 2},
	{405, 15, 51, 2},
	{423, 15, 48, 2},
	{436, 15, 46, 2},
	{442, 15, 45, 2},
	{456, 15, 43, 2},
	{471, 15, 41, 2},
	{486, 15, 39, 2},
	{503, 15, 37, 2},
	{512, 15, 36, 2},
	{531, 15, 34, 2},
	{551, 15, 32, 2},
	{561, 15, 31, 2},
	{584, 15, 29, 2},
	{596, 15, 28, 2},
	{621, 15, 26, 2},
	{634, 15, 25, 2},
	{649, 15, 24, 2},
	{679, 15, 22, 2},
	{695, 15, 63, 1},
	{712, 15, 61, 1},
	{739, 15, 58, 1},
	{758, 15, 56, 1},
	{778, 15, 54, 1},
	{811, 15, 51, 1},
	{834, 15, 49, 1},
	{858, 15, 47, 1},
	{884, 15, 45, 1},
	{912, 15, 43, 1},
	{926, 15, 42, 1},
	{957, 15, 40, 1},
	{989, 15, 38, 1},
	{1024, 15, 36, 1},

};
static const struct mira050_fine_gain_lut_new fine_gain_lut_8bit_16x[] = {
	// gain_256,gdig_preamp,rg_bias,rg_mult
	{256, 3, 36, 3},
	{261, 3, 35, 3},
	{270, 3, 33, 3},
	{275, 3, 32, 3},
	{286, 3, 30, 3},
	{292, 3, 29, 3},
	{304, 3, 27, 3},
	{310, 3, 26, 3},
	{324, 3, 24, 3},
	{332, 3, 23, 3},
	{339, 3, 22, 3},
	{352, 3, 62, 2},
	{365, 3, 59, 2},
	{374, 3, 57, 2},
	{384, 3, 55, 2},
	{394, 3, 53, 2},
	{405, 3, 51, 2},
	{423, 3, 48, 2},
	{436, 3, 46, 2},
	{442, 3, 45, 2},
	{456, 3, 43, 2},
	{471, 3, 41, 2},
	{486, 3, 39, 2},
	{503, 3, 37, 2},
	{512, 3, 36, 2},
	{531, 3, 34, 2},
	{551, 3, 32, 2},
	{561, 3, 31, 2},
	{584, 3, 29, 2},
	{596, 3, 28, 2},
	{621, 3, 26, 2},
	{634, 3, 25, 2},
	{649, 3, 24, 2},
	{679, 3, 22, 2},
	{695, 3, 63, 1},
	{712, 3, 61, 1},
	{739, 3, 58, 1},
	{758, 3, 56, 1},
	{778, 3, 54, 1},
	{811, 3, 51, 1},
	{834, 3, 49, 1},
	{858, 3, 47, 1},
	{884, 3, 45, 1},
	{912, 3, 43, 1},
	{926, 3, 42, 1},
	{957, 3, 40, 1},
	{989, 3, 38, 1},
	{1024, 7, 36, 2},
	{1042, 7, 35, 2},
	{1081, 7, 33, 2},
	{1101, 7, 32, 2},
	{1144, 7, 30, 2},
	{1167, 7, 29, 2},
	{1216, 7, 27, 2},
	{1242, 7, 26, 2},
	{1297, 7, 24, 2},
	{1327, 7, 23, 2},
	{1357, 7, 22, 2},
	{1406, 7, 62, 1},
	{1459, 7, 59, 1},
	{1497, 7, 57, 1},
	{1536, 7, 55, 1},
	{1599, 7, 52, 1},
	{1644, 7, 50, 1},
	{1692, 7, 48, 1},
	{1742, 7, 46, 1},
	{1796, 7, 44, 1},
	{1853, 7, 42, 1},
	{1883, 7, 41, 1},
	{1946, 7, 39, 1},
	{2013, 7, 37, 1},
	{2085, 15, 35, 2},
	{2122, 15, 34, 2},
	{2203, 15, 32, 2},
	{2245, 15, 31, 2},
	{2335, 15, 29, 2},
	{2382, 15, 28, 2},
	{2484, 15, 26, 2},
	{2538, 15, 25, 2},
	{2594, 15, 24, 2},
	{2715, 15, 22, 2},
	{2779, 15, 63, 1},
	{2882, 15, 60, 1},
	{2955, 15, 58, 1},
	{3032, 15, 56, 1},
	{3155, 15, 53, 1},
	{3243, 15, 51, 1},
	{3335, 15, 49, 1},
	{3433, 15, 47, 1},
	{3537, 15, 45, 1},
	{3648, 15, 43, 1},
	{3766, 15, 41, 1},
	{3827, 15, 40, 1},
	{3957, 15, 38, 1},
	{4096, 15, 36, 1},

};

static const char *const mira050_test_pattern_menu[] = {
	"Disabled",
	"Fixed Data",
	"2D Gradient",
};

static const int mira050_test_pattern_val[] = {
	MIRA050_TEST_PATTERN_DISABLE,
	MIRA050_TEST_PATTERN_FIXED_DATA,
	MIRA050_TEST_PATTERN_2D_GRADIENT,
};

/* regulator supplies */
static const char *const mira050_supply_name[] = {
	// TODO(jalv): Check supply names
	/* Supplies can be enabled in any order */
	"VANA", /* Analog (2.8V) supply */
	"VDIG", /* Digital Core (1.8V) supply */
	"VDDL", /* IF (1.2V) supply */
};

#define MIRA050_NUM_SUPPLIES ARRAY_SIZE(mira050_supply_name)

/*
 * The supported formats. All flip/mirror combinations have the same byte order because the sensor
 * is monochrome
 */
static const u32 codes[] = {
	// MEDIA_BUS_FMT_Y8_1X8,
	// MEDIA_BUS_FMT_Y10_1X10,
	// MEDIA_BUS_FMT_Y12_1X12,
	MEDIA_BUS_FMT_SGRBG8_1X8,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGRBG12_1X12,
};

/* Mode configs */
/*
 * Only one mode is exposed to the public (576x768 at 12 bit).
 * Three codes (8/10/12 bit) are exposed to public.
 * The public user specifies the code.
 * That is used to specify which internal supported_mode to use.
 */
#define MIRA050_SUPPORTED_MODE_SIZE_PUBLIC 1
static const struct mira050_mode supported_modes[] = {
	{
		/* 12 bit mode */
		.width = 576,
		.height = 768,
		.crop = {
			.left = MIRA050_PIXEL_ARRAY_LEFT,
			.top = MIRA050_PIXEL_ARRAY_TOP,
			.width = 576,
			.height = 768},
		.reg_list_pre_soft_reset = {
			.num_of_regs = ARRAY_SIZE(full_576_768_50fps_12b_1lane_reg_pre_soft_reset),
			.regs = full_576_768_50fps_12b_1lane_reg_pre_soft_reset,
		},
		.reg_list_post_soft_reset = {
			.num_of_regs = ARRAY_SIZE(full_576_768_50fps_12b_1lane_reg_post_soft_reset),
			.regs = full_576_768_50fps_12b_1lane_reg_post_soft_reset,
		},
		.min_vblank = MIRA050_MIN_VBLANK_60,
		.max_vblank = MIRA050_MAX_VBLANK,

		.hblank = 0,
		.bit_depth = 12,
		.code = MEDIA_BUS_FMT_SGRBG12_1X12,
		.gain_min = 0,
		.gain_max = 2, // this is means 0,1,2 correspond to 1x 2x 4x gain
	},
	{
		/* 10 bit highspeed / low power mode */
		.width = 576,
		.height = 768,
		.crop = {.left = MIRA050_PIXEL_ARRAY_LEFT, .top = MIRA050_PIXEL_ARRAY_TOP, .width = 576, .height = 768},
		.reg_list_pre_soft_reset = {
			.num_of_regs = ARRAY_SIZE(full_576_768_50fps_10b_hs_1lane_reg_pre_soft_reset),
			.regs = full_576_768_50fps_10b_hs_1lane_reg_pre_soft_reset,
		},
		.reg_list_post_soft_reset = {
			.num_of_regs = ARRAY_SIZE(full_576_768_50fps_10b_hs_1lane_reg_post_soft_reset),
			.regs = full_576_768_50fps_10b_hs_1lane_reg_post_soft_reset,
		},
		.min_vblank = MIRA050_MIN_VBLANK_120,
		.max_vblank = MIRA050_MAX_VBLANK,
		.hblank = 0,
		.bit_depth = 10,
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.gain_min = 0,
		.gain_max = ARRAY_SIZE(fine_gain_lut_10bit_hs_4x) - 1,
	},
	{
		/* 8 bit mode */
		.width = 576,
		.height = 768,
		.crop = {.left = MIRA050_PIXEL_ARRAY_LEFT, .top = MIRA050_PIXEL_ARRAY_TOP, .width = 576, .height = 768},
		.reg_list_pre_soft_reset = {
			.num_of_regs = ARRAY_SIZE(full_576_768_50fps_8b_1lane_reg_pre_soft_reset),
			.regs = full_576_768_50fps_8b_1lane_reg_pre_soft_reset,
		},
		.reg_list_post_soft_reset = {
			.num_of_regs = ARRAY_SIZE(full_576_768_50fps_8b_1lane_reg_post_soft_reset),
			.regs = full_576_768_50fps_8b_1lane_reg_post_soft_reset,
		},
		.min_vblank = MIRA050_MIN_VBLANK_120,
		.max_vblank = MIRA050_MAX_VBLANK,
		.hblank = 0,
		.bit_depth = 8,
		.code = MEDIA_BUS_FMT_SGRBG8_1X8,
		.gain_min = 0,
		.gain_max = ARRAY_SIZE(fine_gain_lut_8bit_16x) - 1,
	},

};

struct mira050
{
	struct v4l2_subdev sd;
	struct media_pad pad[NUM_PADS];

	struct v4l2_mbus_framefmt fmt;

	struct clk *xclk; /* system clock to MIRA050 */
	u32 xclk_freq;

	// struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[MIRA050_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *gain;
	// custom v4l2 control
	struct v4l2_ctrl *mira050_reg_w;
	struct v4l2_ctrl *mira050_reg_r;
	u16 mira050_reg_w_cached_addr;
	u8 mira050_reg_w_cached_flag;

	/* Current mode */
	const struct mira050_mode *mode;
	/* current bit depth, may defer from mode->bit_depth */
	u8 bit_depth;
	/* OTP_CALIBRATION_VALUE stored in OTP memory */

	u16 otp_dark_cal_8bit;
	u16 otp_dark_cal_10bit_hs;
	u16 otp_dark_cal_10bit;
	u16 otp_dark_cal_12bit;

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
	/* A flag to force write_start/stop_streaming_regs even if (skip_reg_upload==1) */
	u8 force_stream_ctrl;
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

static inline struct mira050 *to_mira050(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct mira050, sd);
}

static int mira050_read(struct mira050 *mira050, u16 reg, u8 *val)
{
	int ret;
	unsigned char data_w[2] = {reg >> 8, reg & 0xff};
	struct i2c_client *client = v4l2_get_subdevdata(&mira050->sd);

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

static int mira050_write(struct mira050 *mira050, u16 reg, u8 val)
{
	int ret;
	unsigned char data[3] = {reg >> 8, reg & 0xff, val};
	struct i2c_client *client = v4l2_get_subdevdata(&mira050->sd);

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
		ret = mira050_read(mira050, ret_reg, &ret_val);
		printk(KERN_INFO "[MIRA050]: Write reg 0x%4.4x, Read ret_reg 0x%4.4x, val = 0x%x.\n",
				reg, ret_reg, ret_val);
		if (val != ret_val) {
			printk(KERN_INFO "[MIRA050]: WARNING Write reg 0x%4.4x, val = 0x%x, read ret_reg = 0x%4.4x, ret_val = 0x%x.\n",
				reg, val, ret_reg, ret_val);
		}
	}
	*/

	return ret;
}

/*
 * mira050 is big-endian: msb of val goes to lower reg addr
 */
static int mira050_write_be16(struct mira050 *mira050, u16 reg, u16 val)
{
	int ret;
	unsigned char data[4] = {reg >> 8, reg & 0xff, (val >> 8) & 0xff, val & 0xff};
	struct i2c_client *client = v4l2_get_subdevdata(&mira050->sd);

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
 * mira050 is big-endian: msb of val goes to lower reg addr
 */
static int mira050_write_be24(struct mira050 *mira050, u16 reg, u32 val)
{
	int ret;
	unsigned char data[5] = {reg >> 8, reg & 0xff, (val >> 16) & 0xff, (val >> 8) & 0xff, val & 0xff};
	struct i2c_client *client = v4l2_get_subdevdata(&mira050->sd);

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
 * mira050 is big-endian: msb of val goes to lower reg addr
 */
static int mira050_write_be32(struct mira050 *mira050, u16 reg, u32 val)
{
	int ret;
	unsigned char data[6] = {reg >> 8, reg & 0xff, (val >> 24) & 0xff, (val >> 16) & 0xff, (val >> 8) & 0xff, val & 0xff};
	struct i2c_client *client = v4l2_get_subdevdata(&mira050->sd);

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
 * mira050 OTP 32-bit val on I2C is big-endian. However, val content can be little-endian.
 */
static int mira050_read_be32(struct mira050 *mira050, u16 reg, u32 *val)
{
	int ret;
	unsigned char data_w[2] = {reg >> 8, reg & 0xff};
	/* Big-endian 32-bit buffer. */
	unsigned char data_r[4];
	struct i2c_client *client = v4l2_get_subdevdata(&mira050->sd);

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
static int mira050_write_regs(struct mira050 *mira050,
							  const struct mira050_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira050->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++)
	{
		ret = mira050_write(mira050, regs[i].address, regs[i].val);
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
			// ret = mira050_read(mira050, regs[i].address, &val);
			// printk(KERN_INFO "[MIRA050]: Read reg 0x%4.4x, val = 0x%x.\n",
			// 		regs[i].address, val);
		}
	}

	return 0;
}

/*
 * Read OTP memory: 8-bit addr and 32-bit value
 */
static int mira050_otp_read(struct mira050 *mira050, u8 addr, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira050->sd);
	u8 busy_status = 1;
	int poll_cnt = 0;
	int poll_cnt_max = 10;
	int ret;
	mira050_write(mira050, MIRA050_BANK_SEL_REG, 0);
	mira050_write(mira050, MIRA050_OTP_COMMAND, 0);
	mira050_write(mira050, MIRA050_OTP_ADDR, addr);
	mira050_write(mira050, MIRA050_OTP_START, 1);
	usleep_range(15, 50);
	mira050_write(mira050, MIRA050_OTP_START, 0);
	for (poll_cnt = 0; poll_cnt < poll_cnt_max; poll_cnt++)
	{
		mira050_read(mira050, MIRA050_OTP_BUSY, &busy_status);
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
		ret = mira050_read_be32(mira050, MIRA050_OTP_DOUT, val);
		printk(KERN_INFO "[MIRA050]: Read OTP 0x%x, val = 0x%x.\n",
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
static int mira050pmic_write(struct i2c_client *client, u8 reg, u8 val)
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

static int mira050pmic_read(struct i2c_client *client, u8 reg, u8 *val)
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
static int mira050_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira050 *mira050 = to_mira050(sd);
	int ret = -EINVAL;

	printk(KERN_INFO "[MIRA050]: Entering power on function.\n");

	if (mira050->powered == 0)
	{
		ret = regulator_bulk_enable(MIRA050_NUM_SUPPLIES, mira050->supplies);
		if (ret)
		{
			dev_err(&client->dev, "%s: failed to enable regulators\n",
					__func__);
			return ret;
		}

		ret = clk_prepare_enable(mira050->xclk);
		if (ret)
		{
			dev_err(&client->dev, "%s: failed to enable clock\n",
					__func__);
			goto reg_off;
		}

		usleep_range(MIRA050_XCLR_MIN_DELAY_US,
					 MIRA050_XCLR_MIN_DELAY_US + MIRA050_XCLR_DELAY_RANGE_US);
		mira050->powered = 1;
	}
	else
	{
		printk(KERN_INFO "[MIRA050]: Skip regulator and clk enable, because mira015->powered == %d.\n", mira050->powered);
	}
	return 0;

reg_off:
	ret = regulator_bulk_disable(MIRA050_NUM_SUPPLIES, mira050->supplies);
	return ret;
}

static int mira050_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira050 *mira050 = to_mira050(sd);

	printk(KERN_INFO "[MIRA050]: Entering power off function.\n");

	if (mira050->skip_reset == 0)
	{
		if (mira050->powered == 1)
		{
			regulator_bulk_disable(MIRA050_NUM_SUPPLIES, mira050->supplies);
			clk_disable_unprepare(mira050->xclk);
			mira050->powered = 0;
		}
		else
		{
			printk(KERN_INFO "[MIRA050]: Skip disabling regulator and clk due to mira015->powered == %d.\n", mira050->powered);
		}
	}
	else
	{
		printk(KERN_INFO "[MIRA050]: Skip disabling regulator and clk due to mira050->skip_reset=%u.\n", mira050->skip_reset);
	}

	return 0;
}

static int mira050_write_illum_trig_regs(struct mira050 *mira050)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&mira050->sd);
	int ret = 0;
	int en_trig_illum = 0;
	int en_trig_sync = 1;
	int dmux0_sel = 40;
	// Set context bank 1A or bank 1B
	ret = mira050_write(mira050, MIRA050_RW_CONTEXT_REG, 0);
	if (ret)
	{
		dev_err(&client->dev, "Error setting RW_CONTEXT.");
		return ret;
	}
	// Set reg bank 0 or 1
	ret = mira050_write(mira050, MIRA050_BANK_SEL_REG, 1);
	if (ret)
	{
		dev_err(&client->dev, "Error setting BANK_SEL_REG.");
		return ret;
	}

	/*
	if mira050->illum_enable
		if illum_width_auto
			use sync trig
		else:
			use illum trig
	else:
		all off
	*/

	if (mira050->illum_enable)
	{
		if (mira050->illum_width_auto)
		{
			dmux0_sel = 40;
			en_trig_illum = 0;
			en_trig_sync = 1;
		}
		else
		{
			dmux0_sel = 39;
			en_trig_illum = 1;
			en_trig_sync = 0;
		}
	}
	else
	{
		en_trig_illum = 0;
		en_trig_sync = 0;
	}

	// Enable or disable illumination trigger
	printk(KERN_INFO "[MIRA050]: Writing EN_TRIG_ILLUM to %d.\n", en_trig_illum);
	ret = mira050_write(mira050, MIRA050_EN_TRIG_ILLUM, en_trig_illum);
	if (ret)
	{
		dev_err(&client->dev, "Error setting EN_TRIG_ILLUM to %d.", en_trig_illum);
		return ret;
	}
	// Enable or disable illumination trigger
	printk(KERN_INFO "[MIRA050]: Writing MIRA050_TRIG_SYNC_ON_REQ_1 to %d.\n", en_trig_sync);
	ret = mira050_write(mira050, MIRA050_TRIG_SYNC_ON_REQ_1, en_trig_sync);
	if (ret)
	{
		dev_err(&client->dev, "Error setting MIRA050_TRIG_SYNC_ON_REQ_1 to %d.", en_trig_sync);
		return ret;
	}

	// Enable or disable sync trigger
	printk(KERN_INFO "[MIRA050]: Writing EN_TRIG_SYNC to %d.\n", en_trig_sync);
	ret = mira050_write(mira050, MIRA050_EN_TRIG_SYNC, en_trig_sync);
	if (ret)
	{
		dev_err(&client->dev, "Error setting MIRA050_EN_TRIG_SYNC to %d.", en_trig_sync);
		return ret;
	}
	// Set illumination width. Write 24 bits. All 24 bits are valid.
	printk(KERN_INFO "[MIRA050]: Writing ILLUM_WIDTH to %u.\n", mira050->illum_width);
	ret = mira050_write_be24(mira050, MIRA050_ILLUM_WIDTH_REG, mira050->illum_width);
	if (ret)
	{
		dev_err(&client->dev, "Error setting ILLUM_WIDTH to %u.", mira050->illum_width);
		return ret;
	}

	// Set illumination delay. Write 24 bits. Only 20 bits, [19:0], are valid.
	printk(KERN_INFO "[MIRA050]: Writing ILLUM_DELAY to %u.\n", mira050->illum_delay);
	ret = mira050_write_be24(mira050, MIRA050_ILLUM_DELAY_REG, mira050->illum_delay);
	if (ret)
	{
		dev_err(&client->dev, "Error setting ILLUM_DELAY to %u.", mira050->illum_delay);
		return ret;
	}

	return ret;

	// Set reg bank 0 or 1
	ret = mira050_write(mira050, MIRA050_BANK_SEL_REG, 0);
	if (ret)
	{
		dev_err(&client->dev, "Error setting BANK_SEL_REG.");
		return ret;
	}

	// DMUX SEL (set bank)
	printk(KERN_INFO "[MIRA050]: Writing DMUX0SEL to %d.\n", dmux0_sel);
	ret = mira050_write(mira050, MIRA050_DMUX0_SEL, dmux0_sel);
	if (ret)
	{
		dev_err(&client->dev, "Error setting MIRA050_DMUX0_SEL to %d.", dmux0_sel);
		return ret;
	}
}

static int mira050_v4l2_reg_w(struct mira050 *mira050, u32 value)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&mira050->sd);
	u32 ret = 0;
	u32 tmp_flag;

	u16 reg_addr = (value >> 8) & 0xFFFF;
	u8 reg_val = value & 0xFF;
	u8 reg_flag = (value >> 24) & 0xFF;

	// printk(KERN_INFO "[MIRA050]: %s reg_flag: 0x%02X; reg_addr: 0x%04X; reg_val: 0x%02X.\n",
	// 		__func__, reg_flag, reg_addr, reg_val);

	if (reg_flag & AMS_CAMERA_CID_MIRA050_REG_FLAG_CMD_SEL)
	{
		if (reg_flag == AMS_CAMERA_CID_MIRA050_REG_FLAG_SLEEP_US)
		{
			// If it is for sleep, combine all 24 bits of reg_addr and reg_val as sleep us.
			u32 sleep_us_val = value & 0x00FFFFFF;
			// Sleep range needs an interval, default to 1/8 of the sleep value.
			u32 sleep_us_interval = sleep_us_val >> 3;
			printk(KERN_INFO "[MIRA050]: %s sleep_us: %u.\n", __func__, sleep_us_val);
			usleep_range(sleep_us_val, sleep_us_val + sleep_us_interval);
		}
		else if (reg_flag == AMS_CAMERA_CID_MIRA050_REG_FLAG_RESET_ON)
		{
			printk(KERN_INFO "[MIRA050]: %s Enable reset at stream on/off.\n", __func__);
			mira050->skip_reset = 0;
		}
		else if (reg_flag == AMS_CAMERA_CID_MIRA050_REG_FLAG_RESET_OFF)
		{
			printk(KERN_INFO "[MIRA050]: %s Disable reset at stream on/off.\n", __func__);
			mira050->skip_reset = 1;
		}
		else if (reg_flag == AMS_CAMERA_CID_MIRA050_REG_FLAG_REG_UP_ON)
		{
			printk(KERN_INFO "[MIRA050]: %s Enable base register sequence upload.\n", __func__);
			mira050->skip_reg_upload = 0;
		}
		else if (reg_flag == AMS_CAMERA_CID_MIRA050_REG_FLAG_REG_UP_OFF)
		{
			printk(KERN_INFO "[MIRA050]: %s Disable base register sequence upload.\n", __func__);
			mira050->skip_reg_upload = 1;
		}
		else if (reg_flag == AMS_CAMERA_CID_MIRA050_REG_FLAG_POWER_ON)
		{
			printk(KERN_INFO "[MIRA050]: %s Call power on function mira050_power_on().\n", __func__);
			/* Temporarily disable skip_reset if manually doing power on/off */
			tmp_flag = mira050->skip_reset;
			mira050->skip_reset = 0;
			mira050_power_on(&client->dev);
			mira050->skip_reset = tmp_flag;
		}
		else if (reg_flag == AMS_CAMERA_CID_MIRA050_REG_FLAG_POWER_OFF)
		{
			printk(KERN_INFO "[MIRA050]: %s Call power off function mira050_power_off().\n", __func__);
			/* Temporarily disable skip_reset if manually doing power on/off */
			tmp_flag = mira050->skip_reset;
			mira050->skip_reset = 0;
			mira050_power_off(&client->dev);
			mira050->skip_reset = tmp_flag;
		}
		else if (reg_flag == AMS_CAMERA_CID_MIRA050_REG_FLAG_ILLUM_TRIG_ON)
		{
			printk(KERN_INFO "[MIRA050]: %s Enable illumination trigger.\n", __func__);
			mira050->illum_enable = 1;
			mira050_write_illum_trig_regs(mira050);
		}
		else if (reg_flag == AMS_CAMERA_CID_MIRA050_REG_FLAG_ILLUM_TRIG_OFF)
		{
			printk(KERN_INFO "[MIRA050]: %s Disable illumination trigger.\n", __func__);
			mira050->illum_enable = 0;
			mira050_write_illum_trig_regs(mira050);
		}
		else if (reg_flag == AMS_CAMERA_CID_MIRA050_REG_FLAG_ILLUM_WIDTH)
		{
			// Combine all 24 bits of reg_addr and reg_val as ILLUM_WIDTH.
			u32 illum_width = value & 0x00FFFFFF;
			printk(KERN_INFO "[MIRA050]: %s Set ILLUM_WIDTH to 0x%X.\n", __func__, illum_width);
			mira050->illum_width = illum_width;
			mira050_write_illum_trig_regs(mira050);
		}
		else if (reg_flag == AMS_CAMERA_CID_MIRA050_REG_FLAG_ILLUM_DELAY)
		{
			// Combine reg_addr and reg_val, then select 20 bits from [19:0] as ILLUM_DELAY.
			u32 illum_delay = value & 0x000FFFFF;
			printk(KERN_INFO "[MIRA050]: %s Set ILLUM_DELAY to 0x%X.\n", __func__, illum_delay);
			mira050->illum_delay = illum_delay;
			mira050_write_illum_trig_regs(mira050);
		}
		else if (reg_flag == AMS_CAMERA_CID_MIRA050_REG_FLAG_ILLUM_EXP_T_ON)
		{
			printk(KERN_INFO "[MIRA050]: %s enable ILLUM_WIDTH to automatically track exposure time.\n", __func__);
			mira050->illum_width_auto = 1;
			mira050_write_illum_trig_regs(mira050);
		}
		else if (reg_flag == AMS_CAMERA_CID_MIRA050_REG_FLAG_ILLUM_EXP_T_OFF)
		{
			printk(KERN_INFO "[MIRA050]: %s disable ILLUM_WIDTH to automatically track exposure time.\n", __func__);
			mira050->illum_width_auto = 0;
			mira050_write_illum_trig_regs(mira050);
		}
		else if (reg_flag == AMS_CAMERA_CID_MIRA050_REG_FLAG_STREAM_CTRL_ON)
		{
			printk(KERN_INFO "[MIRA050]: %s Force stream control even if (skip_reg_upload == 1).\n", __func__);
			mira050->force_stream_ctrl = 1;
		}
		else if (reg_flag == AMS_CAMERA_CID_MIRA050_REG_FLAG_STREAM_CTRL_OFF)
		{
			printk(KERN_INFO "[MIRA050]: %s Disable stream control if (skip_reg_upload == 1).\n", __func__);
			mira050->force_stream_ctrl = 0;
		}
		else
		{
			printk(KERN_INFO "[MIRA050]: %s unknown command from flag %u, ignored.\n", __func__, reg_flag);
		}
	}
	else if (reg_flag & AMS_CAMERA_CID_MIRA050_REG_FLAG_FOR_READ)
	{
		// If it is for read, skip reagister write, cache addr and flag for read.
		mira050->mira050_reg_w_cached_addr = reg_addr;
		mira050->mira050_reg_w_cached_flag = reg_flag;
	}
	else
	{
		// If it is for write, select which I2C device by the flag "I2C_SEL".
		if ((reg_flag & AMS_CAMERA_CID_MIRA050_REG_FLAG_I2C_SEL) == AMS_CAMERA_CID_MIRA050_REG_FLAG_I2C_MIRA)
		{
			// Before writing Mira050 register, first optionally select BANK and CONTEXT
			if (reg_flag & AMS_CAMERA_CID_MIRA050_REG_FLAG_USE_BANK)
			{
				u8 bank;
				u8 context;
				// Set conetxt bank 0 or 1
				if (reg_flag & AMS_CAMERA_CID_MIRA050_REG_FLAG_BANK)
				{
					bank = 1;
				}
				else
				{
					bank = 0;
				}
				// printk(KERN_INFO "[MIRA050]: %s select bank: %u.\n", __func__, bank);
				ret = mira050_write(mira050, MIRA050_BANK_SEL_REG, bank);
				if (ret)
				{
					dev_err(&client->dev, "Error setting BANK_SEL_REG.");
					return ret;
				}
				// Set context bank 1A or bank 1B
				if (reg_flag & AMS_CAMERA_CID_MIRA050_REG_FLAG_CONTEXT)
				{
					context = 1;
				}
				else
				{
					context = 0;
				}
				// printk(KERN_INFO "[MIRA050]: %s select context: %u.\n", __func__, context);
				ret = mira050_write(mira050, MIRA050_RW_CONTEXT_REG, context);
				if (ret)
				{
					dev_err(&client->dev, "Error setting RW_CONTEXT.");
					return ret;
				}
			}
			// Writing the actual Mira050 register
			// printk(KERN_INFO "[MIRA050]: %s write reg_addr: 0x%04X; reg_val: 0x%02X.\n", __func__, reg_addr, reg_val);
			ret = mira050_write(mira050, reg_addr, reg_val);
			if (ret)
			{
				dev_err_ratelimited(&client->dev, "Error AMS_CAMERA_CID_MIRA_REG_W reg_addr %X.\n", reg_addr);
				return -EINVAL;
			}
		}
		else if ((reg_flag & AMS_CAMERA_CID_MIRA050_REG_FLAG_I2C_SEL) == AMS_CAMERA_CID_MIRA050_REG_FLAG_I2C_SET_TBD)
		{
			/* User tries to set TBD I2C address, store reg_val to mira050->tbd_client_i2c_addr. Skip write. */
			printk(KERN_INFO "[MIRA050]: mira050->tbd_client_i2c_addr = 0x%X.\n", reg_val);
			mira050->tbd_client_i2c_addr = reg_val;
		}
		else if ((reg_flag & AMS_CAMERA_CID_MIRA050_REG_FLAG_I2C_SEL) == AMS_CAMERA_CID_MIRA050_REG_FLAG_I2C_TBD)
		{
			if (mira050->tbd_client_i2c_addr == MIRA050PMIC_I2C_ADDR)
			{
				// Write PMIC. Use pre-allocated mira050->pmic_client.
				printk(KERN_INFO "[MIRA050]: write pmic_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
				ret = mira050pmic_write(mira050->pmic_client, (u8)(reg_addr & 0xFF), reg_val);
			}
			else if (mira050->tbd_client_i2c_addr == MIRA050UC_I2C_ADDR)
			{
				// Write micro-controller. Use pre-allocated mira050->uc_client.
				printk(KERN_INFO "[MIRA050]: write uc_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
				ret = mira050pmic_write(mira050->uc_client, (u8)(reg_addr & 0xFF), reg_val);
			}
			else if (mira050->tbd_client_i2c_addr == MIRA050LED_I2C_ADDR)
			{
				// Write LED driver. Use pre-allocated mira050->led_client.
				printk(KERN_INFO "[MIRA050]: write led_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
				ret = mira050pmic_write(mira050->led_client, (u8)(reg_addr & 0xFF), reg_val);
			}
			else
			{
				/* Write other TBD I2C address.
				 * The TBD I2C address is set via AMS_CAMERA_CID_MIRA050_REG_FLAG_I2C_SET_TBD.
				 * The TBD I2C address is stored in mira050->tbd_client_i2c_addr.
				 * A temporary I2C client, tmp_client, is created and then destroyed (unregistered).
				 */
				struct i2c_client *tmp_client;
				tmp_client = i2c_new_dummy_device(client->adapter, mira050->tbd_client_i2c_addr);
				if (IS_ERR(tmp_client))
					return PTR_ERR(tmp_client);
				printk(KERN_INFO "[MIRA050]: write tbd_client, i2c_addr %u, reg_addr 0x%X, reg_val 0x%X.\n",
					   mira050->tbd_client_i2c_addr, (u8)(reg_addr & 0xFF), reg_val);
				ret = mira050pmic_write(tmp_client, (u8)(reg_addr & 0xFF), reg_val);
				i2c_unregister_device(tmp_client);
			}
		}
	}

	return 0;
}

static int mira050_v4l2_reg_r(struct mira050 *mira050, u32 *value)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&mira050->sd);
	u32 ret = 0;

	u16 reg_addr = mira050->mira050_reg_w_cached_addr;
	u8 reg_flag = mira050->mira050_reg_w_cached_flag;
	u8 reg_val = 0;

	*value = 0;

	if ((reg_flag & AMS_CAMERA_CID_MIRA050_REG_FLAG_I2C_SEL) == AMS_CAMERA_CID_MIRA050_REG_FLAG_I2C_MIRA)
	{
		if (reg_flag & AMS_CAMERA_CID_MIRA050_REG_FLAG_USE_BANK)
		{
			u8 bank;
			u8 context;
			// Set conetxt bank 0 or 1
			if (reg_flag & AMS_CAMERA_CID_MIRA050_REG_FLAG_BANK)
			{
				bank = 1;
			}
			else
			{
				bank = 0;
			}
			// printk(KERN_INFO "[MIRA050]: %s select bank: %u.\n", __func__, bank);
			ret = mira050_write(mira050, MIRA050_BANK_SEL_REG, bank);
			if (ret)
			{
				dev_err(&client->dev, "Error setting BANK_SEL_REG.");
				return ret;
			}
			// Set context bank 1A or bank 1B
			if (reg_flag & AMS_CAMERA_CID_MIRA050_REG_FLAG_CONTEXT)
			{
				context = 1;
			}
			else
			{
				context = 0;
			}
			// printk(KERN_INFO "[MIRA050]: %s select context: %u.\n", __func__, context);
			ret = mira050_write(mira050, MIRA050_RW_CONTEXT_REG, context);
			if (ret)
			{
				dev_err(&client->dev, "Error setting RW_CONTEXT.");
				return ret;
			}
		}
		ret = mira050_read(mira050, reg_addr, &reg_val);
		if (ret)
		{
			dev_err_ratelimited(&client->dev, "Error AMS_CAMERA_CID_MIRA_REG_R reg_addr %X.\n", reg_addr);
			return -EINVAL;
		}
	}
	else if ((reg_flag & AMS_CAMERA_CID_MIRA050_REG_FLAG_I2C_SEL) == AMS_CAMERA_CID_MIRA050_REG_FLAG_I2C_TBD)
	{
		if (mira050->tbd_client_i2c_addr == MIRA050PMIC_I2C_ADDR)
		{
			// Read PMIC. Use pre-allocated mira050->pmic_client.
			ret = mira050pmic_read(mira050->pmic_client, (u8)(reg_addr & 0xFF), &reg_val);
			printk(KERN_INFO "[MIRA050]: read pmic_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
		}
		else if (mira050->tbd_client_i2c_addr == MIRA050UC_I2C_ADDR)
		{
			// Read micro-controller. Use pre-allocated mira050->uc_client.
			ret = mira050pmic_read(mira050->uc_client, (u8)(reg_addr & 0xFF), &reg_val);
			printk(KERN_INFO "[MIRA050]: read uc_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
		}
		else if (mira050->tbd_client_i2c_addr == MIRA050LED_I2C_ADDR)
		{
			// Read LED driver. Use pre-allocated mira050->led_client.
			ret = mira050pmic_read(mira050->led_client, (u8)(reg_addr & 0xFF), &reg_val);
			printk(KERN_INFO "[MIRA050]: read led_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
		}
		else
		{
			/* Read other TBD I2C address.
			 * The TBD I2C address is set via AMS_CAMERA_CID_MIRA050_REG_FLAG_I2C_SET_TBD.
			 * The TBD I2C address is stored in mira050->tbd_client_i2c_addr.
			 * A temporary I2C client, tmp_client, is created and then destroyed (unregistered).
			 */
			struct i2c_client *tmp_client;
			tmp_client = i2c_new_dummy_device(client->adapter, mira050->tbd_client_i2c_addr);
			if (IS_ERR(tmp_client))
				return PTR_ERR(tmp_client);
			ret = mira050pmic_read(tmp_client, (u8)(reg_addr & 0xFF), &reg_val);
			printk(KERN_INFO "[MIRA050]: read tbd_client, i2c_addr %u, reg_addr 0x%X, reg_val 0x%X.\n",
				   mira050->tbd_client_i2c_addr, (u8)(reg_addr & 0xFF), reg_val);
			i2c_unregister_device(tmp_client);
		}
	}

	// Return 32-bit value that includes flags, addr, and register value
	*value = ((u32)reg_flag << 24) | ((u32)reg_addr << 8) | (u32)reg_val;

	// printk(KERN_INFO "[MIRA050]: mira050_v4l2_reg_r() reg_flag: 0x%02X; reg_addr: 0x%04X, reg_val: 0x%02X.\n",
	// 		reg_flag, reg_addr, reg_val);

	return 0;
}

// Returns the maximum exposure time in microseconds (reg value)
static u32 mira050_calculate_max_exposure_time(u32 row_length, u32 vsize,
											   u32 vblank)
{
	(void)(row_length);
	(void)(vsize);
	(void)(vblank);
	/* Mira050 does not have a max exposure limit besides register bits */
	// return row_length * (vsize + vblank) - MIRA050_GLOB_NUM_CLK_CYCLES;
	return MIRA050_EXPOSURE_MAX_LINES;
}

static int mira050_write_exposure_reg(struct mira050 *mira050, u32 exposure_lines)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&mira050->sd);
	const u32 min_exposure = MIRA050_EXPOSURE_MIN_US;
	u32 max_exposure = mira050->exposure->maximum * MIRA050_MIN_ROW_LENGTH_US;
	u32 ret = 0;
	u32 exposure = exposure_lines * MIRA050_DEFAULT_LINE_LENGTH;


	if (exposure < min_exposure)
	{
		exposure = min_exposure;
	}
	if (exposure > max_exposure)
	{
		exposure = max_exposure;
	}

	// printk(KERN_INFO "[MIRA050]: mira050_write_exposure_reg: exp us = %u.\n", exposure);

	/* Write Bank 1 context 0 */
	ret = mira050_write(mira050, MIRA050_RW_CONTEXT_REG, 0);
	ret = mira050_write(mira050, MIRA050_BANK_SEL_REG, 1);
	ret = mira050_write_be32(mira050, MIRA050_EXP_TIME_L_REG, exposure);
	/* Write Bank 1 context 1 */
	ret = mira050_write(mira050, MIRA050_RW_CONTEXT_REG, 1);
	ret = mira050_write_be32(mira050, MIRA050_EXP_TIME_L_REG, exposure);
	if (ret)
	{
		dev_err_ratelimited(&client->dev, "Error setting exposure time to %d", exposure);
		return -EINVAL;
	}

	// if (mira050->illum_width_auto == 1)
	// {
	// 	mira050->illum_width = (exposure / 8) * MIRA050_DATA_RATE;
	// 	mira050_write_illum_trig_regs(mira050);
	// }

	return 0;
}

static int mira050_write_target_frame_time_reg(struct mira050 *mira050, u32 target_frame_time_us)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&mira050->sd);
	u32 ret = 0;

	/* Write Bank 1 context 0 */
	ret = mira050_write(mira050, MIRA050_RW_CONTEXT_REG, 0);
	ret = mira050_write(mira050, MIRA050_BANK_SEL_REG, 1);
	ret = mira050_write_be32(mira050, MIRA050_TARGET_FRAME_TIME_REG, target_frame_time_us);
	/* Write Bank 1 context 1 */
	ret = mira050_write(mira050, MIRA050_RW_CONTEXT_REG, 1);
	ret = mira050_write_be32(mira050, MIRA050_TARGET_FRAME_TIME_REG, target_frame_time_us);
	if (ret)
	{
		dev_err_ratelimited(&client->dev, "Error setting target frame time to %d", target_frame_time_us);
		return -EINVAL;
	}

	return 0;
}

static int mira050_write_start_streaming_regs(struct mira050 *mira050)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&mira050->sd);
	int ret = 0;

	// Set conetxt bank 0 or 1
	ret = mira050_write(mira050, MIRA050_BANK_SEL_REG, 0);
	if (ret)
	{
		dev_err(&client->dev, "Error setting BANK_SEL_REG.");
		return ret;
	}

	// Set context bank 1A or bank 1B
	ret = mira050_write(mira050, MIRA050_RW_CONTEXT_REG, 0);
	if (ret)
	{
		dev_err(&client->dev, "Error setting RW_CONTEXT.");
		return ret;
	}

	// Raising CMD_REQ_1 to 1 for REQ_EXP
	ret = mira050_write(mira050, MIRA050_CMD_REQ_1_REG,
						1);
	if (ret)
	{
		dev_err(&client->dev, "Error setting CMD_REQ_1 to 1 for REQ_EXP.");
		return ret;
	}

	usleep_range(10, 20);

	// Setting CMD_REQ_1 tp 0 for REQ_EXP
	ret = mira050_write(mira050, MIRA050_CMD_REQ_1_REG,
						0);
	if (ret)
	{
		dev_err(&client->dev, "Error setting CMD_REQ_1 to 0 for REQ_EXP.");
		return ret;
	}
	usleep_range(10, 20);

	return ret;
}

static int mira050_write_stop_streaming_regs(struct mira050 *mira050)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&mira050->sd);
	int ret = 0;

	// Set conetxt bank 0 or 1
	ret = mira050_write(mira050, MIRA050_BANK_SEL_REG, 0);
	if (ret)
	{
		dev_err(&client->dev, "Error setting BANK_SEL_REG.");
		return ret;
	}

	// Raising CMD_HALT_BLOCK to 1 to stop streaming
	ret = mira050_write(mira050, MIRA050_CMD_HALT_BLOCK_REG,
						1);
	if (ret)
	{
		dev_err(&client->dev, "Error setting CMD_HALT_BLOCK to 1.");
		return ret;
	}

	usleep_range(10, 20);

	// Setting CMD_HALT_BLOCK to 0 to stop streaming
	ret = mira050_write(mira050, MIRA050_CMD_HALT_BLOCK_REG,
						0);
	if (ret)
	{
		dev_err(&client->dev, "Error setting CMD_HALT_BLOCK to 0.");
		return ret;
	}
	usleep_range(10, 20);

	/*
	 * Wait for one frame to make sure sensor is set to
	 * software standby in V-blank
	 *
	 * frame_time = frame length rows * Tline
	 * Tline = line length / pixel clock (in MHz)
	 */
	/*
	u32 frame_time;
		frame_time = MIRA050_DEFAULT_FRAME_LENGTH *
			MIRA050_DEFAULT_LINE_LENGTH / MIRA050_DEFAULT_PIXEL_CLOCK;

		usleep_range(frame_time, frame_time + 1000);
	*/

	return ret;
}

static int mira050_write_analog_gain_reg(struct mira050 *mira050, u8 gain)
{
	struct i2c_client *const client = v4l2_get_subdevdata(&mira050->sd);
	u32 num_of_regs;
	u32 ret = 0;
	u32 wait_us = 20000;
	u16 target_black_level = 128;
	u16 cds_offset = 1700;
	u16 dark_offset_100 = 1794; // noncont clock
	u16 scale_factor = 1;
	u16 preamp_gain_inv = 1;
	u16 preamp_gain = 1;

	u16 analog_gain = 1;
	u16 offset_clipping = 0;
	u16 scaled_offset = 0;
	printk(KERN_INFO "[MIRA050]: Write analog gain %u",gain);

	// Select partial register sequence according to bit depth
	if (mira050->bit_depth == 12)
	{
		mira050_write_stop_streaming_regs(mira050);
		usleep_range(wait_us, wait_us + 100);
		scale_factor = 1;

		// Select register sequence according to gain value
		if (gain == 0)
		{
			analog_gain = 1;
			// printk(KERN_INFO "[MIRA050]: Write reg sequence for analog gain x1 in 12 bit mode");
			num_of_regs = ARRAY_SIZE(partial_analog_gain_x1_12bit);
			ret = mira050_write_regs(mira050, partial_analog_gain_x1_12bit, num_of_regs);
			cds_offset = 1700;
		}
		else if (gain == 1)
		{
			analog_gain = 2;
			// printk(KERN_INFO "[MIRA050]: Write reg sequence for analog gain x2 in 12 bit mode");
			num_of_regs = ARRAY_SIZE(partial_analog_gain_x2_12bit);
			ret = mira050_write_regs(mira050, partial_analog_gain_x2_12bit, num_of_regs);
			cds_offset = 2708;
		}
		else if (gain == 2)
		{
			analog_gain = 4;

			// printk(KERN_INFO "[MIRA050]: Write reg sequence for analog gain x4 in 12 bit mode");
			num_of_regs = ARRAY_SIZE(partial_analog_gain_x4_12bit);
			ret = mira050_write_regs(mira050, partial_analog_gain_x4_12bit, num_of_regs);
			cds_offset = 4500;
		}
		else
		{
			// Other gains are not supported
			// printk(KERN_INFO "[MIRA050]: Ignore analog gain %u in 12 bit mode", gain);
		}
		u16 part1 = (mira050->otp_dark_cal_12bit + dark_offset_100) / 100;
		u16 part3 = (dark_offset_100 / 100);
		u16 part2 = analog_gain / (scale_factor * preamp_gain);
		scaled_offset = (u16)((mira050->otp_dark_cal_12bit + dark_offset_100) / 100 * analog_gain / (scale_factor * preamp_gain)) - (u16)(dark_offset_100 / 100);
		// printk(KERN_INFO "[MIRA050]: scaled offset  12 bit mode is %u dark cal is %u", scaled_offset, mira050->otp_dark_cal_12bit);
		// printk(KERN_INFO "[MIRA050]: scaled offset  12 bit mode part1 %u part2 %u part3 %u ", part1, part2, part3);



		scaled_offset = (u16)(((mira050->otp_dark_cal_12bit + dark_offset_100) * analog_gain * preamp_gain_inv / (scale_factor)) - dark_offset_100) / 100;
		// newly added.
		// printk(KERN_INFO "[MIRA050]: scaled offset  new 12 bit mode is %u dark cal is %u", scaled_offset, mira050->otp_dark_cal_12bit);

		/* Avoid negative offset_clipping value. */
		offset_clipping = ((int)(cds_offset - target_black_level * preamp_gain_inv + scaled_offset) < 0 ? 0 : (int)(cds_offset - target_black_level * preamp_gain_inv + scaled_offset));
		/* Stop streaming and wait for frame data transmission done */
		// mira050_write_stop_streaming_regs(mira050);


		/* Avoid negative offset_clipping value. */
		// offset_clipping = ((int)(cds_offset - target_black_level / preamp_gain + scaled_offset) < 0 ? 0 : (int)(cds_offset - target_black_level / preamp_gain + scaled_offset));
		/* Stop streaming and wait for frame data transmission done */
		// mira050_write_stop_streaming_regs(mira050);
		printk(KERN_INFO "[MIRA050]: offset clip  12 bit mode is  %u", offset_clipping);

		usleep_range(wait_us, wait_us + 100);
		/* Write fine gain registers */
		
		mira050_write(mira050, MIRA050_BANK_SEL_REG, 0);
		mira050_write_be16(mira050, MIRA050_OFFSET_CLIPPING, offset_clipping);
		printk(KERN_INFO "[MIRA050]: Write offset clipping, val = 0x%x.\n",
			   offset_clipping);
		mira050_write_start_streaming_regs(mira050);

		// mira050_write_start_streaming_regs(mira050);
	}
	else if (mira050->bit_depth == 10) // 10bit high speed mode gain 1-4
	{
		dark_offset_100 = 291; // noncont clock
		scale_factor = 4;
		cds_offset = 1540;
		target_black_level = 32;

		if (gain < ARRAY_SIZE(fine_gain_lut_10bit_hs_4x))
		{
			u32 analog_gain = fine_gain_lut_10bit_hs_4x[gain].analog_gain;
			u8 gdig_preamp = fine_gain_lut_10bit_hs_4x[gain].gdig_preamp;
			u8 rg_adcgain = fine_gain_lut_10bit_hs_4x[gain].rg_adcgain;
			u8 rg_mult = fine_gain_lut_10bit_hs_4x[gain].rg_mult;
			/* otp_cal_val should come from OTP, but OTP may have incorrect value. */
			u16 preamp_gain_inv = 16 / (gdig_preamp + 1); // invert because fixed point arithmetic

			u16 scaled_offset = (u16)(((mira050->otp_dark_cal_10bit_hs + dark_offset_100) * analog_gain * preamp_gain_inv / (scale_factor)/256) - dark_offset_100) / 100; //div by 256 for analog gain
			// newly added.
			printk(KERN_INFO "[MIRA050]: scaled offset  10 bit mode is %u dark cal is %u", scaled_offset, mira050->otp_dark_cal_10bit_hs);

			/* Avoid negative offset_clipping value. */
			u16 offset_clipping = ((int)(cds_offset - target_black_level * preamp_gain_inv + scaled_offset) < 0 ? 0 : (int)(cds_offset - target_black_level * preamp_gain_inv + scaled_offset));
			/* Stop streaming and wait for frame data transmission done */
			// mira050_write_stop_streaming_regs(mira050);
			printk(KERN_INFO "[MIRA050]: offset clip  10 bit mode is  %u", offset_clipping);

			// int part1 = (int)(otp_cal_val + 2.5) ;
			// int part2 = (int)(part1*analog_gain / (int)(gdig_preamp + 1)) ;
			// int offset_clipping = (part2>>8) - 2;
			// int scaled_offset = ((otp_cal_val - 1540) / 4 - target_black_level) * 16 / (gdig_preamp + 1);
			/* Avoid negative offset_clipping value. */
			// int offset_clipping = (int)((otp_cal_fine_val + 4) * analog_gain / (int)(gdig_preamp + 1) / 256 - 4);

			// int offset_clipping_calc = (int)(cds_offset - (target_black_level  - offset_clipping));
			/*preamp_gain, _, _ = gain_settings._gain_lut[gain]
scaled_offset = (calibration_value + 2.5) * gain/(16*preamp_gain) - 2.5
mira_xs_api.write_register(
'OFFSET_CLIPPING',
2*act1 - round(target_black_level/preamp_gain - scaled_offset)
	offset_clipping = (calibration_value + 2.5) * (analog_gain/4) - 2.5
cds_offset = 1540
offset_clip = int(cds_offset - round(target/digital_gain - offset_clipping))
*/
			// if (offset_clipping_calc < 0)
			// {
			// 	offset_clipping = 0;
			// }
			// else
			// {
			// 	offset_clipping = (uint16_t)(offset_clipping_calc);
			// }
			//  = (int)(cds_offset - (target_black_level*digital_gain - offset_clipping)) < 0 ? 0 : (int)(cds_offset - (target_black_level*digital_gain - offset_clipping));

			// u16 offset_clipping = (offset_clipping_calc < 0) ? 0 : (int)(offset_clipping_calc);
			/* Stop streaming and wait for frame data transmission done */
			mira050_write_stop_streaming_regs(mira050);
			usleep_range(wait_us, wait_us + 100);
			/* Write fine gain registers */
			printk(KERN_INFO "[MIRA050]: Write reg sequence for analog gain %u in 10 bit mode", gain);
			printk(KERN_INFO "[MIRA050]: analoggain: %u,gdig_preamp: %u rg_adcgain: %u, rg_mult: %u, offset_clipping: %u,   offset_clipping: %u\n",
				   analog_gain, gdig_preamp, rg_adcgain, rg_mult, offset_clipping, offset_clipping);
			mira050_write(mira050, MIRA050_RW_CONTEXT_REG, 0);
			mira050_write(mira050, MIRA050_BANK_SEL_REG, 1);
			mira050_write(mira050, MIRA050_GDIG_PREAMP, gdig_preamp);
			mira050_write(mira050, MIRA050_BANK_SEL_REG, 0);
			mira050_write(mira050, MIRA050_BIAS_RG_ADCGAIN, rg_adcgain);
			mira050_write(mira050, MIRA050_BIAS_RG_MULT, rg_mult);
			mira050_write_be16(mira050, MIRA050_OFFSET_CLIPPING, offset_clipping);
			/* Resume streaming */
			mira050_write_start_streaming_regs(mira050);
		}
	}
	else if (mira050->bit_depth == 8)
	{
		dark_offset_100 = 72; // noncont clock
		scale_factor = 16;
		cds_offset = 1540;
		target_black_level = 16;

		if (gain < ARRAY_SIZE(fine_gain_lut_8bit_16x))
		{
			u32 analog_gain = fine_gain_lut_8bit_16x[gain].analog_gain;
			u8 gdig_preamp = fine_gain_lut_8bit_16x[gain].gdig_preamp;
			u8 rg_adcgain = fine_gain_lut_8bit_16x[gain].rg_adcgain;
			u8 rg_mult = fine_gain_lut_8bit_16x[gain].rg_mult;
			/* otp_cal_val should come from OTP, but OTP may have incorrect value. */
			u16 preamp_gain_inv = 16 / (gdig_preamp + 1);

			// int part1 = (int)(otp_cal_val + 2.5) ;
			// int part2 = (int)(part1*analog_gain / (int)(gdig_preamp + 1)) ;
			// int offset_clipping = (part2>>8) - 2;
			// int scaled_offset = ((otp_cal_val - 1540) / 4 - target_black_level) * 16 / (gdig_preamp + 1);
			/* Avoid negative offset_clipping value. */

			// newly added.
			u16 scaled_offset = (u16)(((mira050->otp_dark_cal_8bit + dark_offset_100) * analog_gain * preamp_gain_inv / (scale_factor)/256) - dark_offset_100) / 100;
			printk(KERN_INFO "[MIRA050]: scaled offset 8 bit mode is %u dark cal is %u", scaled_offset, mira050->otp_dark_cal_8bit);

			/* Avoid negative offset_clipping value. */
			u16 offset_clipping = ((int)(cds_offset - target_black_level * preamp_gain_inv + scaled_offset) < 0 ? 0 : (int)(cds_offset - target_black_level * preamp_gain_inv + scaled_offset));
			/* Stop streaming and wait for frame data transmission done */
			// mira050_write_stop_streaming_regs(mira050);
			printk(KERN_INFO "[MIRA050]: offset clip  8 bit mode is  %u", offset_clipping);

			// int part1 = (int)(otp_cal_val + 2.5) ;
			// int part2 = (int)(part1*analog_gain / (int)(gdig_preamp + 1)) ;
			// int offset_clipping = (part2>>8) - 2;
			// int scaled_offset = ((otp_cal_val - 1540) / 4 - target_black_level) * 16 / (gdig_preamp + 1);
			/* Avoid negative offset_clipping value. */
			// int offset_clipping = (int)((otp_cal_fine_val + 4) * analog_gain / (int)(gdig_preamp + 1) / 256 - 4);

			// int offset_clipping_calc = (int)(cds_offset - (target_black_level  - offset_clipping));
			/*preamp_gain, _, _ = gain_settings._gain_lut[gain]
scaled_offset = (calibration_value + 2.5) * gain/(16*preamp_gain) - 2.5
mira_xs_api.write_register(
'OFFSET_CLIPPING',
2*act1 - round(target_black_level/preamp_gain - scaled_offset)
	offset_clipping = (calibration_value + 2.5) * (analog_gain/4) - 2.5
cds_offset = 1540
offset_clip = int(cds_offset - round(target/digital_gain - offset_clipping))
*/
			// if (offset_clipping_calc < 0)
			// {
			// 	offset_clipping = 0;
			// }
			// else
			// {
			// 	offset_clipping = (uint16_t)(offset_clipping_calc);
			// }
			printk(KERN_INFO "[MIRA050]: est offset: %u,offset_clipping_calc: %u rg_adcgain: %u, rg_mult: %u, offset_clipping: %u\n",
				   analog_gain / 256, gdig_preamp, rg_adcgain, rg_mult, offset_clipping);
			//  = (int)(cds_offset - (target_black_level*digital_gain - offset_clipping)) < 0 ? 0 : (int)(cds_offset - (target_black_level*digital_gain - offset_clipping));

			// u16 offset_clipping = (offset_clipping_calc < 0) ? 0 : (int)(offset_clipping_calc);
			/* Stop streaming and wait for frame data transmission done */
			mira050_write_stop_streaming_regs(mira050);
			usleep_range(wait_us, wait_us + 100);
			/* Write fine gain registers */
			printk(KERN_INFO "[MIRA050]: Write reg sequence for analog gain %u in 8 bit mode", gain);
			printk(KERN_INFO "[MIRA050]: analoggain: %u,gdig_preamp: %u rg_adcgain: %u, rg_mult: %u, offset_clipping: %u,   offset_clipping: %u\n",
				   analog_gain, gdig_preamp, rg_adcgain, rg_mult, offset_clipping, offset_clipping);
			mira050_write(mira050, MIRA050_RW_CONTEXT_REG, 0);
			mira050_write(mira050, MIRA050_BANK_SEL_REG, 1);
			mira050_write(mira050, MIRA050_GDIG_PREAMP, gdig_preamp);
			mira050_write(mira050, MIRA050_BANK_SEL_REG, 0);
			mira050_write(mira050, MIRA050_BIAS_RG_ADCGAIN, rg_adcgain);
			mira050_write(mira050, MIRA050_BIAS_RG_MULT, rg_mult);
			mira050_write_be16(mira050, MIRA050_OFFSET_CLIPPING, offset_clipping);
			/* Resume streaming */
			mira050_write_start_streaming_regs(mira050);
		}
	}
	else
	{
		// Other bit depths are not supported
		printk(KERN_INFO "[MIRA050]: Ignore analog gain in %u bit mode", mira050->mode->bit_depth);
	}

	if (ret)
	{
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
	}

	// Always return 0 even if it fails
	return 0;
}

// Gets the format code if supported. Otherwise returns the default format code `codes[0]`
static u32 mira050_validate_format_code_or_default(struct mira050 *mira050, u32 code)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira050->sd);
	unsigned int i;

	lockdep_assert_held(&mira050->mutex);

	for (i = 0; i < ARRAY_SIZE(codes); i++)
		if (codes[i] == code)
			break;

	if (i >= ARRAY_SIZE(codes))
	{
		dev_err_ratelimited(&client->dev, "Could not set requested format code %u", code);
		dev_err_ratelimited(&client->dev, "Using default format %u", codes[0]);
		i = 0;
	}

	return codes[i];
}

static void mira050_set_default_format(struct mira050 *mira050)
{
	struct v4l2_mbus_framefmt *fmt;

	fmt = &mira050->fmt;
	fmt->code = MEDIA_BUS_FMT_SGRBG12_1X12; // MEDIA_BUS_FMT_Y12_1X12;
	mira050->bit_depth = 12;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
													  fmt->colorspace,
													  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
	fmt->width = supported_modes[0].width;
	fmt->height = supported_modes[0].height;
	fmt->field = V4L2_FIELD_NONE;
}

static int mira050_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct mira050 *mira050 = to_mira050(sd);
	struct v4l2_mbus_framefmt *try_fmt_img =
		v4l2_subdev_get_try_format(sd, fh->state, IMAGE_PAD);
	struct v4l2_mbus_framefmt *try_fmt_meta =
		v4l2_subdev_get_try_format(sd, fh->state, METADATA_PAD);
	struct v4l2_rect *try_crop;

	mutex_lock(&mira050->mutex);

	/* Initialize try_fmt for the image pad */
	try_fmt_img->width = supported_modes[0].width;
	try_fmt_img->height = supported_modes[0].height;
	try_fmt_img->code = mira050_validate_format_code_or_default(mira050,
																MEDIA_BUS_FMT_SGRBG12_1X12);
	try_fmt_img->field = V4L2_FIELD_NONE;

	/* TODO(jalv): Initialize try_fmt for the embedded metadata pad */
	try_fmt_meta->width = MIRA050_EMBEDDED_LINE_WIDTH;
	try_fmt_meta->height = MIRA050_NUM_EMBEDDED_LINES;
	try_fmt_meta->code = MEDIA_BUS_FMT_SENSOR_DATA;
	try_fmt_meta->field = V4L2_FIELD_NONE;

	/* Initialize try_crop rectangle. */
	try_crop = v4l2_subdev_get_try_crop(sd, fh->state, 0);
	try_crop->top = MIRA050_PIXEL_ARRAY_TOP;
	try_crop->left = MIRA050_PIXEL_ARRAY_LEFT;
	try_crop->width = MIRA050_PIXEL_ARRAY_WIDTH;
	try_crop->height = MIRA050_PIXEL_ARRAY_HEIGHT;

	mutex_unlock(&mira050->mutex);

	return 0;
}

static int mira050_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mira050 *mira050 =
		container_of(ctrl->handler, struct mira050, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&mira050->sd);
	int ret = 0;
	u32 target_frame_time_us;

	// Debug print
	// printk(KERN_INFO "[MIRA050]: mira050_set_ctrl() id: 0x%X value: 0x%X.\n", ctrl->id, ctrl->val);

	if (ctrl->id == V4L2_CID_VBLANK)
	{
		int exposure_max, exposure_def;

		/* Update max exposure while meeting expected vblanking */
		exposure_max = mira050_calculate_max_exposure_time(MIRA050_MIN_ROW_LENGTH,
														   mira050->mode->height,
														   ctrl->val);
		exposure_def = (exposure_max < MIRA050_DEFAULT_EXPOSURE_LINES) ? exposure_max : MIRA050_DEFAULT_EXPOSURE_LINES;
		__v4l2_ctrl_modify_range(mira050->exposure,
								 mira050->exposure->minimum,
								 (int)( exposure_max ), mira050->exposure->step,
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

	if (mira050->skip_reg_upload == 0)
	{
		switch (ctrl->id)
		{
		case V4L2_CID_ANALOGUE_GAIN:
			printk(KERN_INFO "[MIRA050]: V4L2_CID_ANALOGUE_GAIN: = %u !!!!!!!!!!!!!\n",
					ctrl->val);
			ret = mira050_write_analog_gain_reg(mira050, ctrl->val);
			break;
		case V4L2_CID_EXPOSURE:
			// printk(KERN_INFO "[MIRA050]: V4L2_CID_EXPOSURE: exp line = %u, exp us = %u.\n",
			//		ctrl->val, ctrl->val * MIRA050_MIN_ROW_LENGTH_NS / 1000);
			ret = mira050_write_exposure_reg(mira050, ctrl->val);
			break;
		case V4L2_CID_TEST_PATTERN:
			ret = mira050_write(mira050, MIRA050_BANK_SEL_REG, 0);
			// Fixed data is hard coded to 0xAB.
			ret = mira050_write(mira050, MIRA050_TRAINING_WORD_REG, 0xAB);
			// Gradient is hard coded to 45 degree.
			ret = mira050_write(mira050, MIRA050_DELTA_TEST_IMG_REG, 0x01);
			ret = mira050_write(mira050, MIRA050_TEST_PATTERN_REG,
								mira050_test_pattern_val[ctrl->val]);
			break;
		case V4L2_CID_HFLIP:
			// TODO: HFLIP requires multiple register writes
			// ret = mira050_write(mira050, MIRA050_HFLIP_REG,
			//		        ctrl->val);
			break;
		case V4L2_CID_VFLIP:
			// TODO: VFLIP seems not supported in Mira050
			// ret = mira050_write(mira050, MIRA050_VFLIP_REG,
			//		        ctrl->val);
			break;
		case V4L2_CID_VBLANK:
			/*
			 * In libcamera, frame time (== 1/framerate) is controlled by VBLANK:
			 * TARGET_FRAME_TIME (us) = 1000000 * ((1/PIXEL_RATE)*(WIDTH+HBLANK)*(HEIGHT+VBLANK))
			 */
			mira050->target_frame_time_us = (u32)((u64)(1000000 * (u64)(mira050->mode->width + mira050->mode->hblank) * (u64)(mira050->mode->height + ctrl->val)) / MIRA050_PIXEL_RATE);
			// Debug print
			printk(KERN_INFO "[MIRA050]: mira050_write_target_frame_time_reg target_frame_time_us = %u.\n",
				   mira050->target_frame_time_us);
			printk(KERN_INFO "[MIRA050]: width %d, hblank %d, vblank %d, height %d, ctrl->val %d.\n",
				   mira050->mode->width, mira050->mode->hblank, mira050->mode->min_vblank, mira050->mode->height, ctrl->val);
			ret = mira050_write_target_frame_time_reg(mira050, mira050->target_frame_time_us);
			break;
		case V4L2_CID_HBLANK:
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

static int mira050_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mira050 *mira050 =
		container_of(ctrl->handler, struct mira050, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&mira050->sd);
	int ret = 0;

	// printk(KERN_INFO "[MIRA050]: mira050_s_ctrl() id: %X value: %X.\n", ctrl->id, ctrl->val);

	/* Previously, register writes when powered off will be buffered.
	 * The buffer will be written to sensor when start_streaming.
	 * Now, register writes happens immediately, even powered off.
	 * Register writes when powered off will fail.
	 * Users need to make sure first power on then write register.
	 */

	switch (ctrl->id)
	{
	case AMS_CAMERA_CID_MIRA_REG_W:
		ret = mira050_v4l2_reg_w(mira050, ctrl->val);
		break;
	default:
		dev_info(&client->dev,
				 "set ctrl(id:0x%x,val:0x%x) is not handled\n",
				 ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	// TODO: FIXIT
	return ret;
}

static int mira050_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mira050 *mira050 =
		container_of(ctrl->handler, struct mira050, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&mira050->sd);
	int ret = 0;

	// printk(KERN_INFO "[MIRA050]: mira050_g_ctrl() id: %X.\n", ctrl->id);

	/*
	 * Ideally, V4L2 register read should happen only when powered on.
	 * However, perhaps there are use cases that,
	 * reading other I2C addr is desired when mira sensor is powered off.
	 * Therefore, the check of "powered" flag is disabled for now.
	 */

	switch (ctrl->id)
	{
	case AMS_CAMERA_CID_MIRA_REG_R:
		ret = mira050_v4l2_reg_r(mira050, (u32 *)&ctrl->cur.val);
		ctrl->val = ctrl->cur.val;
		break;
	default:
		dev_info(&client->dev,
				 "get ctrl(id:0x%x) is not handled\n",
				 ctrl->id);
		ret = -EINVAL;
		break;
	}

	// TODO: FIXIT
	return ret;
}

static const struct v4l2_ctrl_ops mira050_ctrl_ops = {
	.s_ctrl = mira050_set_ctrl,
};

static const struct v4l2_ctrl_ops mira050_custom_ctrl_ops = {
	.g_volatile_ctrl = mira050_g_ctrl,
	.s_ctrl = mira050_s_ctrl,
};

/* list of custom v4l2 ctls */
static struct v4l2_ctrl_config custom_ctrl_config_list[] = {
	/* Do not change the name field for the controls! */
	{
		.ops = &mira050_custom_ctrl_ops,
		.id = AMS_CAMERA_CID_MIRA_REG_W,
		.name = "mira_reg_w",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = 0,
		.min = 0,
		.max = 0x7FFFFFFF,
		.def = 0,
		.step = 1,
	},
	{
		.ops = &mira050_custom_ctrl_ops,
		.id = AMS_CAMERA_CID_MIRA_REG_R,
		.name = "mira_reg_r",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = 0,
		.min = 0,
		.max = 0x7FFFFFFF,
		.def = 0,
		.step = 1,
	},

};

// This function should enumerate all the media bus formats for the requested pads. If the requested
// format index is beyond the number of avaialble formats it shall return -EINVAL;
static int mira050_enum_mbus_code(struct v4l2_subdev *sd,
								  struct v4l2_subdev_state *sd_state,
								  struct v4l2_subdev_mbus_code_enum *code)
{
	struct mira050 *mira050 = to_mira050(sd);

	if (code->pad >= NUM_PADS)
		return -EINVAL;

	if (code->pad == IMAGE_PAD)
	{
		if (code->index >= ARRAY_SIZE(codes))
			return -EINVAL;

		code->code = mira050_validate_format_code_or_default(mira050,
															 codes[code->index]);
	}
	else
	{
		if (code->index > 0)
			return -EINVAL;

		code->code = MEDIA_BUS_FMT_SENSOR_DATA;
	}

	return 0;
}

static int mira050_enum_frame_size(struct v4l2_subdev *sd,
								   struct v4l2_subdev_state *sd_state,
								   struct v4l2_subdev_frame_size_enum *fse)
{
	struct mira050 *mira050 = to_mira050(sd);

	if (fse->pad >= NUM_PADS)
		return -EINVAL;

	if (fse->pad == IMAGE_PAD)
	{
		/* Two options about how many modes to be exposed:
		 * - Expose all supported_modes by ARRAY_SIZE(supported_modes).
		 * - Expose less modes by MIRA050_SUPPORTED_MODE_SIZE_PUBLIC.
		 */
		/* if (fse->index >= ARRAY_SIZE(supported_modes)) */
		if (fse->index >= MIRA050_SUPPORTED_MODE_SIZE_PUBLIC)
			return -EINVAL;

		if (fse->code != mira050_validate_format_code_or_default(mira050, fse->code))
			return -EINVAL;

		fse->min_width = supported_modes[fse->index].width;
		fse->max_width = fse->min_width;
		fse->min_height = supported_modes[fse->index].height;
		fse->max_height = fse->min_height;
	}
	else
	{
		if (fse->code != MEDIA_BUS_FMT_SENSOR_DATA || fse->index > 0)
			return -EINVAL;

		fse->min_width = MIRA050_EMBEDDED_LINE_WIDTH;
		fse->max_width = fse->min_width;
		fse->min_height = MIRA050_NUM_EMBEDDED_LINES;
		fse->max_height = fse->min_height;
	}

	return 0;
}

static void mira050_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
													  fmt->colorspace,
													  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void mira050_update_image_pad_format(struct mira050 *mira050,
											const struct mira050_mode *mode,
											struct v4l2_subdev_format *fmt)
{
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	mira050_reset_colorspace(&fmt->format);
}

static void mira050_update_metadata_pad_format(struct v4l2_subdev_format *fmt)
{
	fmt->format.width = MIRA050_EMBEDDED_LINE_WIDTH;
	fmt->format.height = MIRA050_NUM_EMBEDDED_LINES;
	fmt->format.code = MEDIA_BUS_FMT_SENSOR_DATA;
	fmt->format.field = V4L2_FIELD_NONE;
}

static int __mira050_get_pad_format(struct mira050 *mira050,
									struct v4l2_subdev_state *sd_state,
									struct v4l2_subdev_format *fmt)
{
	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
	{
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(&mira050->sd, sd_state, fmt->pad);

		try_fmt->code = fmt->pad == IMAGE_PAD ? mira050_validate_format_code_or_default(mira050, try_fmt->code) : MEDIA_BUS_FMT_SENSOR_DATA;
		fmt->format = *try_fmt;
	}
	else
	{
		if (fmt->pad == IMAGE_PAD)
		{
			mira050_update_image_pad_format(mira050, mira050->mode,
											fmt);
			fmt->format.code = mira050_validate_format_code_or_default(mira050,
																	   mira050->fmt.code);
		}
		else
		{
			mira050_update_metadata_pad_format(fmt);
		}
	}

	return 0;
}

static int mira050_get_pad_format(struct v4l2_subdev *sd,
								  struct v4l2_subdev_state *sd_state,
								  struct v4l2_subdev_format *fmt)
{
	struct mira050 *mira050 = to_mira050(sd);
	int ret;

	mutex_lock(&mira050->mutex);
	ret = __mira050_get_pad_format(mira050, sd_state, fmt);
	mutex_unlock(&mira050->mutex);

	return ret;
}

static int mira050_set_pad_format(struct v4l2_subdev *sd,
								  struct v4l2_subdev_state *sd_state,
								  struct v4l2_subdev_format *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct mira050 *mira050 = to_mira050(sd);
	const struct mira050_mode *mode;
	struct v4l2_mbus_framefmt *framefmt;
	u32 max_exposure = 0, default_exp = 0;
	int rc = 0;

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&mira050->mutex);

	if (fmt->pad == IMAGE_PAD)
	{
		/* Validate format or use default */
		fmt->format.code = mira050_validate_format_code_or_default(mira050,
																   fmt->format.code);

		switch (fmt->format.code)
		{
		case MEDIA_BUS_FMT_SGRBG10_1X10:
			printk(KERN_INFO "[MIRA050]: fmt->format.code() selects 10 bit mode.\n");
			mira050->mode = &supported_modes[1];
			mira050->bit_depth = 10;
			// return 0;
			break;

		case MEDIA_BUS_FMT_SGRBG12_1X12:
			printk(KERN_INFO "[MIRA050]: fmt->format.code() selects 12 bit mode.\n");
			mira050->mode = &supported_modes[0];
			mira050->bit_depth = 12;
			// return 0;
			break;

		case MEDIA_BUS_FMT_SGRBG8_1X8:
			printk(KERN_INFO "[MIRA050]: fmt->format.code() selects 8 bit mode.\n");
			mira050->mode = &supported_modes[2];
			mira050->bit_depth = 8;
			// return 0;
			break;
		default:
			printk(KERN_ERR "Unknown format requested fmt->format.code() %d", fmt->format.code);
		}
		mode = v4l2_find_nearest_size(supported_modes,
									  ARRAY_SIZE(supported_modes),
									  width, height,
									  fmt->format.width,
									  fmt->format.height);
		mira050_update_image_pad_format(mira050, mode, fmt);
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		{
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
												  fmt->pad);
			*framefmt = fmt->format;
		}
		else if (mira050->mode != mode ||
				 mira050->fmt.code != fmt->format.code)
		{

			mira050->fmt = fmt->format;
			// mira050->mode = mode;

			// Update controls based on new mode (range and current value).
			max_exposure = mira050_calculate_max_exposure_time(MIRA050_MIN_ROW_LENGTH,
															   mira050->mode->height,
															   mira050->mode->min_vblank);

			default_exp = MIRA050_DEFAULT_EXPOSURE_LINES > max_exposure ? max_exposure : MIRA050_DEFAULT_EXPOSURE_LINES;
			rc = __v4l2_ctrl_modify_range(mira050->exposure,
										  mira050->exposure->minimum,
										  (int)(1 + max_exposure), mira050->exposure->step,
										  (int)(1 + default_exp));
			if (rc)
			{
				dev_err(&client->dev, "Error setting exposure range");
			}

			printk(KERN_INFO "[MIRA050]: Mira050 SETTING ANA GAIN RANGE  = %u.\n",
				   ARRAY_SIZE(fine_gain_lut_8bit_16x) - 1);
			// #FIXME #TODO
			//  rc = __v4l2_ctrl_modify_range(mira050->gain,
			//  					 0, ARRAY_SIZE(fine_gain_lut_8bit_16x) - 1, 1, 0);
			rc = __v4l2_ctrl_modify_range(mira050->gain,
										  mira050->mode->gain_min,
										  mira050->mode->gain_max,
										  1,
										  0);
			if (rc)
			{
				dev_err(&client->dev, "Error setting gain range");
			}

			printk(KERN_INFO "[MIRA050]: Mira050 VBLANK  = %u.\n",
				   mira050->mode->min_vblank);

			rc = __v4l2_ctrl_modify_range(mira050->vblank,
										  mira050->mode->min_vblank,
										  mira050->mode->max_vblank,
										  1,
										  MIRA050_MIN_VBLANK_60);
			if (rc)
			{
				dev_err(&client->dev, "Error setting exposure range");
			}
			// Set the current vblank value
			rc = __v4l2_ctrl_s_ctrl(mira050->vblank, MIRA050_MIN_VBLANK_60);
			if (rc)
			{
				dev_err(&client->dev, "Error setting vblank value to %u",
						mira050->mode->min_vblank);
			}
		}
	}
	else
	{
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		{
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
												  fmt->pad);
			*framefmt = fmt->format;
		}
		else
		{
			/* Only one embedded data mode is supported */
			mira050_update_metadata_pad_format(fmt);
		}
	}

	mutex_unlock(&mira050->mutex);

	return 0;
}

static int mira050_set_framefmt(struct mira050 *mira050)
{
	// TODO: There is no easy way to change frame format
	switch (mira050->fmt.code)
	{
	case MEDIA_BUS_FMT_SGRBG8_1X8:
		printk(KERN_INFO "[MIRA050]: mira050_set_framefmt() selects 8 bit mode.\n");
		mira050->mode = &supported_modes[2];
		mira050->bit_depth = 8;
		__v4l2_ctrl_modify_range(mira050->gain,
								 0, ARRAY_SIZE(fine_gain_lut_8bit_16x) - 1, 1, 0);
		return 0;
	case MEDIA_BUS_FMT_SGRBG10_1X10:
		printk(KERN_INFO "[MIRA050]: mira050_set_framefmt() selects 10 bit mode.\n");
		mira050->mode = &supported_modes[1];
		mira050->bit_depth = 10;
		__v4l2_ctrl_modify_range(mira050->gain,
								 0, ARRAY_SIZE(fine_gain_lut_10bit_hs_4x) - 1, 1, 0);
		return 0;
	case MEDIA_BUS_FMT_SGRBG12_1X12:
		printk(KERN_INFO "[MIRA050]: mira050_set_framefmt() selects 12 bit mode.\n");
		mira050->mode = &supported_modes[0];
		mira050->bit_depth = 12;
		__v4l2_ctrl_modify_range(mira050->gain,
								 mira050->mode->gain_min, mira050->mode->gain_max,
								 MIRA050_ANALOG_GAIN_STEP, MIRA050_ANALOG_GAIN_DEFAULT);
		return 0;
	default:
		printk(KERN_ERR "Unknown format requested %d", mira050->fmt.code);
	}

	return -EINVAL;
}

static const struct v4l2_rect *
__mira050_get_pad_crop(struct mira050 *mira050, struct v4l2_subdev_state *sd_state,
					   unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which)
	{
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&mira050->sd, sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &mira050->mode->crop;
	}

	return NULL;
}

static int mira050_get_selection(struct v4l2_subdev *sd,
								 struct v4l2_subdev_state *sd_state,
								 struct v4l2_subdev_selection *sel)
{
	switch (sel->target)
	{
	case V4L2_SEL_TGT_CROP:
	{
		struct mira050 *mira050 = to_mira050(sd);

		mutex_lock(&mira050->mutex);
		sel->r = *__mira050_get_pad_crop(mira050, sd_state, sel->pad,
										 sel->which);
		mutex_unlock(&mira050->mutex);

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = MIRA050_NATIVE_WIDTH;
		sel->r.height = MIRA050_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = MIRA050_PIXEL_ARRAY_TOP;
		sel->r.left = MIRA050_PIXEL_ARRAY_LEFT;
		sel->r.width = MIRA050_PIXEL_ARRAY_WIDTH;
		sel->r.height = MIRA050_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

static int mira050_start_streaming(struct mira050 *mira050)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira050->sd);
	const struct mira050_reg_list *reg_list;

	u32 otp_dark_cal_8bit;
	u32 otp_dark_cal_10bit_hs;
	u32 otp_dark_cal_10bit;
	u32 otp_dark_cal_12bit;
	int ret;

	printk(KERN_INFO "[MIRA050]: Entering START STREAMING function !!!!!!!!!!.\n");

	/* Follow examples of other camera driver, here use pm_runtime_resume_and_get */
	ret = pm_runtime_resume_and_get(&client->dev);

	if (ret < 0)
	{
		printk(KERN_INFO "[MIRA050]: get_sync failed, but continue.\n");
		pm_runtime_put_noidle(&client->dev);
		return ret;
	}

	/* Set current mode according to frame format bit depth */
	ret = mira050_set_framefmt(mira050);
	if (ret)
	{
		dev_err(&client->dev, "%s failed to set frame format: %d\n",
				__func__, ret);
		goto err_rpm_put;
	}
	printk(KERN_INFO "[MIRA050]: Register sequence for %d bit mode will be used.\n", mira050->mode->bit_depth);

	if (mira050->skip_reg_upload == 0)
	{
		/* Apply pre soft reset default values of current mode */
		reg_list = &mira050->mode->reg_list_pre_soft_reset;
		printk(KERN_INFO "[MIRA050]: Write %d regs.\n", reg_list->num_of_regs);
		ret = mira050_write_regs(mira050, reg_list->regs, reg_list->num_of_regs);
		if (ret)
		{
			dev_err(&client->dev, "%s failed to set mode\n", __func__);
			goto err_rpm_put;
		}

		usleep_range(10, 50);

		/* Apply post soft reset default values of current mode */
		reg_list = &mira050->mode->reg_list_post_soft_reset;
		printk(KERN_INFO "[MIRA050]: Write %d regs.\n", reg_list->num_of_regs);
		ret = mira050_write_regs(mira050, reg_list->regs, reg_list->num_of_regs);
		if (ret)
		{
			dev_err(&client->dev, "%s failed to set mode\n", __func__);
			goto err_rpm_put;
		}
	}
	else
	{
		printk(KERN_INFO "[MIRA050]: Skip base register sequence upload, due to mira050->skip_reg_upload=%u.\n", mira050->skip_reg_upload);
	}

	printk(KERN_INFO "[MIRA050]: Entering v4l2 ctrl handler setup function.\n");

	/* Apply customized values from user */
	ret = __v4l2_ctrl_handler_setup(mira050->sd.ctrl_handler);
	printk(KERN_INFO "[MIRA050]: __v4l2_ctrl_handler_setup ret = %d.\n", ret);
	if (ret)
		goto err_rpm_put;




	usleep_range(10, 50);
	/* ********* READ OTP VALUES for revB - all modes ********** */
	ret = mira050_otp_read(mira050, 0x04, &otp_dark_cal_8bit);
	/* OTP_CALIBRATION_VALUE is little-endian, LSB at [7:0], MSB at [15:8] */
	mira050->otp_dark_cal_8bit = (u16)(otp_dark_cal_8bit & 0x0000FFFF);
	if (ret)
	{
		dev_err(&client->dev, "%s failed to read OTP addr 0x01.\n", __func__);
	}
	else
	{
		printk(KERN_INFO "[MIRA050]: OTP_CALIBRATION_VALUE 8b: %u, extracted from 32-bit 0x%X.\n", mira050->otp_dark_cal_8bit, otp_dark_cal_8bit);
	}
	ret = mira050_otp_read(mira050, 0x05, &otp_dark_cal_10bit_hs);
	/* OTP_CALIBRATION_VALUE is little-endian, LSB at [7:0], MSB at [15:8] */
	mira050->otp_dark_cal_10bit_hs = (u16)(otp_dark_cal_10bit_hs & 0x0000FFFF);
	if (ret)
	{
		dev_err(&client->dev, "%s failed to read OTP addr 0x01.\n", __func__);
	}
	else
	{
		printk(KERN_INFO "[MIRA050]: OTP_CALIBRATION_VALUE 10b hs: %u, extracted from 32-bit 0x%X.\n", mira050->otp_dark_cal_10bit_hs, otp_dark_cal_10bit_hs);
	}
	ret = mira050_otp_read(mira050, 0x06, &otp_dark_cal_10bit);
	/* OTP_CALIBRATION_VALUE is little-endian, LSB at [7:0], MSB at [15:8] */
	mira050->otp_dark_cal_10bit = (u16)(otp_dark_cal_10bit & 0x0000FFFF);
	if (ret)
	{
		dev_err(&client->dev, "%s failed to read OTP addr 0x01.\n", __func__);
	}
	else
	{
		printk(KERN_INFO "[MIRA050]: OTP_CALIBRATION_VALUE 10b: %u, extracted from 32-bit 0x%X.\n", mira050->otp_dark_cal_10bit, otp_dark_cal_10bit);
	}
	// 12 bit
	usleep_range(10, 50);
	ret = mira050_otp_read(mira050, 0x07, &otp_dark_cal_12bit);
	/* OTP_CALIBRATION_VALUE is little-endian, LSB at [7:0], MSB at [15:8] */
	mira050->otp_dark_cal_12bit = (u16)(otp_dark_cal_12bit & 0x0000FFFF);

	if (ret)
	{
		dev_err(&client->dev, "%s failed to read OTP addr 0x07.\n", __func__);
	}
	else
	{
		printk(KERN_INFO "[MIRA050]: OTP_CALIBRATION_VALUE 12b: %u, extracted from 32-bit 0x%X.\n", mira050->otp_dark_cal_12bit, otp_dark_cal_12bit);
	}


	// ret = mira050_write_analog_gain_reg(mira050, 0);
	if (mira050->skip_reg_upload == 0 ||
		(mira050->skip_reg_upload == 1 && mira050->force_stream_ctrl == 1))
	{
		printk(KERN_INFO "[MIRA050]: Writing start streaming regs.\n");
		ret = mira050_write_start_streaming_regs(mira050);
		if (ret)
		{
			dev_err(&client->dev, "Could not write stream-on sequence");
			goto err_rpm_put;
		}
	}
	else
	{
		printk(KERN_INFO "[MIRA050]: Skip write_start_streaming_regs due to skip_reg_upload == %d and force_stream_ctrl == %d.\n",
			   mira050->skip_reg_upload, mira050->force_stream_ctrl);
	}

	/* vflip and hflip cannot change during streaming */
	printk(KERN_INFO "[MIRA050]: Entering v4l2 ctrl grab vflip grab vflip.\n");
	__v4l2_ctrl_grab(mira050->vflip, true);
	printk(KERN_INFO "[MIRA050]: Entering v4l2 ctrl grab vflip grab hflip.\n");
	__v4l2_ctrl_grab(mira050->hflip, true);

	return 0;

err_rpm_put:
	pm_runtime_put(&client->dev);
	return ret;
}

static void mira050_stop_streaming(struct mira050 *mira050)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira050->sd);
	int ret = 0;

	/* Unlock controls for vflip and hflip */
	__v4l2_ctrl_grab(mira050->vflip, false);
	__v4l2_ctrl_grab(mira050->hflip, false);

	if (mira050->skip_reset == 0)
	{
		if (mira050->skip_reg_upload == 0 ||
			(mira050->skip_reg_upload == 1 && mira050->force_stream_ctrl == 1))
		{
			printk(KERN_INFO "[MIRA050]: Writing stop streaming regs.\n");
			ret = mira050_write_stop_streaming_regs(mira050);
			if (ret)
			{
				dev_err(&client->dev, "Could not write the stream-off sequence");
			}
		}
		else
		{
			printk(KERN_INFO "[MIRA050]: Skip write_stop_streaming_regs due to skip_reg_upload == %d and force_stream_ctrl == %d.\n",
				   mira050->skip_reg_upload, mira050->force_stream_ctrl);
		}
	}
	else
	{
		printk(KERN_INFO "[MIRA050]: Skip write_stop_streaming_regs due to mira050->skip_reset == %d.\n", mira050->skip_reset);
	}

	pm_runtime_put(&client->dev);
}

static int mira050_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct mira050 *mira050 = to_mira050(sd);
	int ret = 0;

	mutex_lock(&mira050->mutex);
	if (mira050->streaming == enable)
	{
		mutex_unlock(&mira050->mutex);
		return 0;
	}

	printk(KERN_INFO "[MIRA050]: Entering mira050_set_stream enable: %d.\n", enable);

	if (enable)
	{
		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = mira050_start_streaming(mira050);
		if (ret)
			goto err_unlock;
	}
	else
	{
		mira050_stop_streaming(mira050);
	}

	mira050->streaming = enable;

	mutex_unlock(&mira050->mutex);

	printk(KERN_INFO "[MIRA050]: Returning mira050_set_stream with ret: %d.\n", ret);

	return ret;

err_unlock:
	mutex_unlock(&mira050->mutex);

	return ret;
}

static int __maybe_unused mira050_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira050 *mira050 = to_mira050(sd);

	printk(KERN_INFO "[MIRA050]: Entering suspend function.\n");

	if (mira050->streaming)
		mira050_stop_streaming(mira050);

	return 0;
}

static int __maybe_unused mira050_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira050 *mira050 = to_mira050(sd);
	int ret;

	printk(KERN_INFO "[MIRA050]: Entering resume function.\n");

	if (mira050->streaming)
	{
		ret = mira050_start_streaming(mira050);
		if (ret)
			goto error;
	}

	return 0;

error:
	mira050_stop_streaming(mira050);
	mira050->streaming = false;

	return ret;
}

static int mira050_get_regulators(struct mira050 *mira050)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira050->sd);
	unsigned int i;

	for (i = 0; i < MIRA050_NUM_SUPPLIES; i++)
		mira050->supplies[i].supply = mira050_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
								   MIRA050_NUM_SUPPLIES,
								   mira050->supplies);
}

/* Verify chip ID */
static int mira050_identify_module(struct mira050 *mira050)
{
	int ret;
	u8 val;

	ret = mira050_read(mira050, 0x25, &val);
	printk(KERN_INFO "[MIRA050]: Read reg 0x%4.4x, val = 0x%x.\n",
		   0x25, val);
	ret = mira050_read(mira050, 0x3, &val);
	printk(KERN_INFO "[MIRA050]: Read reg 0x%4.4x, val = 0x%x.\n",
		   0x3, val);
	ret = mira050_read(mira050, 0x4, &val);
	printk(KERN_INFO "[MIRA050]: Read reg 0x%4.4x, val = 0x%x.\n",
		   0x4, val);

	return 0;
}

static const struct v4l2_subdev_core_ops mira050_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops mira050_video_ops = {
	.s_stream = mira050_set_stream,
};

static const struct v4l2_subdev_pad_ops mira050_pad_ops = {
	.enum_mbus_code = mira050_enum_mbus_code,
	.get_fmt = mira050_get_pad_format,
	.set_fmt = mira050_set_pad_format,
	.get_selection = mira050_get_selection,
	.enum_frame_size = mira050_enum_frame_size,
};

static const struct v4l2_subdev_ops mira050_subdev_ops = {
	.core = &mira050_core_ops,
	.video = &mira050_video_ops,
	.pad = &mira050_pad_ops,
};

static const struct v4l2_subdev_internal_ops mira050_internal_ops = {
	.open = mira050_open,
};

/* Initialize control handlers */
static int mira050_init_controls(struct mira050 *mira050)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira050->sd);
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct v4l2_fwnode_device_properties props;
	int ret;
	struct v4l2_ctrl_config *mira050_reg_w;
	struct v4l2_ctrl_config *mira050_reg_r;

	ctrl_hdlr = &mira050->ctrl_handler;
	/* v4l2_ctrl_handler_init gives a hint/guess of the number of v4l2_ctrl_new */
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 16);
	if (ret)
		return ret;

	mutex_init(&mira050->mutex);
	ctrl_hdlr->lock = &mira050->mutex;

	printk(KERN_INFO "[MIRA050]: %s V4L2_CID_PIXEL_RATE %X.\n", __func__, V4L2_CID_PIXEL_RATE);
	printk(KERN_INFO "[MIRA050]: %s INIT_CONTROLS bitmode %X.\n", __func__, mira050->mode->bit_depth);

	/* By default, PIXEL_RATE is read only */
	mira050->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &mira050_ctrl_ops,
											V4L2_CID_PIXEL_RATE,
											MIRA050_PIXEL_RATE,
											MIRA050_PIXEL_RATE, 1,
											MIRA050_PIXEL_RATE);

	printk(KERN_INFO "[MIRA050]: %s V4L2_CID_VBLANK %X.\n", __func__, V4L2_CID_VBLANK);

	mira050->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &mira050_ctrl_ops,
										V4L2_CID_VBLANK, mira050->mode->min_vblank,
										mira050->mode->max_vblank, 1,
										MIRA050_MIN_VBLANK_60);

	printk(KERN_INFO "[MIRA050]: %s V4L2_CID_HBLANK %X.\n", __func__, V4L2_CID_HBLANK);

	mira050->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &mira050_ctrl_ops,
										V4L2_CID_HBLANK, mira050->mode->hblank,
										mira050->mode->hblank, 1,
										mira050->mode->hblank);

	// Make the vblank control read only. This could be changed to allow changing framerate in
	// runtime, but would require adapting other settings
	// mira050->vblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	// Exposure is indicated in number of lines here
	// Max is determined by vblank + vsize and Tglob.
	printk(KERN_INFO "[MIRA050]: %s V4L2_CID_EXPOSURE %X.\n", __func__, V4L2_CID_EXPOSURE);
	mira050->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &mira050_ctrl_ops,
										  V4L2_CID_EXPOSURE,
										  MIRA050_EXPOSURE_MIN_US/10, MIRA050_EXPOSURE_MAX_US/10,
										  1,
										  MIRA050_DEFAULT_EXPOSURE_LINES);
	printk(KERN_INFO "[MIRA050]: %s V4L2_CID_ANALOGUE_GAIN %X.\n", __func__, V4L2_CID_ANALOGUE_GAIN);

	mira050->gain = v4l2_ctrl_new_std(ctrl_hdlr, &mira050_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
									  mira050->mode->gain_min, mira050->mode->gain_max,
									  MIRA050_ANALOG_GAIN_STEP, MIRA050_ANALOG_GAIN_DEFAULT);

	printk(KERN_INFO "[MIRA050]: %s V4L2_CID_HFLIP %X.\n", __func__, V4L2_CID_HFLIP);

	mira050->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &mira050_ctrl_ops,
									   V4L2_CID_HFLIP, 0, 0, 1, 0);
	if (mira050->hflip)
		mira050->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	printk(KERN_INFO "[MIRA050]: %s V4L2_CID_VFLIP %X.\n", __func__, V4L2_CID_VFLIP);

	mira050->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &mira050_ctrl_ops,
									   V4L2_CID_VFLIP, 0, 0, 1, 0);
	if (mira050->vflip)
		mira050->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	printk(KERN_INFO "[MIRA050]: %s V4L2_CID_TEST_PATTERN %X.\n", __func__, V4L2_CID_TEST_PATTERN);
	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &mira050_ctrl_ops,
								 V4L2_CID_TEST_PATTERN,
								 ARRAY_SIZE(mira050_test_pattern_menu) - 1,
								 0, 0, mira050_test_pattern_menu);

	/*
	 * Custom op
	 */
	mira050_reg_w = &custom_ctrl_config_list[0];
	printk(KERN_INFO "[MIRA050]: %s AMS_CAMERA_CID_MIRA_REG_W %X.\n", __func__, AMS_CAMERA_CID_MIRA_REG_W);
	mira050->mira050_reg_w = v4l2_ctrl_new_custom(ctrl_hdlr, mira050_reg_w, NULL);

	mira050_reg_r = &custom_ctrl_config_list[1];
	printk(KERN_INFO "[MIRA050]: %s AMS_CAMERA_CID_MIRA_REG_R %X.\n", __func__, AMS_CAMERA_CID_MIRA_REG_R);
	mira050->mira050_reg_r = v4l2_ctrl_new_custom(ctrl_hdlr, mira050_reg_r, NULL);
	if (mira050->mira050_reg_r)
		mira050->mira050_reg_r->flags |= (V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY);

	if (ctrl_hdlr->error)
	{
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
				__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &mira050_ctrl_ops,
										  &props);
	if (ret)
		goto error;

	mira050->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&mira050->mutex);

	return ret;
}

static void mira050_free_controls(struct mira050 *mira050)
{
	v4l2_ctrl_handler_free(mira050->sd.ctrl_handler);
	mutex_destroy(&mira050->mutex);
}

static int mira050_check_hwcfg(struct device *dev)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY};
	int ret = -EINVAL;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint)
	{
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg))
	{
		dev_err(dev, "could not parse endpoint\n");
		goto error_out;
	}

	/* Check the number of MIPI CSI2 data lanes */
	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 1)
	{
		dev_err(dev, "only 1 data lanes are currently supported\n");
		goto error_out;
	}

	/* Check the link frequency set in device tree */
	if (!ep_cfg.nr_of_link_frequencies)
	{
		dev_err(dev, "link-frequency property not found in DT\n");
		goto error_out;
	}

	if (ep_cfg.nr_of_link_frequencies != 1 ||
		ep_cfg.link_frequencies[0] != MIRA050_DEFAULT_LINK_FREQ)
	{
		dev_err(dev, "Link frequency not supported: %lld\n",
				ep_cfg.link_frequencies[0]);
		goto error_out;
	}

	// TODO(jalv): Check device tree configuration and make sure it is supported by the driver
	ret = 0;

error_out:
	v4l2_fwnode_endpoint_free(&ep_cfg);
	fwnode_handle_put(endpoint);

	return ret;
}

static int mira050pmic_init_controls(struct i2c_client *pmic_client, struct i2c_client *uc_client)
{
	int ret;
	u8 val;

	// uC, set atb and jtag high
	// according to old uC fw (svn rev41)
	// 12[3] ldo en
	// 11[4,5] atpg jtag
	// 11/12 i/o direction, 15/16 output high/low
	// uC, set atb and jtag high
	// WARNING this only works on interposer v2 if R307 is not populated. otherwise, invert the bit for ldo
	ret = mira050pmic_write(uc_client, 12, 0xF7);
	ret = mira050pmic_write(uc_client, 16, 0xFF); // ldo en:1
	ret = mira050pmic_write(uc_client, 11, 0XCF);
	ret = mira050pmic_write(uc_client, 15, 0xFF);
	ret = mira050pmic_write(uc_client, 6, 1); // write

	// Disable master switch //
	ret = mira050pmic_write(pmic_client, 0x62, 0x00);

	// Set all voltages to 0

	// DCDC1=0V
	ret = mira050pmic_write(pmic_client, 0x05, 0x00);
	// DCDC4=0V
	ret = mira050pmic_write(pmic_client, 0x0E, 0x0);
	// LDO1=0V VDDLO_PLL
	ret = mira050pmic_write(pmic_client, 0x11, 0x0);
	// LDO2=0.0V
	ret = mira050pmic_write(pmic_client, 0x14, 0x00);
	// LDO3=0.0V
	ret = mira050pmic_write(pmic_client, 0x17, 0x00);
	// LDO4=0V
	ret = mira050pmic_write(pmic_client, 0x1A, 0x00);
	// LDO5=0.0V
	ret = mira050pmic_write(pmic_client, 0x1C, 0x00);
	// LDO6=0.0V
	ret = mira050pmic_write(pmic_client, 0x1D, 0x00);
	// LDO7=0V
	ret = mira050pmic_write(pmic_client, 0x1E, 0x0);
	// LDO8=0.0V
	ret = mira050pmic_write(pmic_client, 0x1F, 0x00);
	// Disable LDO9 Lock
	ret = mira050pmic_write(pmic_client, 0x24, 0x48);
	// LDO9=0V VDDHI
	ret = mira050pmic_write(pmic_client, 0x20, 0x00);
	// LDO10=0V VDDLO_ANA
	ret = mira050pmic_write(pmic_client, 0x21, 0x0);

	// Enable master switch //
	usleep_range(50, 60);
	ret = mira050pmic_write(pmic_client, 0x62, 0x0D); // enable master switch
	usleep_range(50, 60);

	// start PMIC
	// Keep LDOs always on
	ret = mira050pmic_write(pmic_client, 0x27, 0xFF);
	ret = mira050pmic_write(pmic_client, 0x28, 0xFF);
	ret = mira050pmic_write(pmic_client, 0x29, 0x00);
	ret = mira050pmic_write(pmic_client, 0x2A, 0x00);
	ret = mira050pmic_write(pmic_client, 0x2B, 0x00);

	// Unused LDO off //
	usleep_range(50, 60);
	// set GPIO1=0
	ret = mira050pmic_write(pmic_client, 0x41, 0x04);
	// DCDC2=0.0V SPARE_PWR1
	ret = mira050pmic_write(pmic_client, 0x01, 0x00);
	ret = mira050pmic_write(pmic_client, 0x08, 0x00);
	// DCDC3=0V SPARE_PWR1
	ret = mira050pmic_write(pmic_client, 0x02, 0x00);
	ret = mira050pmic_write(pmic_client, 0x0B, 0x00);
	// LDO2=0.0V
	ret = mira050pmic_write(pmic_client, 0x14, 0x00);
	// LDO3=0.0V
	ret = mira050pmic_write(pmic_client, 0x17, 0x00);
	// LDO5=0.0V
	ret = mira050pmic_write(pmic_client, 0x1C, 0x00);
	// LDO6=0.0V
	ret = mira050pmic_write(pmic_client, 0x1D, 0x00);
	// LDO8=0.0V
	ret = mira050pmic_write(pmic_client, 0x1F, 0x00);

	ret = mira050pmic_write(pmic_client, 0x42, 4);

	// Enable 1.80V //
	usleep_range(50, 60);
	// DCDC1=1.8V VINLDO1p8 >=1P8
	ret = mira050pmic_write(pmic_client, 0x00, 0x00);
	ret = mira050pmic_write(pmic_client, 0x04, 0x34);
	ret = mira050pmic_write(pmic_client, 0x06, 0xBF);
	ret = mira050pmic_write(pmic_client, 0x05, 0xB4);
	// DCDC4=1.8V VDDIO
	ret = mira050pmic_write(pmic_client, 0x03, 0x00);
	ret = mira050pmic_write(pmic_client, 0x0D, 0x34);
	ret = mira050pmic_write(pmic_client, 0x0F, 0xBF);
	ret = mira050pmic_write(pmic_client, 0x0E, 0xB4);

	// Enable 2.85V //
	usleep_range(50, 60);
	// LDO4=2.85V VDDHI alternativ
	ret = mira050pmic_write(pmic_client, 0x1A, 0xB8); // Either 0x00 or 0xB8
	// Disable LDO9 Lock
	ret = mira050pmic_write(pmic_client, 0x24, 0x48);
	// LDO9=2.85V VDDHI
	ret = mira050pmic_read(pmic_client, 0x20, &val);
	dev_err(&pmic_client->dev, "Read 0x20 with val %x\n", val);
	ret = mira050pmic_write(pmic_client, 0x20, 0xB9);
	ret = mira050pmic_read(pmic_client, 0x20, &val);
	dev_err(&pmic_client->dev, "Read 0x20 with val %x\n", val);

	// VPIXH on cob = vdd25A on interposer = LDO4 on pmic
	// VPIXH should connect to VDD28 on pcb, or enable 4th supply
	ret = mira050pmic_read(pmic_client, 0x19, &val);
	dev_err(&pmic_client->dev, "Read 0x19 with val %x\n", val);
	ret = mira050pmic_write(pmic_client, 0x19, 0x38);
	ret = mira050pmic_read(pmic_client, 0x19, &val);
	dev_err(&pmic_client->dev, "Read 0x19 with val %x\n", val);

	// Enable 1.2V //
	usleep_range(700, 710);
	// LDO1=1.2V VDDLO_PLL
	ret = mira050pmic_write(pmic_client, 0x12, 0x16);
	ret = mira050pmic_write(pmic_client, 0x10, 0x16);
	ret = mira050pmic_write(pmic_client, 0x11, 0x90);
	// LDO7=1.2V VDDLO_DIG
	ret = mira050pmic_write(pmic_client, 0x1E, 0x90);
	// LDO10=1.2V VDDLO_ANA
	ret = mira050pmic_write(pmic_client, 0x21, 0x90);

	// Enable green LED //
	usleep_range(50, 60);
	ret = mira050pmic_write(pmic_client, 0x42, 0x15); // gpio2
	// ret = mira050pmic_write(pmic_client, 0x43, 0x40); // leda
	// ret = mira050pmic_write(pmic_client, 0x44, 0x40); // ledb
	ret = mira050pmic_write(pmic_client, 0x45, 0x40); // ledc

	// ret = mira050pmic_write(pmic_client, 0x47, 0x02); // leda ctrl1
	// ret = mira050pmic_write(pmic_client, 0x4F, 0x02); // ledb ctrl1
	ret = mira050pmic_write(pmic_client, 0x57, 0x02); // ledc ctrl1

	// ret = mira050pmic_write(pmic_client, 0x4D, 0x01); // leda ctrl1
	// ret = mira050pmic_write(pmic_client, 0x55, 0x10); // ledb ctrl7
	ret = mira050pmic_write(pmic_client, 0x5D, 0x10); // ledc ctrl7
	ret = mira050pmic_write(pmic_client, 0x61, 0x10); // led seq -- use this to turn on leds. abc0000- 1110000 for all leds

	// uC, set atb and jtag high and ldo_en
	ret = mira050pmic_write(uc_client, 12, 0xF7);
	ret = mira050pmic_write(uc_client, 16, 0xF7); // ldo en:0
	/*
	 * In Mira050-bringup.py, write 11, 0xCF; 15: 0x30.
	 * In mira050.py, write 11, 0x8D; 15, 0xFD.
	 */
	ret = mira050pmic_write(uc_client, 11, 0X8D);
	ret = mira050pmic_write(uc_client, 15, 0xFD);
	ret = mira050pmic_write(uc_client, 6, 1); // write

	usleep_range(2000000, 2001000);

	return 0;
}

static int mira050_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct mira050 *mira050;
	int ret;

	printk(KERN_INFO "[MIRA050]: probing v4l2 sensor.\n");
	printk(KERN_INFO "[MIRA050]: Driver Version 0.0.\n");

	dev_err(dev, "[MIRA050] name: %s.\n", client->name);

	mira050 = devm_kzalloc(&client->dev, sizeof(*mira050), GFP_KERNEL);
	if (!mira050)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&mira050->sd, client, &mira050_subdev_ops);

	/* Check the hardware configuration in device tree */
	if (mira050_check_hwcfg(dev))
		return -EINVAL;

	/* Parse device tree to check if dtoverlay has param skip-reg-upload=1 */
	device_property_read_u32(dev, "skip-reg-upload", &mira050->skip_reg_upload);
	printk(KERN_INFO "[MIRA050]: skip-reg-upload %d.\n", mira050->skip_reg_upload);
	/* Set default TBD I2C device address to LED I2C Address*/
	mira050->tbd_client_i2c_addr = MIRA050LED_I2C_ADDR;
	printk(KERN_INFO "[MIRA050]: User defined I2C device address defaults to LED driver I2C address 0x%X.\n", mira050->tbd_client_i2c_addr);

	/* Get system clock (xclk) */
	mira050->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(mira050->xclk))
	{
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(mira050->xclk);
	}

	mira050->xclk_freq = clk_get_rate(mira050->xclk);
	if (mira050->xclk_freq != MIRA050_SUPPORTED_XCLK_FREQ)
	{
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
				mira050->xclk_freq);
		return -EINVAL;
	}

	ret = mira050_get_regulators(mira050);
	if (ret)
	{
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	{
		printk(KERN_INFO "[MIRA050]: Init PMIC and uC and led driver.\n");
		mira050->pmic_client = i2c_new_dummy_device(client->adapter,
													MIRA050PMIC_I2C_ADDR);
		if (IS_ERR(mira050->pmic_client))
			return PTR_ERR(mira050->pmic_client);
		mira050->uc_client = i2c_new_dummy_device(client->adapter,
												  MIRA050UC_I2C_ADDR);
		if (IS_ERR(mira050->uc_client))
			return PTR_ERR(mira050->uc_client);
		mira050->led_client = i2c_new_dummy_device(client->adapter,
												   MIRA050LED_I2C_ADDR);
		if (IS_ERR(mira050->led_client))
			return PTR_ERR(mira050->led_client);
		mira050pmic_init_controls(mira050->pmic_client, mira050->uc_client);
	}

	dev_err(dev, "[MIRA050] Sleep for 1 second to let PMIC driver complete init.\n");
	usleep_range(1000000, 1000000 + 100);

	/*
	 * The sensor must be powered for mira050_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = mira050_power_on(dev);
	if (ret)
		return ret;

	printk(KERN_INFO "[MIRA050]: Entering identify function.\n");

	ret = mira050_identify_module(mira050);
	if (ret)
		goto error_power_off;

	printk(KERN_INFO "[MIRA050]: Setting support function.\n");

	/* Initialize default illumination trigger parameters */
	/* ILLUM_WIDTH is in unit of SEQ_TIME_BASE, equal to (8/1000) us. */
	mira050->illum_width = MIRA050_ILLUM_WIDTH_DEFAULT;
	/* ILLUM_DELAY is in unit of TIME_UNIT, equal to 1 us. In continuous stream mode, zero delay is 1<<19. */
	mira050->illum_delay = MIRA050_ILLUM_DELAY_DEFAULT;
	mira050->illum_width_auto = MIRA050_ILLUM_WIDTH_AUTO_DEFAULT;
	mira050->illum_enable = MIRA050_ILLUM_ENABLE_DEFAULT;

	/* Set default mode to max resolution */
	mira050->mode = &supported_modes[0];

	printk(KERN_INFO "[MIRA050]: Entering init controls function.\n");

	ret = mira050_init_controls(mira050);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	mira050->sd.internal_ops = &mira050_internal_ops;
	mira050->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
						 V4L2_SUBDEV_FL_HAS_EVENTS;
	mira050->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pads */
	mira050->pad[IMAGE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	mira050->pad[METADATA_PAD].flags = MEDIA_PAD_FL_SOURCE;

	printk(KERN_INFO "[MIRA050]: Entering set default format function.\n");

	/* Initialize default format */
	mira050_set_default_format(mira050);

	printk(KERN_INFO "[MIRA050]: Entering pads init function.\n");

	ret = media_entity_pads_init(&mira050->sd.entity, NUM_PADS, mira050->pad);
	if (ret)
	{
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	printk(KERN_INFO "[MIRA050]: Entering subdev sensor common function.\n");

	ret = v4l2_async_register_subdev_sensor(&mira050->sd);
	if (ret < 0)
	{
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	/* For debug purpose */
	// mira050_start_streaming(mira050);

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

error_media_entity:
	media_entity_cleanup(&mira050->sd.entity);

error_handler_free:
	mira050_free_controls(mira050);

error_power_off:
	mira050_power_off(dev);

	i2c_unregister_device(mira050->pmic_client);
	i2c_unregister_device(mira050->uc_client);
	i2c_unregister_device(mira050->led_client);

	return ret;
}

static void mira050_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira050 *mira050 = to_mira050(sd);

	i2c_unregister_device(mira050->pmic_client);
	i2c_unregister_device(mira050->uc_client);
	i2c_unregister_device(mira050->led_client);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	mira050_free_controls(mira050);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		mira050_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static const struct dev_pm_ops mira050_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mira050_suspend, mira050_resume)
		SET_RUNTIME_PM_OPS(mira050_power_off, mira050_power_on, NULL)};

#endif // __MIRA050_INL__
