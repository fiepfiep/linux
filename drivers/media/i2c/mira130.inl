// SPDX-License-Identifier: GPL-2.0
/*
 * A V4L2 driver for ams MIRA130 cameras.
 * Copyright (C) 2022, ams-OSRAM
 *
 * Based on Sony IMX219 camera driver
 * Copyright (C) 2019, Raspberry Pi (Trading) Ltd
 */

#ifndef __MIRA130_INL__
#define __MIRA130_INL__

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
#define AMS_CAMERA_CID_BASE		(V4L2_CTRL_CLASS_CAMERA | 0x2000)
#define AMS_CAMERA_CID_MIRA_REG_W	(AMS_CAMERA_CID_BASE+0)
#define AMS_CAMERA_CID_MIRA_REG_R	(AMS_CAMERA_CID_BASE+1)

/* Most significant Byte is flag, and most significant bit is unused. */
#define AMS_CAMERA_CID_MIRA130_REG_FLAG_FOR_READ        0b00000001
/* Use bit 5 to indicate special command, bit 1,2,3,4 for command. */
#define AMS_CAMERA_CID_MIRA130_REG_FLAG_CMD_SEL         0b00010000
/* Special command for sleep. The other 3 Bytes (addr+val) is sleep values in us. */
#define AMS_CAMERA_CID_MIRA130_REG_FLAG_SLEEP_US        0b00010000
/* Special command to enable power on (/off) when stream on (/off). */
#define AMS_CAMERA_CID_MIRA130_REG_FLAG_RESET_ON        0b00010010
/* Special command to disable power on (/off) when stream on (/off). */
#define AMS_CAMERA_CID_MIRA130_REG_FLAG_RESET_OFF       0b00010100
/* Special command to enable base register sequence upload, overwrite skip-reg-upload in dtoverlay */
#define AMS_CAMERA_CID_MIRA130_REG_FLAG_REG_UP_ON       0b00010110
/* Special command to disable base register sequence upload, overwrite skip-reg-upload in dtoverlay */
#define AMS_CAMERA_CID_MIRA130_REG_FLAG_REG_UP_OFF      0b00011000
/* Special command to manually power on */
#define AMS_CAMERA_CID_MIRA130_REG_FLAG_POWER_ON        0b00011010
/* Special command to manually power off */
#define AMS_CAMERA_CID_MIRA130_REG_FLAG_POWER_OFF       0b00011100
/* Special command to turn illumination trigger on */
#define AMS_CAMERA_CID_MIRA130_REG_FLAG_ILLUM_TRIG_ON   0b00011110
/* Special command to turn illumination trigger off */
#define AMS_CAMERA_CID_MIRA130_REG_FLAG_ILLUM_TRIG_OFF  0b00010001
/* Special command to enable force_stream_ctrl */
#define AMS_CAMERA_CID_MIRA130_REG_FLAG_STREAM_CTRL_ON  0b00011011
/* Special command to disable force_stream_ctrl */
#define AMS_CAMERA_CID_MIRA130_REG_FLAG_STREAM_CTRL_OFF 0b00011101
/*
 * Bit 6&7 of flag are combined to specify I2C dev (default is Mira).
 * If bit 6&7 is 0b01, the reg_addr and reg_val are for a TBD I2C address.
 * The TBD I2C address is default to MIRA130LED_I2C_ADDR.
 * To change the TBD I2C address, set bit 6&7 to 0b10,
 * then the reg_val will become TBD I2C address.
 * The TBD I2C address is stored in mira130->tbd_client_i2c_addr.
 */
#define AMS_CAMERA_CID_MIRA130_REG_FLAG_I2C_SEL         0b01100000
#define AMS_CAMERA_CID_MIRA130_REG_FLAG_I2C_MIRA        0b00000000
#define AMS_CAMERA_CID_MIRA130_REG_FLAG_I2C_TBD         0b00100000
#define AMS_CAMERA_CID_MIRA130_REG_FLAG_I2C_SET_TBD     0b01000000


/* Pre-allocated i2c_client */
#define MIRA130PMIC_I2C_ADDR 0x2D
#define MIRA130UC_I2C_ADDR 0x0A
#define MIRA130LED_I2C_ADDR 0x53


#define MIRA130_NATIVE_WIDTH			1080U
#define MIRA130_NATIVE_HEIGHT			1280U

#define MIRA130_PIXEL_ARRAY_LEFT		0U
#define MIRA130_PIXEL_ARRAY_TOP			0U
#define MIRA130_PIXEL_ARRAY_WIDTH		1080U
#define MIRA130_PIXEL_ARRAY_HEIGHT		1280U

#define MIRA130_BIT_DEPTH_REG			0x3031
#define MIRA130_BIT_DEPTH_12_BIT		0x0C
#define MIRA130_BIT_DEPTH_10_BIT		0x0A
#define MIRA130_BIT_DEPTH_8_BIT			0x08

#define MIRA130_CSI_DATA_TYPE_REG		0x3037
#define MIRA130_CSI_DATA_TYPE_12_BIT		0x02
#define MIRA130_CSI_DATA_TYPE_10_BIT		0x01
#define MIRA130_CSI_DATA_TYPE_8_BIT		0x00

#define MIRA130_IMAGER_STATE_REG		0x1003
#define MIRA130_IMAGER_STATE_STOP_AT_ROW	0x02
#define MIRA130_IMAGER_STATE_STOP_AT_FRAME	0x04
#define MIRA130_IMAGER_STATE_MASTER_CONTROL	0x10

#define MIRA130_IMAGER_RUN_REG			0x10F0
#define MIRA130_IMAGER_RUN_START		0x01
#define MIRA130_IMAGER_RUN_STOP			0x00

#define MIRA130_IMAGER_RUN_CONT_REG		0x1002
#define MIRA130_IMAGER_RUN_CONT_ENABLE		0x04
#define MIRA130_IMAGER_RUN_CONT_DISABLE		0x00

#define MIRA130_NB_OF_FRAMES_LO_REG		0x10F2
#define MIRA130_NB_OF_FRAMES_HI_REG		0x10F3

// Exposure time is indicated in number of rows
#define MIRA130_EXP_TIME_HI_REG			0x3E00
#define MIRA130_EXP_TIME_LO_REG			0x3E02

#define MIRA130_AGC_MODE_REG			0x3E03
#define MIRA130_ANA_GAIN_REG			0x3E08
#define MIRA130_ANA_FINE_GAIN_REG		0x3E09
#define MIRA130_HDR_ANA_GAIN_REG		0x3E12
#define MIRA130_HDR_ANA_FINE_GAIN_REG		0x3E13

// VBLANK is indicated in number of rows
#define MIRA130_VBLANK_HI_REG			0x320E
#define MIRA130_VBLANK_LO_REG			0x320F

// Sets the duration of the row length in clock cycles of CLK_IN
#define MIRA130_ROW_LENGTH_LO_REG		0x320D
#define MIRA130_ROW_LENGTH_HI_REG		0x320C
#define MIRA130_ROW_LENGTH_MIN			0x02EE

#define MIRA130_HFLIP_REG			0x3221
#define MIRA130_HFLIP_ENABLE_MIRROR		0b00000110

#define MIRA130_VFLIP_REG			0x3221
#define MIRA130_VFLIP_ENABLE_FLIP		0b01100000

#define MIRA130_SUPPORTED_XCLK_FREQ		24000000

#define MIRA130_MIN_VBLANK			120

#define MIRA130_MIN_V_SIZE			1280
#define MIRA130_DEFAULT_EXPOSURE		(MIRA130_MIN_V_SIZE + MIRA130_MIN_VBLANK)
#define MIRA130_EXPOSURE_MIN			1

// Power on function timing
#define MIRA130_XCLR_MIN_DELAY_US		100000
#define MIRA130_XCLR_DELAY_RANGE_US		30

// pixel_rate = link_freq * 2 * nr_of_lanes / bits_per_sample
// 0.6Gb/s * 2 * 2 / 10 = 257698038
#define MIRA130_PIXEL_RATE		(257698037)
/* Should match device tree link freq */
#define MIRA130_DEFAULT_LINK_FREQ	456000000

/* Trick the libcamera with achievable fps via hblank */

/* Formular in libcamera to derive TARGET_FPS:
 * TARGET_FPS=1/((1/MIRA130_PIXEL_RATE)*(WIDTH+HBLANK)*(HEIGHT+MIRA130_MIN_VBLANK))
 * Example: 1080x1280 with HBLANK=0 and MIRA130_MIN_VBLANK=120
 * TARGET_FPS=1/((1/257698037)*1080*(1280+120))=170
 * 
 * Inverse the above formula to derive HBLANK from TARGET_FPS:
 * HBLANK=1/((1/MIRA130_PIXEL_RATE)*TARGET_FPS*(HEIGHT+MIRA130_MIN_VBLANK))-WIDTH
 * Example with TARGET_FPS of 60 fps for 1080x1280
 * HBLANK=1/((1/257698037)*60*(1280+120))-1080=1988
 */
#define MIRA130_HBLANK_1080x1280_60FPS		1988

/* Set max VBLANK to be 2 fps */
/*
 * TARGET_FPS=1/((1/MIRA130_PIXEL_RATE)*(WIDTH+HBLANK)*(HEIGHT+VBLANK))
 * VBLANK=1/((1/MIRA130_PIXEL_RATE)*TARGET_FPS*(WIDTH+HBLANK))-HEIGHT
 * Example with TARGET_FPS of 2 fps and HBLANK of 1988
 * VBLANK=1/((1/257698037)*2*(1080+1988))-1280=40717
 */
#define MIRA130_VBLANK_1080x1280_2FPS		40717
#define MIRA130_MAX_VBLANK			MIRA130_VBLANK_1080x1280_2FPS

#define MIRA130_REG_TEST_PATTERN	0x4501
#define	MIRA130_TEST_PATTERN_DISABLE	0x00
#define	MIRA130_TEST_PATTERN_VERTICAL_GRADIENT	0x01

/* Embedded metadata stream structure */
#define MIRA130_EMBEDDED_LINE_WIDTH 16384
#define MIRA130_NUM_EMBEDDED_LINES 1

/* From Jetson driver */
#define MIRA130_DEFAULT_LINE_LENGTH    (0x02EE)
#define MIRA130_DEFAULT_PIXEL_CLOCK    (24)
#define MIRA130_DEFAULT_FRAME_LENGTH    (0x0578)

/* Illumination trigger */
#define MIRA130_EN_TRIG_ILLUM_REG      0x3361

enum pad_types {
	IMAGE_PAD,
	METADATA_PAD,
	NUM_PADS
};

struct mira130_reg {
	u16 address;
	u8 val;
};

struct mira130_analog_gain_lut {
	u8 gain;
	u8 fine_gain;
};


struct mira130_reg_list {
	unsigned int num_of_regs;
	const struct mira130_reg *regs;
};

struct mira130_v4l2_reg {
	u32 val;
};

/* Mode : resolution and related config&values */
struct mira130_mode {
	/* Frame width */
	unsigned int width;
	/* Frame height */
	unsigned int height;

	/* Analog crop rectangle. */
	struct v4l2_rect crop;

	/* Default register values */
	struct mira130_reg_list reg_list;

	u32 row_length;
	u32 vblank;
	u32 hblank;
	u32 code;
};

// 1080_1280_60fps_10b_2lanes
static const struct mira130_reg full_1080_1280_60fps_10b_2lanes_reg[] = {
	{0x0103,0x01},
	{0x0100,0x00},
	{0x36e9,0x80},
	{0x36f9,0x80},
	{0x300a,0x64},
	{0x3018,0x32},
	{0x3019,0x0c},
	{0x301a,0xb4},
	{0x301f,0xbf},
	{0x3031,0x0a},
	{0x3032,0xa0},
	{0x3038,0x44},
	{0x3207,0x17},
	{0x320c,0x02},
	{0x320d,0xee},
	{0x320e,0x05},
	{0x320f,0x78},
	{0x3217,0x05},
	{0x3218,0x72},
	{0x3250,0xcc},
	{0x3251,0x02},
	{0x3252,0x05},
	{0x3253,0x73},
	{0x3254,0x05},
	{0x3255,0x3b},
	{0x3306,0x78},
	{0x330a,0x00},
	{0x330b,0xc8},
	{0x330f,0x24},
	{0x3314,0x80},
	{0x3315,0x40},
	{0x3317,0xf0},
	{0x331f,0x12},
	{0x3364,0x00},
	{0x3385,0x41},
	{0x3387,0x41},
	{0x3389,0x09},
	{0x33ab,0x00},
	{0x33ac,0x00},
	{0x33b1,0x03},
	{0x33b2,0x12},
	{0x33f8,0x02},
	{0x33fa,0x01},
	{0x3409,0x08},
	{0x34f0,0xc0},
	{0x34f1,0x20},
	{0x34f2,0x03},
	{0x3622,0xf5},
	{0x3630,0x5c},
	{0x3631,0x80},
	{0x3632,0xc8},
	{0x3633,0x32},
	{0x3638,0x2a},
	{0x3639,0x07},
	{0x363b,0x48},
	{0x363c,0x83},
	{0x363d,0x10},
	{0x36ea,0x36},
	{0x36eb,0x04},
	{0x36ec,0x03},
	{0x36ed,0x24},
	{0x36fa,0x2b},
	{0x36fb,0x0b},
	{0x36fc,0x01},
	{0x36fd,0x34},
	{0x3900,0x11},
	{0x3901,0x05},
	{0x3902,0xc5},
	{0x3904,0x04},
	{0x3908,0x91},
	{0x391e,0x00},
	{0x3e01,0x57},
	{0x3e02,0x00},
	{0x3e09,0x20},
	{0x3e0e,0xd2},
	{0x3e14,0xb0},
	{0x3e1e,0x7c},
	{0x3e26,0x20},
	{0x4418,0x38},
	{0x4503,0x10},
	{0x4800,0x24},
	{0x4837,0x1a},
	{0x5000,0x0e},
	{0x540c,0x51},
	{0x550f,0x38},
	{0x5780,0x67},
	{0x5784,0x10},
	{0x5785,0x06},
	{0x5787,0x02},
	{0x5788,0x00},
	{0x5789,0x00},
	{0x578a,0x02},
	{0x578b,0x00},
	{0x578c,0x00},
	{0x5790,0x00},
	{0x5791,0x00},
	{0x5792,0x00},
	{0x5793,0x00},
	{0x5794,0x00},
	{0x5795,0x00},
	{0x5799,0x04},
	{0x36e9,0x54},
	{0x36f9,0x50},
	{0x0100,0x01},
	{0x33fa,0x01},
	{0x3317,0xf0},
};

static const struct mira130_analog_gain_lut analog_gain_lut[] = {
	{0x03,0x20},
	{0x03,0x21},
	{0x03,0x22},
	{0x03,0x23},
	{0x03,0x24},
	{0x03,0x25},
	{0x03,0x26},
	{0x03,0x27},
	{0x03,0x28},
	{0x03,0x29},
	{0x03,0x2A},
	{0x03,0x2B},
	{0x03,0x2C},
	{0x03,0x2D},
	{0x03,0x2E},
	{0x03,0x2F},
	{0x03,0x30},
	{0x03,0x31},
	{0x03,0x32},
	{0x03,0x33},
	{0x03,0x34},
	{0x03,0x35},
	{0x03,0x36},
	{0x03,0x37},
	{0x03,0x38},
	{0x03,0x39},
	{0x23,0x20},
	{0x23,0x21},
	{0x23,0x22},
	{0x23,0x23},
	{0x23,0x24},
	{0x23,0x25},
	{0x23,0x26},
	{0x23,0x27},
	{0x23,0x28},
	{0x23,0x29},
	{0x23,0x2A},
	{0x23,0x2B},
	{0x23,0x2C},
	{0x23,0x2D},
	{0x23,0x2E},
	{0x23,0x2F},
	{0x23,0x30},
	{0x23,0x31},
	{0x23,0x32},
	{0x23,0x33},
	{0x23,0x34},
	{0x23,0x35},
	{0x23,0x36},
	{0x23,0x37},
	{0x23,0x38},
	{0x23,0x39},
	{0x23,0x3A},
	{0x23,0x3B},
	{0x23,0x3C},
	{0x23,0x3D},
	{0x23,0x3E},
	{0x23,0x3F},
	{0x27,0x20},
	{0x27,0x21},
	{0x27,0x22},
	{0x27,0x23},
	{0x27,0x24},
	{0x27,0x25},
	{0x27,0x26},
	{0x27,0x27},
	{0x27,0x28},
	{0x27,0x29},
	{0x27,0x2A},
	{0x27,0x2B},
	{0x27,0x2C},
	{0x27,0x2D},
	{0x27,0x2E},
	{0x27,0x2F},
	{0x27,0x30},
	{0x27,0x31},
	{0x27,0x32},
	{0x27,0x33},
	{0x27,0x34},
	{0x27,0x35},
	{0x27,0x36},
	{0x27,0x37},
	{0x27,0x38},
	{0x27,0x39},
	{0x27,0x3A},
	{0x27,0x3B},
	{0x27,0x3C},
	{0x27,0x3D},
	{0x27,0x3E},
	{0x27,0x3F},
	{0x2F,0x20},
	{0x2F,0x21},
	{0x2F,0x22},
	{0x2F,0x23},
	{0x2F,0x24},
	{0x2F,0x25},
	{0x2F,0x26},
	{0x2F,0x27},
	{0x2F,0x28},
	{0x2F,0x29},
	{0x2F,0x2A},
	{0x2F,0x2B},
	{0x2F,0x2C},
	{0x2F,0x2D},
	{0x2F,0x2E},
	{0x2F,0x2F},
	{0x2F,0x30},
	{0x2F,0x31},
	{0x2F,0x32},
	{0x2F,0x33},
	{0x2F,0x34},
	{0x2F,0x35},
	{0x2F,0x36},
	{0x2F,0x37},
	{0x2F,0x38},
	{0x2F,0x39},
	{0x2F,0x3A},
	{0x2F,0x3B},
	{0x2F,0x3C},
	{0x2F,0x3D},
	{0x2F,0x3E},
	{0x2F,0x3F},
	{0x3F,0x20},
	{0x3F,0x21},
	{0x3F,0x22},
	{0x3F,0x23},
	{0x3F,0x24},
	{0x3F,0x25},
	{0x3F,0x26},
	{0x3F,0x27},
	{0x3F,0x28},
	{0x3F,0x29},
	{0x3F,0x2A},
	{0x3F,0x2B},
	{0x3F,0x2C},
	{0x3F,0x2D},
	{0x3F,0x2E},
	{0x3F,0x2F},
	{0x3F,0x30},
	{0x3F,0x31},
	{0x3F,0x32},
	{0x3F,0x33},
	{0x3F,0x34},
	{0x3F,0x35},
	{0x3F,0x36},
	{0x3F,0x37},
	{0x3F,0x38},
	{0x3F,0x39},
	{0x3F,0x3A},
	{0x3F,0x3B},
	{0x3F,0x3C},
	{0x3F,0x3D},
	{0x3F,0x3E},
	{0x3F,0x3F},
};

static const char * const mira130_test_pattern_menu[] = {
	"Disabled",
	"Vertial Gradient",
};

static const int mira130_test_pattern_val[] = {
	MIRA130_TEST_PATTERN_DISABLE,
	MIRA130_TEST_PATTERN_VERTICAL_GRADIENT,
};


/* regulator supplies */
static const char * const mira130_supply_name[] = {
	// TODO(jalv): Check supply names
	/* Supplies can be enabled in any order */
	"VANA",  /* Analog (2.8V) supply */
	"VDIG",  /* Digital Core (1.8V) supply */
	"VDDL",  /* IF (1.2V) supply */
};

#define MIRA130_NUM_SUPPLIES ARRAY_SIZE(mira130_supply_name)

/*
 * The supported formats. All flip/mirror combinations have the same byte order because the sensor
 * is monochrome
 */
static const u32 codes[] = {
	MEDIA_BUS_FMT_SGRBG10_1X10,
};

/* Mode configs */
static const struct mira130_mode supported_modes[] = {
	/* 60fps 10bpp mode */
	{
		.width = 1080,
		.height = 1280,
		.crop = {
			.left = MIRA130_PIXEL_ARRAY_LEFT,
			.top = MIRA130_PIXEL_ARRAY_TOP,
			.width = 1080,
			.height = 1280
		},
		.reg_list = {
			.num_of_regs = ARRAY_SIZE(full_1080_1280_60fps_10b_2lanes_reg),
			.regs = full_1080_1280_60fps_10b_2lanes_reg,
		},
		// ROW_LENGTH is configured by register 0x320C, 0x320D.
		.row_length = MIRA130_ROW_LENGTH_MIN,
		.vblank = MIRA130_MIN_VBLANK,
		.hblank = MIRA130_HBLANK_1080x1280_60FPS,
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
	},
};

struct mira130 {
	struct v4l2_subdev sd;
	struct media_pad pad[NUM_PADS];

	struct v4l2_mbus_framefmt fmt;

	struct clk *xclk; /* system clock to MIRA130 */
	u32 xclk_freq;

	//struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[MIRA130_NUM_SUPPLIES];

	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *gain;
	// custom v4l2 control
	struct v4l2_ctrl *mira130_reg_w;
	struct v4l2_ctrl *mira130_reg_r;
	u16 mira130_reg_w_cached_addr;
	u8 mira130_reg_w_cached_flag;


	/* Current mode */
	const struct mira130_mode *mode;
	/* Whether to skip base register sequence upload */
	u32 skip_reg_upload;
	/* Whether to reset sensor when stream on/off */
	u32 skip_reset;
	/* Whether regulator and clk are powered on */
	u32 powered;
	/* A flag to force write_start/stop_streaming_regs even if (skip_reg_upload==1) */
	u8 force_stream_ctrl;


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

static inline struct mira130 *to_mira130(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct mira130, sd);
}

static int mira130_read(struct mira130 *mira130, u16 reg, u8 *val)
{
	int ret;
	unsigned char data_w[2] = { reg >> 8, reg & 0xff };
	struct i2c_client *client = v4l2_get_subdevdata(&mira130->sd);

	ret = i2c_master_send(client, data_w, 2);
	/*
	 * A negative return code, or sending the wrong number of bytes, both
	 * count as an error.
	 */
	if (ret != 2) {
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
	if (ret == 1) {
		ret = 0;
	} else {
		dev_dbg(&client->dev, "%s: i2c read error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
	}

	return ret;
}

static int mira130_write(struct mira130 *mira130, u16 reg, u8 val)
{
	int ret;
	unsigned char data[3] = { reg >> 8, reg & 0xff, val};
	struct i2c_client *client = v4l2_get_subdevdata(&mira130->sd);

	ret = i2c_master_send(client, data, 3);
	/*
	 * Writing the wrong number of bytes also needs to be flagged as an
	 * error. Success needs to produce a 0 return code.
	 */
	if (ret == 3) {
		ret = 0;
	} else {
		dev_dbg(&client->dev, "%s: i2c write error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
	}

	return ret;
}

/*
 * mira130 is big-endian: msb of val goes to lower reg addr
 */
static int mira130_write16(struct mira130 *mira130, u16 reg, u16 val)
{
       int ret;
       unsigned char data[4] = { reg >> 8, reg & 0xff, (val >> 8) & 0xff, val & 0xff };
       struct i2c_client *client = v4l2_get_subdevdata(&mira130->sd);

       ret = i2c_master_send(client, data, 4);
       /*
        * Writing the wrong number of bytes also needs to be flagged as an
        * error. Success needs to produce a 0 return code.
        */
       if (ret == 4) {
               ret = 0;
       } else {
               dev_dbg(&client->dev, "%s: i2c write error, reg: %x\n",
                               __func__, reg);
               if (ret >= 0)
                       ret = -EINVAL;
       }

       return ret;
}

/*
 * mira130 is big-endian: msb of val goes to lower reg addr
 */
static int mira130_write24(struct mira130 *mira130, u16 reg, u32 val)
{
       int ret;
       unsigned char data[5] = { reg >> 8, reg & 0xff, (val >> 16) & 0xff, (val >> 8) & 0xff, val & 0xff };
       struct i2c_client *client = v4l2_get_subdevdata(&mira130->sd);

       ret = i2c_master_send(client, data, 5);
       /*
        * Writing the wrong number of bytes also needs to be flagged as an
        * error. Success needs to produce a 0 return code.
        */
       if (ret == 5) {
               ret = 0;
       } else {
               dev_dbg(&client->dev, "%s: i2c write error, reg: %x\n",
                               __func__, reg);
               if (ret >= 0)
                       ret = -EINVAL;
       }

       return ret;
}


/* Write a list of registers */
static int mira130_write_regs(struct mira130 *mira130,
			     const struct mira130_reg *regs, u32 len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira130->sd);
	unsigned int i;
	int ret;

	for (i = 0; i < len; i++) {
		ret = mira130_write(mira130, regs[i].address, regs[i].val);
		if (ret) {
			dev_err_ratelimited(&client->dev,
					    "Failed to write reg 0x%4.4x. error = %d\n",
					    regs[i].address, ret);

			return ret;
		} else {
			// Debug code below
			//u8 val;
			//ret = mira130_read(mira130, regs[i].address, &val);
			//printk(KERN_INFO "[MIRA130]: Read reg 0x%4.4x, val = 0x%x.\n",
			//		regs[i].address, val);
		}
	}

	return 0;
}

static int mira130pmic_write(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;
	unsigned char data[2] = { reg & 0xff, val};

	ret = i2c_master_send(client, data, 2);
	/*
	 * Writing the wrong number of bytes also needs to be flagged as an
	 * error. Success needs to produce a 0 return code.
	 */
	if (ret == 2) {
		ret = 0;
	} else {
		dev_dbg(&client->dev, "%s: i2c write error, reg: %x\n",
				__func__, reg);
		if (ret >= 0)
			ret = -EINVAL;
	}

	return ret;
}

static int mira130pmic_read(struct i2c_client *client, u8 reg, u8 *val)
{
	struct i2c_msg msgs[2];
	u8 addr_buf[1] = { reg & 0xff };
	u8 data_buf[1] = { 0 };
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
static int mira130_power_on(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira130 *mira130 = to_mira130(sd);
	int ret = -EINVAL;

	printk(KERN_INFO "[MIRA130]: Entering power on function.\n");

	if (mira130->powered == 0) {
		ret = regulator_bulk_enable(MIRA130_NUM_SUPPLIES, mira130->supplies);
		if (ret) {
			dev_err(&client->dev, "%s: failed to enable regulators\n",
				__func__);
			return ret;
		}
		ret = clk_prepare_enable(mira130->xclk);
		if (ret) {
			dev_err(&client->dev, "%s: failed to enable clock\n",
				__func__);
			goto reg_off;
		}
		usleep_range(MIRA130_XCLR_MIN_DELAY_US,
			     MIRA130_XCLR_MIN_DELAY_US + MIRA130_XCLR_DELAY_RANGE_US);
		mira130->powered = 1;
	} else {
		printk(KERN_INFO "[MIRA130]: Skip regulator and clk enable, because mira130->powered == %d.\n", mira130->powered);
	}
	return 0;

reg_off:
	ret = regulator_bulk_disable(MIRA130_NUM_SUPPLIES, mira130->supplies);
	mira130->powered = 0;
	return ret;
}

static int mira130_power_off(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira130 *mira130 = to_mira130(sd);

	printk(KERN_INFO "[MIRA130]: Entering power off function.\n");

	if (mira130->skip_reset == 0) {
		if (mira130->powered == 1) {
			regulator_bulk_disable(MIRA130_NUM_SUPPLIES, mira130->supplies);
			clk_disable_unprepare(mira130->xclk);
			mira130->powered = 0;
		} else {
			printk(KERN_INFO "[MIRA130]: Skip disabling regulator and clk due to mira130->powered == %d.\n", mira130->powered);
		}
	} else {
		printk(KERN_INFO "[MIRA130]: Skip disabling regulator and clk due to mira130->skip_reset=%u.\n", mira130->skip_reset);
	}

	return 0;
}

static int mira130_write_illum_trig_regs(struct mira130* mira130, u8 enable) {
	struct i2c_client* const client = v4l2_get_subdevdata(&mira130->sd);
	int ret = 0;
	u8 enable_reg;

	// Enable or disable illumination trigger
	/* Enable: set bit [7:6] to 0b00. Disable: set bit [7:0] to 0b11. */
	if (enable != 0) {
		enable_reg = 0b00000000;
	} else {
		enable_reg = 0b11000000;
	}
	printk(KERN_INFO "[MIRA130]: Writing EN_TRIG_ILLUM to %d.\n", enable_reg);
	ret = mira130_write(mira130, MIRA130_EN_TRIG_ILLUM_REG, enable_reg);
	if (ret) {
		dev_err(&client->dev, "Error setting EN_TRIG_ILLUM to %d.", enable_reg);
		return ret;
	}
	
	return ret;
}



static int mira130_write_start_streaming_regs(struct mira130* mira130) {
	struct i2c_client* const client = v4l2_get_subdevdata(&mira130->sd);
	int ret = 0;

	(void)client;
	// TODO: Check any register write is needed.

	return ret;
}

static int mira130_write_stop_streaming_regs(struct mira130* mira130) {
	struct i2c_client* const client = v4l2_get_subdevdata(&mira130->sd);
	int ret = 0;
	u32 frame_time;
	int try_cnt;

	(void)client;
	(void)try_cnt;

	// TODO: check any register write is needed.

        /*
         * Wait for one frame to make sure sensor is set to
         * software standby in V-blank
         *
         * frame_time = frame length rows * Tline
         * Tline = line length / pixel clock (in MHz)
         */
        frame_time = MIRA130_DEFAULT_FRAME_LENGTH *
            MIRA130_DEFAULT_LINE_LENGTH / MIRA130_DEFAULT_PIXEL_CLOCK;

        usleep_range(frame_time, frame_time + 1000);

	return ret;
}


static int mira130_v4l2_reg_w(struct mira130 *mira130, u32 value) {
	struct i2c_client* const client = v4l2_get_subdevdata(&mira130->sd);
	u32 ret = 0;
	u32 tmp_flag;

	u16 reg_addr = (value >> 8) & 0xFFFF;
	u8 reg_val = value & 0xFF;
	u8 reg_flag = (value >> 24) & 0xFF;

	// printk(KERN_INFO "[MIRA130]: %s reg_flag: 0x%02X; reg_addr: 0x%04X; reg_val: 0x%02X.\n",
	// 		__func__, reg_flag, reg_addr, reg_val);

	if (reg_flag & AMS_CAMERA_CID_MIRA130_REG_FLAG_CMD_SEL) {
		if (reg_flag == AMS_CAMERA_CID_MIRA130_REG_FLAG_SLEEP_US) {
			// If it is for sleep, combine all 24 bits of reg_addr and reg_val as sleep us.
			u32 sleep_us_val = value & 0x00FFFFFF;
			// Sleep range needs an interval, default to 1/8 of the sleep value.
			u32 sleep_us_interval = sleep_us_val >> 3;
			printk(KERN_INFO "[MIRA130]: %s sleep_us: %u.\n", __func__, sleep_us_val);
			usleep_range(sleep_us_val, sleep_us_val + sleep_us_interval);
		} else if (reg_flag == AMS_CAMERA_CID_MIRA130_REG_FLAG_RESET_ON) {
			printk(KERN_INFO "[MIRA130]: %s Enable reset at stream on/off.\n", __func__);
			mira130->skip_reset = 0;
		} else if (reg_flag == AMS_CAMERA_CID_MIRA130_REG_FLAG_RESET_OFF) {
			printk(KERN_INFO "[MIRA130]: %s Disable reset at stream on/off.\n", __func__);
			mira130->skip_reset = 1;
		} else if (reg_flag == AMS_CAMERA_CID_MIRA130_REG_FLAG_REG_UP_ON) {
			printk(KERN_INFO "[MIRA130]: %s Enable base register sequence upload.\n", __func__);
			mira130->skip_reg_upload = 0;
		} else if (reg_flag == AMS_CAMERA_CID_MIRA130_REG_FLAG_REG_UP_OFF) {
			printk(KERN_INFO "[MIRA130]: %s Disable base register sequence upload.\n", __func__);
			mira130->skip_reg_upload = 1;
		} else if (reg_flag == AMS_CAMERA_CID_MIRA130_REG_FLAG_POWER_ON) {
			printk(KERN_INFO "[MIRA130]: %s Call power on function mira130_power_on().\n", __func__);
			/* Temporarily disable skip_reset if manually doing power on/off */
			tmp_flag = mira130->skip_reset;
			mira130->skip_reset = 0;
			mira130_power_on(&client->dev);
			mira130->skip_reset = tmp_flag;
		} else if (reg_flag == AMS_CAMERA_CID_MIRA130_REG_FLAG_POWER_OFF) {
			printk(KERN_INFO "[MIRA130]: %s Call power off function mira130_power_off().\n", __func__);
			/* Temporarily disable skip_reset if manually doing power on/off */
			tmp_flag = mira130->skip_reset;
			mira130->skip_reset = 0;
			mira130_power_off(&client->dev);
			mira130->skip_reset = tmp_flag;
		} else if (reg_flag == AMS_CAMERA_CID_MIRA130_REG_FLAG_ILLUM_TRIG_ON) {
			printk(KERN_INFO "[MIRA130]: %s Enable illumination trigger.\n", __func__);
			mira130_write_illum_trig_regs(mira130, 1);
		} else if (reg_flag == AMS_CAMERA_CID_MIRA130_REG_FLAG_ILLUM_TRIG_OFF) {
			printk(KERN_INFO "[MIRA130]: %s Disable illumination trigger.\n", __func__);
			mira130_write_illum_trig_regs(mira130, 0);
		} else if (reg_flag == AMS_CAMERA_CID_MIRA130_REG_FLAG_STREAM_CTRL_ON) {
			printk(KERN_INFO "[MIRA130]: %s Force stream control even if (skip_reg_upload == 1).\n", __func__);
			mira130->force_stream_ctrl = 1;
		} else if (reg_flag == AMS_CAMERA_CID_MIRA130_REG_FLAG_STREAM_CTRL_OFF) {
			printk(KERN_INFO "[MIRA130]: %s Disable stream control if (skip_reg_upload == 1).\n", __func__);
			mira130->force_stream_ctrl = 0;
		} else {
			printk(KERN_INFO "[MIRA130]: %s unknown command from flag %u, ignored.\n", __func__, reg_flag);
		}
	} else if (reg_flag & AMS_CAMERA_CID_MIRA130_REG_FLAG_FOR_READ) {
		// If it is for read, skip reagister write, cache addr and flag for read.
		mira130->mira130_reg_w_cached_addr = reg_addr;
		mira130->mira130_reg_w_cached_flag = reg_flag;
	} else {
		// If it is for write, select which I2C device by the flag "I2C_SEL".
		if ((reg_flag & AMS_CAMERA_CID_MIRA130_REG_FLAG_I2C_SEL) == AMS_CAMERA_CID_MIRA130_REG_FLAG_I2C_MIRA) {
			// Writing the actual Mira130 register
			// printk(KERN_INFO "[MIRA130]: %s write reg_addr: 0x%04X; reg_val: 0x%02X.\n", __func__, reg_addr, reg_val);
			ret = mira130_write(mira130, reg_addr, reg_val);
			if (ret) {
				dev_err_ratelimited(&client->dev, "Error AMS_CAMERA_CID_MIRA_REG_W reg_addr %X.\n", reg_addr);
				return -EINVAL;
			}
		} else if ((reg_flag & AMS_CAMERA_CID_MIRA130_REG_FLAG_I2C_SEL) == AMS_CAMERA_CID_MIRA130_REG_FLAG_I2C_SET_TBD) {
			/* User tries to set TBD I2C address, store reg_val to mira130->tbd_client_i2c_addr. Skip write. */
			printk(KERN_INFO "[MIRA130]: mira130->tbd_client_i2c_addr = 0x%X.\n", reg_val);
			mira130->tbd_client_i2c_addr = reg_val;
		} else if ((reg_flag & AMS_CAMERA_CID_MIRA130_REG_FLAG_I2C_SEL) == AMS_CAMERA_CID_MIRA130_REG_FLAG_I2C_TBD) {
			if (mira130->tbd_client_i2c_addr == MIRA130PMIC_I2C_ADDR) {
				// Write PMIC. Use pre-allocated mira130->pmic_client.
				printk(KERN_INFO "[MIRA130]: write pmic_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
				ret = mira130pmic_write(mira130->pmic_client, (u8)(reg_addr & 0xFF), reg_val);
			} else if (mira130->tbd_client_i2c_addr == MIRA130UC_I2C_ADDR) {
				// Write micro-controller. Use pre-allocated mira130->uc_client.
				printk(KERN_INFO "[MIRA130]: write uc_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
				ret = mira130pmic_write(mira130->uc_client, (u8)(reg_addr & 0xFF), reg_val);
			} else if (mira130->tbd_client_i2c_addr == MIRA130LED_I2C_ADDR) {
				// Write LED driver. Use pre-allocated mira130->led_client.
				printk(KERN_INFO "[MIRA130]: write led_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
				ret = mira130pmic_write(mira130->led_client, (u8)(reg_addr & 0xFF), reg_val);
			} else {
				/* Write other TBD I2C address.
				 * The TBD I2C address is set via AMS_CAMERA_CID_MIRA130_REG_FLAG_I2C_SET_TBD.
				 * The TBD I2C address is stored in mira130->tbd_client_i2c_addr.
				 * A temporary I2C client, tmp_client, is created and then destroyed (unregistered).
				 */
				struct i2c_client *tmp_client;
				tmp_client = i2c_new_dummy_device(client->adapter, mira130->tbd_client_i2c_addr);
				if (IS_ERR(tmp_client))
					return PTR_ERR(tmp_client);
				printk(KERN_INFO "[MIRA130]: write tbd_client, i2c_addr %u, reg_addr 0x%X, reg_val 0x%X.\n",
						mira130->tbd_client_i2c_addr, (u8)(reg_addr & 0xFF), reg_val);
				ret = mira130pmic_write(tmp_client, (u8)(reg_addr & 0xFF), reg_val);
				i2c_unregister_device(tmp_client);
			}
		}
	}

	return 0;
}

static int mira130_v4l2_reg_r(struct mira130 *mira130, u32 *value) {
	struct i2c_client* const client = v4l2_get_subdevdata(&mira130->sd);
	u32 ret = 0;

	u16 reg_addr = mira130->mira130_reg_w_cached_addr;
	u8 reg_flag = mira130->mira130_reg_w_cached_flag;
	u8 reg_val = 0;

	*value = 0;

	if ((reg_flag & AMS_CAMERA_CID_MIRA130_REG_FLAG_I2C_SEL) == AMS_CAMERA_CID_MIRA130_REG_FLAG_I2C_MIRA) {
		ret = mira130_read(mira130, reg_addr, &reg_val);
		if (ret) {
			dev_err_ratelimited(&client->dev, "Error AMS_CAMERA_CID_MIRA_REG_R reg_addr %X.\n", reg_addr);
			return -EINVAL;
		}
	} else if ((reg_flag & AMS_CAMERA_CID_MIRA130_REG_FLAG_I2C_SEL) == AMS_CAMERA_CID_MIRA130_REG_FLAG_I2C_TBD) {
		if (mira130->tbd_client_i2c_addr == MIRA130PMIC_I2C_ADDR) {
			// Read PMIC. Use pre-allocated mira130->pmic_client.
			ret = mira130pmic_read(mira130->pmic_client, (u8)(reg_addr & 0xFF), &reg_val);
			printk(KERN_INFO "[MIRA130]: read pmic_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
		} else if (mira130->tbd_client_i2c_addr == MIRA130UC_I2C_ADDR) {
			// Read micro-controller. Use pre-allocated mira130->uc_client.
			ret = mira130pmic_read(mira130->uc_client, (u8)(reg_addr & 0xFF), &reg_val);
			printk(KERN_INFO "[MIRA130]: read uc_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
		} else if (mira130->tbd_client_i2c_addr == MIRA130LED_I2C_ADDR) {
			// Read LED driver. Use pre-allocated mira130->led_client.
			ret = mira130pmic_read(mira130->led_client, (u8)(reg_addr & 0xFF), &reg_val);
			printk(KERN_INFO "[MIRA130]: read led_client, reg_addr 0x%X, reg_val 0x%X.\n", (u8)(reg_addr & 0xFF), reg_val);
		} else {
			/* Read other TBD I2C address.
			 * The TBD I2C address is set via AMS_CAMERA_CID_MIRA130_REG_FLAG_I2C_SET_TBD.
			 * The TBD I2C address is stored in mira130->tbd_client_i2c_addr.
			 * A temporary I2C client, tmp_client, is created and then destroyed (unregistered).
			 */
			struct i2c_client *tmp_client;
			tmp_client = i2c_new_dummy_device(client->adapter, mira130->tbd_client_i2c_addr);
			if (IS_ERR(tmp_client))
				return PTR_ERR(tmp_client);
			ret = mira130pmic_read(tmp_client, (u8)(reg_addr & 0xFF), &reg_val);
			printk(KERN_INFO "[MIRA130]: read tbd_client, i2c_addr %u, reg_addr 0x%X, reg_val 0x%X.\n",
					mira130->tbd_client_i2c_addr, (u8)(reg_addr & 0xFF), reg_val);
			i2c_unregister_device(tmp_client);
		}
	}

	// Return 32-bit value that includes flags, addr, and register value
	*value = ((u32)reg_flag << 24) | ((u32)reg_addr << 8) | (u32)reg_val;

	// printk(KERN_INFO "[MIRA130]: mira130_v4l2_reg_r() reg_flag: 0x%02X; reg_addr: 0x%04X, reg_val: 0x%02X.\n",
	// 		reg_flag, reg_addr, reg_val);

	return 0;
}

// Returns the maximum exposure time in row_length (reg value).
static u32 mira130_calculate_max_exposure_time(u32 row_length, u32 vsize,
					       u32 vblank) {
	return (vsize + vblank);
}

static int mira130_write_analog_gain_reg(struct mira130 *mira130, u8 gain) {
	struct i2c_client* const client = v4l2_get_subdevdata(&mira130->sd);
	u32 ret = 0;

	if (gain < ARRAY_SIZE(analog_gain_lut)) {
		u8 lut_gain = analog_gain_lut[gain].gain;
		u8 lut_fine_gain = analog_gain_lut[gain].fine_gain;
		ret |= mira130_write(mira130, MIRA130_AGC_MODE_REG, 0x0B);
		ret |= mira130_write(mira130, MIRA130_ANA_GAIN_REG, lut_gain);
		ret |= mira130_write(mira130, MIRA130_ANA_FINE_GAIN_REG, lut_fine_gain);
		ret |= mira130_write(mira130, MIRA130_HDR_ANA_GAIN_REG, lut_gain);
		ret |= mira130_write(mira130, MIRA130_HDR_ANA_FINE_GAIN_REG, lut_fine_gain);
	}
	if (ret) {
		dev_err(&client->dev, "%s failed to set mode\n", __func__);
	}
	return 0;
}

static int mira130_write_exposure_reg(struct mira130 *mira130, u32 exposure) {
	struct i2c_client* const client = v4l2_get_subdevdata(&mira130->sd);
	const u32 max_exposure = mira130_calculate_max_exposure_time(mira130->mode->row_length,
		mira130->mode->height, mira130->mode->vblank);
	u32 ret = 0;
	u32 capped_exposure = exposure;

	if (exposure > max_exposure) {
		capped_exposure = max_exposure;
	}

	// Mira130 exposure time register is in the unit of 1/16 line
	ret = mira130_write24(mira130, MIRA130_EXP_TIME_HI_REG, capped_exposure << 4);
	if (ret) {
		dev_err_ratelimited(&client->dev, "Error setting exposure time to %d", capped_exposure);
		return -EINVAL;
	}

	return 0;
}

// Gets the format code if supported. Otherwise returns the default format code `codes[0]`
static u32 mira130_validate_format_code_or_default(struct mira130 *mira130, u32 code)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira130->sd);
	unsigned int i;

	lockdep_assert_held(&mira130->mutex);

	for (i = 0; i < ARRAY_SIZE(codes); i++)
		if (codes[i] == code)
			break;

	if (i >= ARRAY_SIZE(codes)) {
		dev_err_ratelimited(&client->dev, "Could not set requested format code %u", code);
		dev_err_ratelimited(&client->dev, "Using default format %u", codes[0]);
		i = 0;
	}

	return codes[i];
}

static void mira130_set_default_format(struct mira130 *mira130)
{
	struct v4l2_mbus_framefmt *fmt;

	fmt = &mira130->fmt;
	fmt->code = supported_modes[0].code;
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

static int mira130_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct mira130 *mira130 = to_mira130(sd);
	struct v4l2_mbus_framefmt *try_fmt_img =
		v4l2_subdev_get_try_format(sd, fh->state, IMAGE_PAD);
	struct v4l2_mbus_framefmt *try_fmt_meta =
		v4l2_subdev_get_try_format(sd, fh->state, METADATA_PAD);
	struct v4l2_rect *try_crop;

	mutex_lock(&mira130->mutex);

	/* Initialize try_fmt for the image pad */
	try_fmt_img->width = supported_modes[0].width;
	try_fmt_img->height = supported_modes[0].height;
	try_fmt_img->code = mira130_validate_format_code_or_default(mira130,
						   supported_modes[0].code);
	try_fmt_img->field = V4L2_FIELD_NONE;

	/* TODO(jalv): Initialize try_fmt for the embedded metadata pad */
	try_fmt_meta->width = MIRA130_EMBEDDED_LINE_WIDTH;
	try_fmt_meta->height = MIRA130_NUM_EMBEDDED_LINES;
	try_fmt_meta->code = MEDIA_BUS_FMT_SENSOR_DATA;
	try_fmt_meta->field = V4L2_FIELD_NONE;



	/* Initialize try_crop rectangle. */
	try_crop = v4l2_subdev_get_try_crop(sd, fh->state, 0);
	try_crop->top = supported_modes[0].crop.top;
	try_crop->left = supported_modes[0].crop.left;
	try_crop->width = supported_modes[0].crop.width;
	try_crop->height = supported_modes[0].crop.height;

	mutex_unlock(&mira130->mutex);

	return 0;
}

static int mira130_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mira130 *mira130 =
		container_of(ctrl->handler, struct mira130, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&mira130->sd);
	int ret = 0;
	u8 val;

	if (ctrl->id == V4L2_CID_VBLANK) {
		int exposure_max, exposure_def;

		/* Update max exposure while meeting expected vblanking */
		exposure_max = mira130_calculate_max_exposure_time(mira130->mode->row_length,
				                mira130->mode->height, ctrl->val);
		exposure_def = (exposure_max < MIRA130_DEFAULT_EXPOSURE) ?
			exposure_max : MIRA130_DEFAULT_EXPOSURE;
		__v4l2_ctrl_modify_range(mira130->exposure,
					 mira130->exposure->minimum,
					 exposure_max, mira130->exposure->step,
					 exposure_def);
	}


	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(&client->dev) == 0) {
		dev_info(&client->dev,
                         "device in use, ctrl(id:0x%x,val:0x%x) is not handled\n",
                         ctrl->id, ctrl->val);
		return 0;
	}

	if (mira130->skip_reg_upload == 0) {
		switch (ctrl->id) {
		case V4L2_CID_ANALOGUE_GAIN:
			ret = mira130_write_analog_gain_reg(mira130, ctrl->val);
			break;
		case V4L2_CID_EXPOSURE:
			ret = mira130_write_exposure_reg(mira130, ctrl->val);
			break;
		case V4L2_CID_TEST_PATTERN:
			ret = mira130_write(mira130, MIRA130_REG_TEST_PATTERN,
					       mira130_test_pattern_val[ctrl->val]);
			break;
		case V4L2_CID_HFLIP:
			if (ctrl->val != 0) {
				val = mira130_read(mira130, MIRA130_HFLIP_REG, &val);
				val = val | MIRA130_HFLIP_ENABLE_MIRROR;
				ret = mira130_write(mira130, MIRA130_HFLIP_REG,
						val);
			} else {
				val = mira130_read(mira130, MIRA130_HFLIP_REG, &val);
				val = val & (~ MIRA130_HFLIP_ENABLE_MIRROR);
				ret = mira130_write(mira130, MIRA130_HFLIP_REG,
						val);
			}
			break;
		case V4L2_CID_VFLIP:
			if (ctrl->val != 0) {
				val = mira130_read(mira130, MIRA130_VFLIP_REG, &val);
				val = val | MIRA130_VFLIP_ENABLE_FLIP;
				ret = mira130_write(mira130, MIRA130_VFLIP_REG,
						val);
			} else {
				val = mira130_read(mira130, MIRA130_VFLIP_REG, &val);
				val = val & (~ MIRA130_VFLIP_ENABLE_FLIP);
				ret = mira130_write(mira130, MIRA130_VFLIP_REG,
						val);
			}
			break;
		case V4L2_CID_VBLANK:
			ret = mira130_write16(mira130, MIRA130_VBLANK_HI_REG,
						ctrl->val + mira130->mode->height);
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

static int mira130_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mira130 *mira130 =
		container_of(ctrl->handler, struct mira130, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&mira130->sd);
	int ret = 0;

	// printk(KERN_INFO "[MIRA130]: mira130_s_ctrl() id: %X value: %X.\n", ctrl->id, ctrl->val);

	/* Previously, register writes when powered off will be buffered.
	 * The buffer will be written to sensor when start_streaming.
	 * Now, register writes happens immediately, even powered off.
	 * Register writes when powered off will fail.
	 * Users need to make sure first power on then write register.
	 */

	switch (ctrl->id) {
	case AMS_CAMERA_CID_MIRA_REG_W:
		ret = mira130_v4l2_reg_w(mira130, ctrl->val);
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

static int mira130_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mira130 *mira130 =
		container_of(ctrl->handler, struct mira130, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&mira130->sd);
	int ret = 0;

	// printk(KERN_INFO "[MIRA130]: mira130_g_ctrl() id: %X.\n", ctrl->id);

	/*
	 * Ideally, V4L2 register read should happen only when powered on.
	 * However, perhaps there are use cases that,
	 * reading other I2C addr is desired when mira sensor is powered off.
	 * Therefore, the check of "powered" flag is disabled for now.
	 */

	switch (ctrl->id) {
	case AMS_CAMERA_CID_MIRA_REG_R:
		ret = mira130_v4l2_reg_r(mira130, (u32 *)&ctrl->cur.val);
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


static const struct v4l2_ctrl_ops mira130_ctrl_ops = {
	.s_ctrl = mira130_set_ctrl,
};

static const struct v4l2_ctrl_ops mira130_custom_ctrl_ops = {
	.g_volatile_ctrl = mira130_g_ctrl,
	.s_ctrl = mira130_s_ctrl,
};


/* list of custom v4l2 ctls */
static struct v4l2_ctrl_config custom_ctrl_config_list[] = {
	/* Do not change the name field for the controls! */
	{
		.ops = &mira130_custom_ctrl_ops,
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
		.ops = &mira130_custom_ctrl_ops,
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
static int mira130_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct mira130 *mira130 = to_mira130(sd);

	if (code->pad >= NUM_PADS)
		return -EINVAL;

	if (code->pad == IMAGE_PAD) {
		if (code->index >= ARRAY_SIZE(codes))
			return -EINVAL;

		code->code = mira130_validate_format_code_or_default(mira130,
						    codes[code->index]);
	} else {
		if (code->index > 0)
			return -EINVAL;

		code->code = MEDIA_BUS_FMT_SENSOR_DATA;
	}

	return 0;
}

static int mira130_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct mira130 *mira130 = to_mira130(sd);

	if (fse->pad >= NUM_PADS)
		return -EINVAL;

	if (fse->pad == IMAGE_PAD) {
		if (fse->index >= ARRAY_SIZE(supported_modes))
			return -EINVAL;

		if (fse->code != mira130_validate_format_code_or_default(mira130, fse->code))
			return -EINVAL;

		fse->min_width = supported_modes[fse->index].width;
		fse->max_width = fse->min_width;
		fse->min_height = supported_modes[fse->index].height;
		fse->max_height = fse->min_height;
	} else {
		if (fse->code != MEDIA_BUS_FMT_SENSOR_DATA || fse->index > 0)
			return -EINVAL;

		fse->min_width = MIRA130_EMBEDDED_LINE_WIDTH;
		fse->max_width = fse->min_width;
		fse->min_height = MIRA130_NUM_EMBEDDED_LINES;
		fse->max_height = fse->min_height;
	}

	return 0;
}

static void mira130_reset_colorspace(struct v4l2_mbus_framefmt *fmt)
{
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(fmt->colorspace);
	fmt->quantization = V4L2_MAP_QUANTIZATION_DEFAULT(true,
							  fmt->colorspace,
							  fmt->ycbcr_enc);
	fmt->xfer_func = V4L2_MAP_XFER_FUNC_DEFAULT(fmt->colorspace);
}

static void mira130_update_image_pad_format(struct mira130 *mira130,
					   const struct mira130_mode *mode,
					   struct v4l2_subdev_format *fmt)
{
	if (mode != NULL) {
		printk(KERN_INFO "[MIRA130]: mira130_update_image_pad_format() width %d, height %d.\n",
				mode->width, mode->height);
	} else {
		printk(KERN_ERR "[MIRA130]: mira130_update_image_pad_format() mode is NULL.\n");
	}
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	mira130_reset_colorspace(&fmt->format);
}

static void mira130_update_metadata_pad_format(struct v4l2_subdev_format *fmt)
{
	fmt->format.width = MIRA130_EMBEDDED_LINE_WIDTH;
	fmt->format.height = MIRA130_NUM_EMBEDDED_LINES;
	fmt->format.code = MEDIA_BUS_FMT_SENSOR_DATA;
	fmt->format.field = V4L2_FIELD_NONE;

}

static int __mira130_get_pad_format(struct mira130 *mira130,
				   struct v4l2_subdev_state *sd_state,
				   struct v4l2_subdev_format *fmt)
{
	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
		struct v4l2_mbus_framefmt *try_fmt =
			v4l2_subdev_get_try_format(&mira130->sd, sd_state, fmt->pad);

		try_fmt->code = fmt->pad == IMAGE_PAD ?
				mira130_validate_format_code_or_default(mira130, try_fmt->code) :
				MEDIA_BUS_FMT_SENSOR_DATA;
		fmt->format = *try_fmt;
	} else {
		if (fmt->pad == IMAGE_PAD) {
			mira130_update_image_pad_format(mira130, mira130->mode,
						       fmt);
			fmt->format.code = mira130_validate_format_code_or_default(mira130,
							      mira130->fmt.code);
		} else {
			mira130_update_metadata_pad_format(fmt);
		}
	}

	return 0;
}

static int mira130_get_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct mira130 *mira130 = to_mira130(sd);
	int ret;

	mutex_lock(&mira130->mutex);
	ret = __mira130_get_pad_format(mira130, sd_state, fmt);
	mutex_unlock(&mira130->mutex);

	return ret;
}

static int mira130_set_pad_format(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct mira130 *mira130 = to_mira130(sd);
	const struct mira130_mode *mode;
	struct v4l2_mbus_framefmt *framefmt;
	u32 max_exposure = 0, default_exp = 0;

	if (fmt->pad >= NUM_PADS)
		return -EINVAL;

	mutex_lock(&mira130->mutex);

	if (fmt->pad == IMAGE_PAD) {
		/* Validate format or use default */
		fmt->format.code = mira130_validate_format_code_or_default(mira130,
									  fmt->format.code);

		mode = v4l2_find_nearest_size(supported_modes,
					      ARRAY_SIZE(supported_modes),
					      width, height,
					      fmt->format.width,
					      fmt->format.height);
		mira130_update_image_pad_format(mira130, mode, fmt);
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			printk(KERN_INFO "[MIRA130]: mira130_set_pad_format() use try_format.\n");
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		} else if (mira130->mode != mode ||
			mira130->fmt.code != fmt->format.code) {

			printk(KERN_INFO "[MIRA130]: mira130_set_pad_format() use new mode.\n");
			printk(KERN_INFO "[MIRA130]: mira130->mode %p mode %p.\n", (void *)mira130->mode, (void *)mode);
			printk(KERN_INFO "[MIRA130]: mira130->fmt.code 0x%x fmt->format.code 0x%x.\n", mira130->fmt.code, fmt->format.code);

			mira130->fmt = fmt->format;
			mira130->mode = mode;

			// Update controls based on new mode (range and current value).
			max_exposure = mira130_calculate_max_exposure_time(mira130->mode->row_length,
									   mira130->mode->height,
									   mira130->mode->vblank);
			default_exp = MIRA130_DEFAULT_EXPOSURE > max_exposure ? max_exposure : MIRA130_DEFAULT_EXPOSURE;
			printk(KERN_INFO "[MIRA130]: mira130_set_pad_format() min_exp %d max_exp %d, default_exp %d\n",
					MIRA130_EXPOSURE_MIN, max_exposure, default_exp);
			__v4l2_ctrl_modify_range(mira130->exposure,
						     MIRA130_EXPOSURE_MIN,
						     max_exposure, 1,
						     default_exp);

			// Set the current vblank value
			printk(KERN_INFO "[MIRA130]: mira130_set_pad_format() mira130->mode->vblank %d\n",
					mira130->mode->vblank);

			__v4l2_ctrl_s_ctrl(mira130->vblank, mira130->mode->vblank);
		}
	} else {
		if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
			framefmt = v4l2_subdev_get_try_format(sd, sd_state,
							      fmt->pad);
			*framefmt = fmt->format;
		} else {
			/* Only one embedded data mode is supported */
			mira130_update_metadata_pad_format(fmt);
		}
	}

	printk(KERN_INFO "[MIRA130]: mira130_set_pad_format() to unlock and return.\n");

	mutex_unlock(&mira130->mutex);

	return 0;
}

static int mira130_set_framefmt(struct mira130 *mira130)
{
	if (mira130->skip_reg_upload == 0) {
		switch (mira130->fmt.code) {
		case MEDIA_BUS_FMT_Y10_1X10:
		case MEDIA_BUS_FMT_SGRBG10_1X10:
			printk(KERN_INFO "[MIRA130]: mira130_set_framefmt() write 10 bpp regs.\n");
			mira130_write(mira130, MIRA130_BIT_DEPTH_REG,MIRA130_BIT_DEPTH_10_BIT);
			mira130_write(mira130, MIRA130_CSI_DATA_TYPE_REG,
				MIRA130_CSI_DATA_TYPE_10_BIT);
			return 0;
		default:
			printk(KERN_ERR "Unknown format requested %d", mira130->fmt.code);
		}
	}

	return -EINVAL;
}

static const struct v4l2_rect *
__mira130_get_pad_crop(struct mira130 *mira130, struct v4l2_subdev_state *sd_state,
		      unsigned int pad, enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_crop(&mira130->sd, sd_state, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &mira130->mode->crop;
	}

	return NULL;
}

static int mira130_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP: {
		struct mira130 *mira130 = to_mira130(sd);

		mutex_lock(&mira130->mutex);
		sel->r = *__mira130_get_pad_crop(mira130, sd_state, sel->pad,
						sel->which);
		mutex_unlock(&mira130->mutex);

		return 0;
	}

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.top = 0;
		sel->r.left = 0;
		sel->r.width = MIRA130_NATIVE_WIDTH;
		sel->r.height = MIRA130_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.top = MIRA130_PIXEL_ARRAY_TOP;
		sel->r.left = MIRA130_PIXEL_ARRAY_LEFT;
		sel->r.width = MIRA130_PIXEL_ARRAY_WIDTH;
		sel->r.height = MIRA130_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

static int mira130_start_streaming(struct mira130 *mira130)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira130->sd);
	const struct mira130_reg_list *reg_list;
	int ret;

	printk(KERN_INFO "[MIRA130]: Entering start streaming function.\n");

	/* Follow examples of other camera driver, here use pm_runtime_resume_and_get */
	ret = pm_runtime_resume_and_get(&client->dev);

	if (ret < 0) {
		//printk(KERN_INFO "[MIRA130]: get_sync failed, but continue.\n");
		pm_runtime_put_noidle(&client->dev);
		return ret;
	}

	/* Apply default values of current mode */
	if (mira130->skip_reg_upload == 0) {
		/* Stop treaming before uploading register sequence */
		printk(KERN_INFO "[MIRA130]: Writing stop streaming regs.\n");
		ret = mira130_write_stop_streaming_regs(mira130);
		if (ret) {
			dev_err(&client->dev, "Could not write stream-on sequence");
			goto err_rpm_put;
		}

		reg_list = &mira130->mode->reg_list;
		printk(KERN_INFO "[MIRA130]: Write %d regs.\n", reg_list->num_of_regs);
		ret = mira130_write_regs(mira130, reg_list->regs, reg_list->num_of_regs);
		if (ret) {
			dev_err(&client->dev, "%s failed to set mode\n", __func__);
			goto err_rpm_put;
		}

		ret = mira130_set_framefmt(mira130);
		if (ret) {
			dev_err(&client->dev, "%s failed to set frame format: %d\n",
				__func__, ret);
			goto err_rpm_put;
		}
	} else {
		printk(KERN_INFO "[MIRA130]: Skip base register sequence upload, due to mira130->skip_reg_upload=%u.\n", mira130->skip_reg_upload);
	}


	printk(KERN_INFO "[MIRA130]: Entering v4l2 ctrl handler setup function.\n");

	/* Apply customized values from user */
	ret =  __v4l2_ctrl_handler_setup(mira130->sd.ctrl_handler);
	printk(KERN_INFO "[MIRA130]: __v4l2_ctrl_handler_setup ret = %d.\n", ret);
	if (ret)
		goto err_rpm_put;

	if (mira130->skip_reg_upload == 0 ||
		(mira130->skip_reg_upload == 1 && mira130->force_stream_ctrl == 1) ) {
		printk(KERN_INFO "[MIRA130]: Writing start streaming regs.\n");
		ret = mira130_write_start_streaming_regs(mira130);
		if (ret) {
			dev_err(&client->dev, "Could not write stream-on sequence");
			goto err_rpm_put;
		}
	} else {
		printk(KERN_INFO "[MIRA130]: Skip write_start_streaming_regs due to skip_reg_upload == %d and force_stream_ctrl == %d.\n",
				mira130->skip_reg_upload, mira130->force_stream_ctrl);
	}

	/* vflip and hflip cannot change during streaming */
	printk(KERN_INFO "[MIRA130]: Entering v4l2 ctrl grab vflip grab vflip.\n");
	__v4l2_ctrl_grab(mira130->vflip, true);
	printk(KERN_INFO "[MIRA130]: Entering v4l2 ctrl grab vflip grab hflip.\n");
	__v4l2_ctrl_grab(mira130->hflip, true);

	return 0;

err_rpm_put:
	pm_runtime_put(&client->dev);
	return ret;
}

static void mira130_stop_streaming(struct mira130 *mira130)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira130->sd);
	int ret = 0;

	/* Unlock controls for vflip and hflip */
	__v4l2_ctrl_grab(mira130->vflip, false);
	__v4l2_ctrl_grab(mira130->hflip, false);

	if (mira130->skip_reset == 0) {
		if (mira130->skip_reg_upload == 0 ||
			(mira130->skip_reg_upload == 1 && mira130->force_stream_ctrl == 1) ) {

			ret = mira130_write_stop_streaming_regs(mira130);
			if (ret) {
				dev_err(&client->dev, "Could not write the stream-off sequence");
			}
		} else {
			printk(KERN_INFO "[MIRA130]: Skip write_stop_streaming_regs due to skip_reg_upload == %d and force_stream_ctrl == %d.\n",
					mira130->skip_reg_upload, mira130->force_stream_ctrl);
		}
	} else {
		printk(KERN_INFO "[MIRA130]: Skip write_stop_streaming_regs due to mira130->skip_reset == %d.\n", mira130->skip_reset);
	}

	pm_runtime_put(&client->dev);
}

static int mira130_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct mira130 *mira130 = to_mira130(sd);
	int ret = 0;

	mutex_lock(&mira130->mutex);
	if (mira130->streaming == enable) {
		mutex_unlock(&mira130->mutex);
		return 0;
	}

	printk(KERN_INFO "[MIRA130]: Entering mira130_set_stream enable: %d.\n", enable);

	if (enable) {
		/*
		 * Apply default & customized values
		 * and then start streaming.
		 */
		ret = mira130_start_streaming(mira130);
		if (ret)
			goto err_unlock;
	} else {
		mira130_stop_streaming(mira130);
	}

	mira130->streaming = enable;

	mutex_unlock(&mira130->mutex);

	printk(KERN_INFO "[MIRA130]: Returning mira130_set_stream with ret: %d.\n", ret);

	return ret;

err_unlock:
	mutex_unlock(&mira130->mutex);

	return ret;
}

static int __maybe_unused mira130_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira130 *mira130 = to_mira130(sd);

	printk(KERN_INFO "[MIRA130]: Entering suspend function.\n");

	if (mira130->streaming)
		mira130_stop_streaming(mira130);

	return 0;
}

static int __maybe_unused mira130_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira130 *mira130 = to_mira130(sd);
	int ret;

	printk(KERN_INFO "[MIRA130]: Entering resume function.\n");

	if (mira130->streaming) {
		ret = mira130_start_streaming(mira130);
		if (ret)
			goto error;
	}

	return 0;

error:
	mira130_stop_streaming(mira130);
	mira130->streaming = false;

	return ret;
}

static int mira130_get_regulators(struct mira130 *mira130)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira130->sd);
	unsigned int i;

	for (i = 0; i < MIRA130_NUM_SUPPLIES; i++)
		mira130->supplies[i].supply = mira130_supply_name[i];

	return devm_regulator_bulk_get(&client->dev,
				       MIRA130_NUM_SUPPLIES,
				       mira130->supplies);
}

/* OTP power on */
static int mira130_otp_power_on(struct mira130 *mira130)
{
	int ret;

	ret = mira130_write(mira130, 0x0080, 0x04);

	return 0;
}

/* OTP power off */
static int mira130_otp_power_off(struct mira130 *mira130)
{
	int ret;

	ret = mira130_write(mira130, 0x0080, 0x08);

	return 0;
}

/* OTP power on */
static int mira130_otp_read(struct mira130 *mira130, u8 addr, u8 offset, u8 *val)
{
	int ret;

	ret = mira130_write(mira130, 0x0086, addr);
	ret = mira130_write(mira130, 0x0080, 0x02);
	ret = mira130_read(mira130, 0x0082 + offset, val);
	return 0;
}


/* Verify chip ID */
static int mira130_identify_module(struct mira130 *mira130)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira130->sd);
	int ret;
	u8 val;

	mira130_otp_power_on(mira130);

	usleep_range(100, 110);

	ret = mira130_otp_read(mira130, 0x0d, 0, &val);
	dev_err(&client->dev, "Read OTP add 0x0d with val %x\n", val);

	mira130_otp_power_off(mira130);

	val = 0;
	mira130_read(mira130, 0x3107, &val);
	printk(KERN_INFO "[MIRA130]: %s Sensor ID high byte %X.\n", __func__, val);
	mira130_read(mira130, 0x3108, &val);
	printk(KERN_INFO "[MIRA130]: %s Sensor ID low byte %X.\n", __func__, val);

	return 0;
}

static const struct v4l2_subdev_core_ops mira130_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_video_ops mira130_video_ops = {
	.s_stream = mira130_set_stream,
};

static const struct v4l2_subdev_pad_ops mira130_pad_ops = {
	.enum_mbus_code = mira130_enum_mbus_code,
	.get_fmt = mira130_get_pad_format,
	.set_fmt = mira130_set_pad_format,
	.get_selection = mira130_get_selection,
	.enum_frame_size = mira130_enum_frame_size,
};

static const struct v4l2_subdev_ops mira130_subdev_ops = {
	.core = &mira130_core_ops,
	.video = &mira130_video_ops,
	.pad = &mira130_pad_ops,
};

static const struct v4l2_subdev_internal_ops mira130_internal_ops = {
	.open = mira130_open,
};

/* Initialize control handlers */
static int mira130_init_controls(struct mira130 *mira130)
{
	struct i2c_client *client = v4l2_get_subdevdata(&mira130->sd);
	struct v4l2_ctrl_handler *ctrl_hdlr;
	struct v4l2_fwnode_device_properties props;
	int ret;
	struct v4l2_ctrl_config *mira130_reg_w;
	struct v4l2_ctrl_config *mira130_reg_r;

	u32 max_exposure = 0;

	ctrl_hdlr = &mira130->ctrl_handler;
	/* v4l2_ctrl_handler_init gives a hint/guess of the number of v4l2_ctrl_new */
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 16);
	if (ret)
		return ret;

	mutex_init(&mira130->mutex);
	ctrl_hdlr->lock = &mira130->mutex;

	printk(KERN_INFO "[MIRA130]: %s V4L2_CID_PIXEL_RATE %X.\n", __func__, V4L2_CID_PIXEL_RATE);

	/* By default, PIXEL_RATE is read only */
	mira130->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &mira130_ctrl_ops,
					        V4L2_CID_PIXEL_RATE,
					        MIRA130_PIXEL_RATE,
					        MIRA130_PIXEL_RATE, 1,
					        MIRA130_PIXEL_RATE);

	printk(KERN_INFO "[MIRA130]: %s V4L2_CID_VBLANK %X.\n", __func__, V4L2_CID_VBLANK);

	mira130->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &mira130_ctrl_ops,
					   V4L2_CID_VBLANK, MIRA130_MIN_VBLANK,
					   MIRA130_MAX_VBLANK, 1,
					   mira130->mode->vblank);

	printk(KERN_INFO "[MIRA130]: %s V4L2_CID_HBLANK %X.\n", __func__, V4L2_CID_HBLANK);

	mira130->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &mira130_ctrl_ops,
					   V4L2_CID_HBLANK, mira130->mode->hblank,
					   mira130->mode->hblank, 1,
					   mira130->mode->hblank);

	// Make the vblank control read only. This could be changed to allow changing framerate in
	// runtime, but would require adapting other settings
	// mira130->vblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	// Exposure is indicated in number of lines here
	// Max is determined by vblank + vsize and Tglob.
	max_exposure = mira130_calculate_max_exposure_time(mira130->mode->row_length,
	                                                   mira130->mode->height,
	                                                   mira130->mode->vblank);

	printk(KERN_INFO "[MIRA130]: %s V4L2_CID_EXPOSURE %X.\n", __func__, V4L2_CID_EXPOSURE);

	mira130->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &mira130_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     MIRA130_EXPOSURE_MIN, max_exposure,
					     1,
					     MIRA130_DEFAULT_EXPOSURE);

	printk(KERN_INFO "[MIRA130]: %s V4L2_CID_ANALOGUE_GAIN %X.\n", __func__, V4L2_CID_ANALOGUE_GAIN);

	mira130->gain = v4l2_ctrl_new_std(ctrl_hdlr, &mira130_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  0, ARRAY_SIZE(analog_gain_lut) - 1,
			  1, 0);

	printk(KERN_INFO "[MIRA130]: %s V4L2_CID_HFLIP %X.\n", __func__, V4L2_CID_HFLIP);

	mira130->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &mira130_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 0);
	if (mira130->hflip)
		mira130->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	printk(KERN_INFO "[MIRA130]: %s V4L2_CID_VFLIP %X.\n", __func__, V4L2_CID_VFLIP);

	mira130->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &mira130_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 0);
	if (mira130->vflip)
		mira130->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	printk(KERN_INFO "[MIRA130]: %s V4L2_CID_TEST_PATTERN %X.\n", __func__, V4L2_CID_TEST_PATTERN);
	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &mira130_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(mira130_test_pattern_menu) - 1,
				     0, 0, mira130_test_pattern_menu);
	/*
	 * Custom op
	 */
	mira130_reg_w = &custom_ctrl_config_list[0];
	printk(KERN_INFO "[MIRA130]: %s AMS_CAMERA_CID_MIRA_REG_W %X.\n", __func__, AMS_CAMERA_CID_MIRA_REG_W);
	mira130->mira130_reg_w = v4l2_ctrl_new_custom(ctrl_hdlr, mira130_reg_w, NULL);

	mira130_reg_r = &custom_ctrl_config_list[1];
	printk(KERN_INFO "[MIRA130]: %s AMS_CAMERA_CID_MIRA_REG_R %X.\n", __func__, AMS_CAMERA_CID_MIRA_REG_R);
	mira130->mira130_reg_r = v4l2_ctrl_new_custom(ctrl_hdlr, mira130_reg_r, NULL);
	if (mira130->mira130_reg_r)
		mira130->mira130_reg_r->flags |= (V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "%s control init failed (%d)\n",
			__func__, ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &mira130_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	mira130->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);
	mutex_destroy(&mira130->mutex);

	return ret;
}

static void mira130_free_controls(struct mira130 *mira130)
{
	v4l2_ctrl_handler_free(mira130->sd.ctrl_handler);
	mutex_destroy(&mira130->mutex);
}

static int mira130_check_hwcfg(struct device *dev)
{
	struct fwnode_handle *endpoint;
	struct v4l2_fwnode_endpoint ep_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	int ret = -EINVAL;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint) {
		dev_err(dev, "endpoint node not found\n");
		return -EINVAL;
	}

	if (v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep_cfg)) {
		dev_err(dev, "could not parse endpoint\n");
		goto error_out;
	}

	/* Check the number of MIPI CSI2 data lanes */
	if (ep_cfg.bus.mipi_csi2.num_data_lanes != 2) {
		dev_err(dev, "only 2 data lanes are currently supported\n");
		goto error_out;
	}

	/* Check the link frequency set in device tree */
	if (!ep_cfg.nr_of_link_frequencies) {
		dev_err(dev, "link-frequency property not found in DT\n");
		goto error_out;
	}

	if (ep_cfg.nr_of_link_frequencies != 1 ||
	    ep_cfg.link_frequencies[0] != MIRA130_DEFAULT_LINK_FREQ) {
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

static int mira130pmic_init_controls(struct i2c_client *client)
{
	int ret;
	u8 val;

	ret = mira130pmic_write(client, 0x62, 0x00);
	ret = mira130pmic_write(client, 0x61, 0x00);

	ret = mira130pmic_read(client, 0x61, &val);
	dev_err(&client->dev, "Read 0x61 with val %x\n", val);


	usleep_range(100, 110);

	ret = mira130pmic_write(client, 0x05, 0x00);
	ret = mira130pmic_write(client, 0x0e, 0x00);
	ret = mira130pmic_write(client, 0x11, 0x00);
	ret = mira130pmic_write(client, 0x14, 0x00);
	ret = mira130pmic_write(client, 0x17, 0x00);
	ret = mira130pmic_write(client, 0x1a, 0x00);
	ret = mira130pmic_write(client, 0x1c, 0x00);
	ret = mira130pmic_write(client, 0x1d, 0x00);
	ret = mira130pmic_write(client, 0x1e, 0x00);
	ret = mira130pmic_write(client, 0x1f, 0x00);

	ret = mira130pmic_write(client, 0x24, 0x48);
	ret = mira130pmic_write(client, 0x20, 0x00);
	ret = mira130pmic_write(client, 0x21, 0x00);
	ret = mira130pmic_write(client, 0x1a, 0x00);
	ret = mira130pmic_write(client, 0x01, 0x00);
	ret = mira130pmic_write(client, 0x08, 0x00);
	ret = mira130pmic_write(client, 0x02, 0x00);
	ret = mira130pmic_write(client, 0x0b, 0x00);
	ret = mira130pmic_write(client, 0x14, 0x00);
	ret = mira130pmic_write(client, 0x17, 0x00);
	ret = mira130pmic_write(client, 0x1c, 0x00);
	ret = mira130pmic_write(client, 0x1d, 0x00);
	ret = mira130pmic_write(client, 0x1f, 0x00);

	usleep_range(50, 60);

	ret = mira130pmic_write(client, 0x62, 0x0d);

	usleep_range(50, 60);
	usleep_range(50000, 50000+100);

	ret = mira130pmic_write(client, 0x27, 0xff);
	ret = mira130pmic_write(client, 0x28, 0xff);
	ret = mira130pmic_write(client, 0x29, 0xff);
	ret = mira130pmic_write(client, 0x2a, 0xff);
	ret = mira130pmic_write(client, 0x2b, 0xff);

	ret = mira130pmic_write(client, 0x41, 0x04);
	usleep_range(50, 60);

	ret = mira130pmic_read(client, 0x20, &val);
	dev_err(&client->dev, "Read 0x20 with val %x\n", val);

	// PCB V2.0 or above, enable LDO9=2.50V for VDD25
	ret = mira130pmic_write(client, 0x20, 0xb2);
	// For PCB V1.0, VDD28 on 2.85V for older PCBs
	// ret = mira130pmic_write(client, 0x20, 0xb9);

	ret = mira130pmic_read(client, 0x20, &val);
	dev_err(&client->dev, "Read 0x20 with val %x\n", val);

	usleep_range(700, 710);

	ret = mira130pmic_write(client, 0x12, 0x16);
	ret = mira130pmic_write(client, 0x10, 0x16);
	ret = mira130pmic_write(client, 0x11, 0x96);
	ret = mira130pmic_write(client, 0x1e, 0x96);
	ret = mira130pmic_write(client, 0x21, 0x96);
	usleep_range(50, 60);

	ret = mira130pmic_write(client, 0x00, 0x04);
	ret = mira130pmic_write(client, 0x04, 0x34);
	ret = mira130pmic_write(client, 0x06, 0xbf);
	ret = mira130pmic_write(client, 0x05, 0xb4);
	ret = mira130pmic_write(client, 0x03, 0x00);
	ret = mira130pmic_write(client, 0x0d, 0x34);
	ret = mira130pmic_write(client, 0x0f, 0xbf);
	ret = mira130pmic_write(client, 0x0e, 0xb4);
	usleep_range(50, 60);

	ret = mira130pmic_write(client, 0x42, 0x05);
	usleep_range(50, 60);

	ret = mira130pmic_write(client, 0x45, 0x40);
	ret = mira130pmic_write(client, 0x57, 0x02);
	ret = mira130pmic_write(client, 0x5d, 0x10);
	ret = mira130pmic_write(client, 0x61, 0x10);

	return 0;
}


static int mira130_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct mira130 *mira130;
	int ret;

	printk(KERN_INFO "[MIRA130]: probing v4l2 sensor.\n");
	printk(KERN_INFO "[MIRA130]: Driver Version 0.0.\n");

	dev_err(dev, "[MIRA130] name: %s.\n", client->name);

	mira130 = devm_kzalloc(&client->dev, sizeof(*mira130), GFP_KERNEL);
	if (!mira130)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&mira130->sd, client, &mira130_subdev_ops);

	/* Check the hardware configuration in device tree */
	if (mira130_check_hwcfg(dev))
		return -EINVAL;

	/* Parse device tree to check if dtoverlay has param skip-reg-upload=1 */
        device_property_read_u32(dev, "skip-reg-upload", &mira130->skip_reg_upload);
	printk(KERN_INFO "[MIRA130]: skip-reg-upload %d.\n", mira130->skip_reg_upload);
	/* Set default TBD I2C device address to LED I2C Address*/
	mira130->tbd_client_i2c_addr = MIRA130LED_I2C_ADDR;
	printk(KERN_INFO "[MIRA130]: User defined I2C device address defaults to LED driver I2C address 0x%X.\n", mira130->tbd_client_i2c_addr);


	/* Get system clock (xclk) */
	mira130->xclk = devm_clk_get(dev, NULL);
	if (IS_ERR(mira130->xclk)) {
		dev_err(dev, "failed to get xclk\n");
		return PTR_ERR(mira130->xclk);
	}

	mira130->xclk_freq = clk_get_rate(mira130->xclk);
	if (mira130->xclk_freq != MIRA130_SUPPORTED_XCLK_FREQ) {
		dev_err(dev, "xclk frequency not supported: %d Hz\n",
			mira130->xclk_freq);
		return -EINVAL;
	}

	ret = mira130_get_regulators(mira130);
	if (ret) {
		dev_err(dev, "failed to get regulators\n");
		return ret;
	}

	/* Request optional enable pin */
	// mira130->reset_gpio = devm_gpiod_get_optional(dev, "reset",
	//					     GPIOD_OUT_HIGH);

	{
		printk(KERN_INFO "[MIRA130]: Init PMIC.\n");
		mira130->pmic_client = i2c_new_dummy_device(client->adapter,
				MIRA130PMIC_I2C_ADDR);
		if (IS_ERR(mira130->pmic_client))
			return PTR_ERR(mira130->pmic_client);
		mira130->uc_client = i2c_new_dummy_device(client->adapter,
				MIRA130UC_I2C_ADDR);
		if (IS_ERR(mira130->uc_client))
			return PTR_ERR(mira130->uc_client);
		mira130->led_client = i2c_new_dummy_device(client->adapter,
				MIRA130LED_I2C_ADDR);
		if (IS_ERR(mira130->led_client))
			return PTR_ERR(mira130->led_client);

		mira130pmic_init_controls(mira130->pmic_client);
	}

	dev_err(dev, "[MIRA130] Sleep for 1 second to let PMIC driver complete init.\n");
	usleep_range(1000000, 1000000+100);

	/*
	 * The sensor must be powered for mira130_identify_module()
	 * to be able to read the CHIP_ID register
	 */
	ret = mira130_power_on(dev);
	if (ret)
		return ret;

	printk(KERN_INFO "[MIRA130]: Entering identify function.\n");

	ret = mira130_identify_module(mira130);
	if (ret)
		goto error_power_off;

	printk(KERN_INFO "[MIRA130]: Setting support function.\n");

	/* Set default mode to max resolution */
	mira130->mode = &supported_modes[0];

	printk(KERN_INFO "[MIRA130]: Entering init controls function.\n");

	ret = mira130_init_controls(mira130);
	if (ret)
		goto error_power_off;

	/* Initialize subdev */
	mira130->sd.internal_ops = &mira130_internal_ops;
	mira130->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE |
			    V4L2_SUBDEV_FL_HAS_EVENTS;
	mira130->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	/* Initialize source pads */
	mira130->pad[IMAGE_PAD].flags = MEDIA_PAD_FL_SOURCE;
	mira130->pad[METADATA_PAD].flags = MEDIA_PAD_FL_SOURCE;

	printk(KERN_INFO "[MIRA130]: Entering set default format function.\n");

	/* Initialize default format */
	mira130_set_default_format(mira130);

	printk(KERN_INFO "[MIRA130]: Entering pads init function.\n");

	ret = media_entity_pads_init(&mira130->sd.entity, NUM_PADS, mira130->pad);
	if (ret) {
		dev_err(dev, "failed to init entity pads: %d\n", ret);
		goto error_handler_free;
	}

	printk(KERN_INFO "[MIRA130]: Entering subdev sensor common function.\n");

	ret = v4l2_async_register_subdev_sensor(&mira130->sd);
	if (ret < 0) {
		dev_err(dev, "failed to register sensor sub-device: %d\n", ret);
		goto error_media_entity;
	}

	/* Enable runtime PM and turn off the device */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

error_media_entity:
	media_entity_cleanup(&mira130->sd.entity);

error_handler_free:
	mira130_free_controls(mira130);

error_power_off:
	mira130_power_off(dev);

	i2c_unregister_device(mira130->pmic_client);
	i2c_unregister_device(mira130->uc_client);
	i2c_unregister_device(mira130->led_client);

	return ret;
}

static void mira130_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct mira130 *mira130 = to_mira130(sd);

	i2c_unregister_device(mira130->pmic_client);
	i2c_unregister_device(mira130->uc_client);
	i2c_unregister_device(mira130->led_client);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	mira130_free_controls(mira130);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		mira130_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);

}

static const struct dev_pm_ops mira130_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mira130_suspend, mira130_resume)
	SET_RUNTIME_PM_OPS(mira130_power_off, mira130_power_on, NULL)
};

#endif // __MIRA130_INL__

