// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2018 Intel Corporation

#include <linux/acpi.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>

#include <media/v4l2-cci.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#include "ccs/ccs-regs.h"

/*
 * The IMX363 is not MIPI CCS compliant: it does not implement the CCS
 * capability and limit register space (0x0040--0x0fff and 0x1000--0x1fff),
 * without which drivers/media/i2c/ccs/ cannot compute anything. It does
 * implement a subset of the CCS/SMIA register layout though, so the CCS
 * register names are used for those registers here. Everything else is Sony
 * specific and undocumented.
 * The registers shared with the CCS/SMIA layout are written as full 16-bit
 * CCS registers; the register sequences from the vendor driver only ever
 * touched their least significant byte, whose neighbours default to zero.
 */

/* V-timing */
#define IMX363_VTS_30FPS			3136
#define IMX363_VTS_30FPS_HD			1296
#define IMX363_VTS_MAX				65525

/* HBLANK control - read only */
#define IMX363_PPL_DEFAULT			8832

#define IMX363_SENSOR_MODEL_ID			0x0363

/* Exposure control */
#define IMX363_EXPOSURE_OFFSET			10
#define IMX363_EXPOSURE_MIN			4
#define IMX363_EXPOSURE_STEP			1
#define IMX363_EXPOSURE_DEFAULT			0x640
#define IMX363_EXPOSURE_MAX			(IMX363_VTS_MAX - \
						 IMX363_EXPOSURE_OFFSET)

/* Analog gain control */
#define IMX363_ANA_GAIN_MIN			0
#define IMX363_ANA_GAIN_MAX			480
#define IMX363_ANA_GAIN_STEP			1
#define IMX363_ANA_GAIN_DEFAULT			0

/* Digital gain control */
#define IMX363_REG_DIGITAL_GAIN_RED		CCI_REG16(0x0210)
#define IMX363_REG_DIGITAL_GAIN_BLUE		CCI_REG16(0x0212)
#define IMX363_REG_DIGITAL_GAIN_GREENB		CCI_REG16(0x0214)
#define IMX363_DGTL_GAIN_MIN			0
#define IMX363_DGTL_GAIN_MAX			4096	/* Max = 0xfff */
#define IMX363_DGTL_GAIN_DEFAULT		1024
#define IMX363_DGTL_GAIN_STEP			1

/* HDR control */
#define IMX363_EXPOSURE_RATIO_MAX		5




/*
 * CCS_R_REQUESTED_LINK_RATE is a 32-bit fixed point (16.16) value in Mbps.
 * It is written as two 16-bit halves to match the vendor sequence.
 */
#define IMX363_REG_REQUESTED_LINK_RATE		CCI_REG16(0x0820)
#define IMX363_REG_REQUESTED_LINK_RATE_FRAC	CCI_REG16(0x0822)

/* Sony specific registers */
#define IMX363_REG_CLK_BLANK_STOP		CCI_REG8(0x4040)

/* IMX363 native and active pixel array size. */
#define IMX363_NATIVE_WIDTH			4048U
#define IMX363_NATIVE_HEIGHT			3168U
#define IMX363_PIXEL_ARRAY_LEFT			8U
#define IMX363_PIXEL_ARRAY_TOP			24U
#define IMX363_PIXEL_ARRAY_WIDTH		4032U
#define IMX363_PIXEL_ARRAY_HEIGHT		3024U

#define IMX363_EXTCLK_FREQ			24000000

/*
 * Initialisation delay between XCLR low->high and the moment when the sensor
 * can start capture (i.e. can leave software standby), given by T7 in the
 * datasheet is 8ms.  This does include I2C setup time as well.
 *
 * Note, that delay between XCLR low->high and reading the CCI ID register (T6
 * in the datasheet) is much smaller - 600us.
 */
#define IMX519_XCLR_MIN_DELAY_US	8000
#define IMX519_XCLR_DELAY_RANGE_US	1000

struct imx363_reg_list {
	u32 num_of_regs;
	const struct cci_reg_sequence *regs;
};

struct imx363_link_cfg {
	unsigned int lf_to_pix_rate_factor;
	struct imx363_reg_list reg_list;
};

enum {
	IMX363_2_LANE_MODE,
	IMX363_4_LANE_MODE,
	IMX363_LANE_CONFIGS,
};

struct imx363_link_freq_config {
	u32 pixels_per_line;

	/* Configuration for this link frequency / num lanes selection */
	struct imx363_link_cfg link_cfg[IMX363_LANE_CONFIGS];
};

struct imx363_mode {
	/* Frame width */
	u32 width;
	/* Frame height */
	u32 height;

	/* V-timing */
	u32 vts_def;
	u32 vts_min;

	/* Index of Link frequency config to be used */
	u32 link_freq_index;
	/* Default register values */
	struct imx363_reg_list reg_list;

	/* Analog crop rectangle */
	struct v4l2_rect crop;
};

static const struct cci_reg_sequence mipi_2300mbps_24mhz_2l[] = {
	{ CCS_R_EXTCLK_FREQUENCY_MHZ, 0x1800 },
	{ CCS_R_VT_PIX_CLK_DIV, 10 },
	{ CCS_R_VT_SYS_CLK_DIV, 2 },
	{ CCS_R_PRE_PLL_CLK_DIV, 4 },
	{ CCS_R_PLL_MULTIPLIER, 212 },
	{ CCS_R_OP_PIX_CLK_DIV, 10 },
	{ CCS_R_OP_SYS_CLK_DIV, 1 },
	{ CCS_R_OP_PRE_PLL_CLK_DIV, 2 },
	{ CCS_R_OP_PLL_MULTIPLIER, 216 },
	{ CCS_R_PLL_MODE, 0 },

	{ CCS_R_CSI_LANE_MODE, 1 },
};

static const struct cci_reg_sequence mipi_2300mbps_24mhz_4l[] = {
	{ CCS_R_EXTCLK_FREQUENCY_MHZ, 0x1800 },
	{ CCS_R_VT_PIX_CLK_DIV, 3 },
	{ CCS_R_VT_SYS_CLK_DIV, 2 },
	{ CCS_R_PRE_PLL_CLK_DIV, 4 },
	{ CCS_R_PLL_MULTIPLIER, 208 },
	{ CCS_R_OP_PIX_CLK_DIV, 10 },
	{ CCS_R_OP_SYS_CLK_DIV, 1 },
	{ CCS_R_OP_PRE_PLL_CLK_DIV, 4 },
	{ CCS_R_OP_PLL_MULTIPLIER, 230 },
	{ CCS_R_PLL_MODE, 1 },

	{ CCS_R_CSI_LANE_MODE, 3 },
};

static const struct cci_reg_sequence mipi_642mbps_24mhz_2l[] = {
	{ CCS_R_EXTCLK_FREQUENCY_MHZ, 0x1800 },
	{ CCS_R_VT_PIX_CLK_DIV, 5 },
	{ CCS_R_VT_SYS_CLK_DIV, 2 },
	{ CCS_R_PRE_PLL_CLK_DIV, 4 },
	{ CCS_R_PLL_MULTIPLIER, 107 },
	{ CCS_R_OP_PIX_CLK_DIV, 10 },
	{ CCS_R_OP_SYS_CLK_DIV, 1 },
	{ CCS_R_OP_PRE_PLL_CLK_DIV, 2 },
	{ CCS_R_OP_PLL_MULTIPLIER, 216 },
	{ CCS_R_PLL_MODE, 0 },

	{ CCS_R_CSI_LANE_MODE, 1 },
	{ IMX363_REG_REQUESTED_LINK_RATE, 642 * 2 },
	{ IMX363_REG_REQUESTED_LINK_RATE_FRAC, 0 },
};

static const struct cci_reg_sequence mipi_642mbps_24mhz_4l[] = {
	{ CCS_R_EXTCLK_FREQUENCY_MHZ, 0x1800 },
	{ CCS_R_VT_PIX_CLK_DIV, 5 },
	{ CCS_R_VT_SYS_CLK_DIV, 2 },
	{ CCS_R_PRE_PLL_CLK_DIV, 4 },
	{ CCS_R_PLL_MULTIPLIER, 107 },
	{ CCS_R_OP_PIX_CLK_DIV, 10 },
	{ CCS_R_OP_SYS_CLK_DIV, 1 },
	{ CCS_R_OP_PRE_PLL_CLK_DIV, 2 },
	{ CCS_R_OP_PLL_MULTIPLIER, 216 },
	{ CCS_R_PLL_MODE, 0 },

	{ CCS_R_CSI_LANE_MODE, 3 },
	{ IMX363_REG_REQUESTED_LINK_RATE, 642 * 4 },
	{ IMX363_REG_REQUESTED_LINK_RATE_FRAC, 0 },
};

/*
 * Undocumented Sony settings, taken from the vendor driver register traces.
 * They are reproduced verbatim, byte by byte, as their meaning is unknown.
 */
static const struct cci_reg_sequence mode_common_regs[] = {
	{ CCI_REG8(0x0136), 0x18 },
	{ CCI_REG8(0x0137), 0x00 },
	{ CCI_REG8(0x31a3), 0x00 },
	{ CCI_REG8(0x64d4), 0x01 },
	{ CCI_REG8(0x64d5), 0xaa },
	{ CCI_REG8(0x64d6), 0x01 },
	{ CCI_REG8(0x64d7), 0xa9 },
	{ CCI_REG8(0x64d8), 0x01 },
	{ CCI_REG8(0x64d9), 0xa5 },
	{ CCI_REG8(0x64da), 0x01 },
	{ CCI_REG8(0x64db), 0xa1 },
	{ CCI_REG8(0x720a), 0x24 },
	{ CCI_REG8(0x720b), 0x89 },
	{ CCI_REG8(0x720c), 0x85 },
	{ CCI_REG8(0x720d), 0xa1 },
	{ CCI_REG8(0x720e), 0x6e },
	{ CCI_REG8(0x729c), 0x59 },
	{ CCI_REG8(0x817c), 0xff },
	{ CCI_REG8(0x817d), 0x80 },
	{ CCI_REG8(0x9348), 0x96 },
	{ CCI_REG8(0x934b), 0x8c },
	{ CCI_REG8(0x934c), 0x82 },
	{ CCI_REG8(0x9353), 0xaa },
	{ CCI_REG8(0x9354), 0xaa },
	{ CCI_REG8(0x4073), 0x30 },
};

static const struct cci_reg_sequence mode_4032x3024_regs[] = {
	{ CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a },
	{ CCI_REG8(0x0114), 0x03 },
	{ CCI_REG8(0x0220), 0x00 },
	{ CCI_REG8(0x0221), 0x11 },
	{ CCI_REG8(0x0340), 0x0c },
	{ CCI_REG8(0x0341), 0x62 },
	{ CCI_REG8(0x0342), 0x22 },
	{ CCI_REG8(0x0343), 0x80 },
	{ CCI_REG8(0x0381), 0x01 },
	{ CCI_REG8(0x0383), 0x01 },
	{ CCI_REG8(0x0385), 0x01 },
	{ CCI_REG8(0x0387), 0x01 },
	{ CCI_REG8(0x0900), 0x00 },
	{ CCI_REG8(0x0901), 0x11 },
	{ CCI_REG8(0x30f4), 0x02 },
	{ CCI_REG8(0x30f5), 0x80 },
	{ CCI_REG8(0x30f6), 0x00 },
	{ CCI_REG8(0x30f7), 0xc8 },
	{ CCI_REG8(0x31a0), 0x00 },
	{ CCI_REG8(0x31a5), 0x00 },
	{ CCI_REG8(0x31a6), 0x00 },
	{ CCI_REG8(0x560f), 0xbe },
	{ CCI_REG8(0x5856), 0x08 },
	{ CCI_REG8(0x58d0), 0x10 },
	{ CCI_REG8(0x734a), 0x01 },
	{ CCI_REG8(0x734f), 0x2b },
	{ CCI_REG8(0x7441), 0x55 },
	{ CCI_REG8(0x7914), 0x03 },
	{ CCI_REG8(0x7928), 0x04 },
	{ CCI_REG8(0x7929), 0x04 },
	{ CCI_REG8(0x793f), 0x03 },
	{ CCI_REG8(0xbc7b), 0x18 },
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x00 },
	{ CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x00 },
	{ CCI_REG8(0x0348), 0x0f },
	{ CCI_REG8(0x0349), 0xbf },
	{ CCI_REG8(0x034a), 0x0b },
	{ CCI_REG8(0x034b), 0xcf },
	{ CCI_REG8(0x034c), 0x0f },
	{ CCI_REG8(0x034d), 0xc0 },
	{ CCI_REG8(0x034e), 0x0b },
	{ CCI_REG8(0x034f), 0xd0 },
	{ CCI_REG8(0x0408), 0x00 },
	{ CCI_REG8(0x0409), 0x00 },
	{ CCI_REG8(0x040a), 0x00 },
	{ CCI_REG8(0x040b), 0x00 },
	{ CCI_REG8(0x040c), 0x0f },
	{ CCI_REG8(0x040d), 0xc0 },
	{ CCI_REG8(0x040e), 0x0b },
	{ CCI_REG8(0x040f), 0xd0 },
	{ CCI_REG8(0x0301), 0x03 },
	{ CCI_REG8(0x0303), 0x02 },
	{ CCI_REG8(0x0305), 0x04 },
	{ CCI_REG8(0x0306), 0x00 },
	{ CCI_REG8(0x0307), 0xd2 },
	{ CCI_REG8(0x0309), 0x0a },
	{ CCI_REG8(0x030b), 0x01 },
	{ CCI_REG8(0x030d), 0x04 },
	{ CCI_REG8(0x030e), 0x00 },
	{ CCI_REG8(0x030f), 0xdf },
	{ CCI_REG8(0x0310), 0x01 },
	{ CCI_REG8(0x0202), 0x0c },
	{ CCI_REG8(0x0203), 0x52 },
	{ CCI_REG8(0x0224), 0x06 },
	{ CCI_REG8(0x0225), 0x29 },
	{ CCI_REG8(0x0204), 0x00 },
	{ CCI_REG8(0x0205), 0x00 },
	{ CCI_REG8(0x0216), 0x00 },
	{ CCI_REG8(0x0217), 0x00 },
	{ CCI_REG8(0x020e), 0x01 },
	{ CCI_REG8(0x020f), 0x00 },
	{ CCI_REG8(0x0226), 0x01 },
	{ CCI_REG8(0x0227), 0x00 },
};

static const struct cci_reg_sequence mode_1920_1080_regs[] = {
	{ CCS_R_BINNING_MODE, 1 },
	{ CCS_R_BINNING_TYPE, 0x42 },
	/* Analogue crop */
	{ CCS_R_X_ADDR_START, 0 },
	{ CCS_R_Y_ADDR_START, 0 },
	{ CCS_R_X_ADDR_END, 4031 },
	{ CCS_R_Y_ADDR_END, 3023 },
	/* Sub-sampling */
	{ CCS_R_X_EVEN_INC, 1 },
	{ CCS_R_X_ODD_INC, 1 },
	{ CCS_R_Y_EVEN_INC, 1 },
	{ CCS_R_Y_ODD_INC, 1 },
	/* Digital crop */
	{ CCS_R_DIGITAL_CROP_X_OFFSET, 0 },
	{ CCS_R_DIGITAL_CROP_Y_OFFSET, 0 },
	{ CCS_R_DIGITAL_CROP_IMAGE_WIDTH, 1920 },
	{ CCS_R_DIGITAL_CROP_IMAGE_HEIGHT, 1080 },
	{ CCS_R_X_OUTPUT_SIZE, 1920 },
	{ CCS_R_Y_OUTPUT_SIZE, 1080 },
	{ CCS_R_FRAME_LENGTH_LINES, IMX363_VTS_30FPS_HD },
};

/*
 * The supported formats.
 * This table MUST contain 4 entries per format, to cover the various flip
 * combinations in the order
 * - no flip
 * - h flip
 * - v flip
 * - h&v flips
 */
static const u32 codes[] = {
	/* 10-bit modes. */
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10
};

static const char * const imx363_test_pattern_menu[] = {
	"Disabled",
	"Solid Colour",
	"Eight Vertical Colour Bars",
	"Colour Bars With Fade to Grey",
	"Pseudorandom Sequence (PN9)",
};

static const struct regulator_bulk_data imx363_supplies[] = {
	{ .supply = "vana" },	/* Analog (2.8V) supply */
	{ .supply = "vdig" },	/* Digital Core (1.2V) supply */
	{ .supply = "vif" },	/* IF (1.8V) supply */
};

enum {
	IMX363_LINK_FREQ_2300MBPS,
	IMX363_LINK_FREQ_640MBPS,
};

/*
 * Pixel rate does not necessarily relate to link frequency on this sensor as
 * there is a FIFO between the pixel array pipeline and the MIPI serializer.
 * The recommendation from Sony is that the pixel array is always run with a
 * line length of 8832 pixels, which means that there is a large amount of
 * blanking time for the 1048x780 mode. There is no need to replicate this
 * blanking on the CSI2 bus, and the configuration of register 0x0301 allows the
 * divider to be altered.
 *
 * The actual factor between link frequency and pixel rate is in the
 * imx363_link_cfg, so use this to convert between the two.
 * bits per pixel being 10, and D-PHY being DDR is assumed by this function, so
 * the value is only the combination of number of lanes and pixel clock divider.
 */
static u64 link_freq_to_pixel_rate(u64 f, const struct imx363_link_cfg *link_cfg)
{
	f *= 2 * link_cfg->lf_to_pix_rate_factor;
	do_div(f, 10);

	return f;
}

static const s64 link_freq_menu_items[] = {
	[IMX363_LINK_FREQ_2300MBPS] = 636000000ULL,
	[IMX363_LINK_FREQ_640MBPS] = 321000000ULL,
};

#define REGS(_list) { .num_of_regs = ARRAY_SIZE(_list), .regs = _list, }

static const struct imx363_link_freq_config link_freq_configs[] = {
	[IMX363_LINK_FREQ_2300MBPS] = {
		.pixels_per_line = IMX363_PPL_DEFAULT,
		.link_cfg = {
			[IMX363_2_LANE_MODE] = {
				.lf_to_pix_rate_factor = 2,
				.reg_list = REGS(mipi_2300mbps_24mhz_2l),
			},
			[IMX363_4_LANE_MODE] = {
				.lf_to_pix_rate_factor = 4,
				.reg_list = REGS(mipi_2300mbps_24mhz_4l),
			},
		}
	},
	[IMX363_LINK_FREQ_640MBPS] = {
		.pixels_per_line = IMX363_PPL_DEFAULT,
		.link_cfg = {
			[IMX363_2_LANE_MODE] = {
				.lf_to_pix_rate_factor = 2 * 2,
				.reg_list = REGS(mipi_642mbps_24mhz_2l),
			},
			[IMX363_4_LANE_MODE] = {
				.lf_to_pix_rate_factor = 4,
				.reg_list = REGS(mipi_642mbps_24mhz_4l),
			},
		}
	},
};

static const struct imx363_mode supported_modes[] = {
	{
		.width = 4032,
		.height = 3024,
		.vts_def = IMX363_VTS_30FPS,
		.vts_min = IMX363_VTS_30FPS,
		.reg_list = REGS(mode_4032x3024_regs),
		.link_freq_index = IMX363_LINK_FREQ_2300MBPS,
		.crop = {
			.left = IMX363_PIXEL_ARRAY_LEFT,
			.top = IMX363_PIXEL_ARRAY_TOP,
			.width = IMX363_PIXEL_ARRAY_WIDTH,
			.height = IMX363_PIXEL_ARRAY_HEIGHT,
		},
	},
	{
		.width = 1920,
		.height = 1080,
		.vts_def = IMX363_VTS_30FPS_HD,
		.vts_min = IMX363_VTS_30FPS_HD,
		.reg_list = REGS(mode_1920_1080_regs),
		.link_freq_index = IMX363_LINK_FREQ_640MBPS,
		.crop = {
			.left = IMX363_PIXEL_ARRAY_LEFT,
			.top = IMX363_PIXEL_ARRAY_TOP,
			.width = IMX363_PIXEL_ARRAY_WIDTH,
			.height = IMX363_PIXEL_ARRAY_HEIGHT,
		},
	},
};

struct imx363 {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct regmap *regmap;

	struct v4l2_ctrl_handler ctrl_handler;
	/* V4L2 Controls */
	struct v4l2_ctrl *link_freq;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;

	unsigned long link_freq_bitmap;
	unsigned int lane_mode_idx;
	unsigned int csi2_flags;

	struct gpio_desc *reset_gpio;
	struct clk *clk;
	struct regulator_bulk_data *supplies;
};

static inline struct imx363 *to_imx363(struct v4l2_subdev *_sd)
{
	return container_of(_sd, struct imx363, sd);
}

static u32 imx363_get_format_code(const struct imx363 *imx363)
{
	unsigned int i;

	i = (imx363->vflip->val ? 2 : 0) | (imx363->hflip->val ? 1 : 0);

	return codes[i];
}

static const struct imx363_mode *
imx363_find_mode(const struct v4l2_mbus_framefmt *format)
{
	return v4l2_find_nearest_size(supported_modes,
				      ARRAY_SIZE(supported_modes), width, height,
				      format->width, format->height);
}

static int imx363_update_digital_gain(struct imx363 *imx363, u32 val)
{
	int ret = 0;

	cci_write(imx363->regmap, CCS_R_DIGITAL_GAIN_GLOBAL, val, &ret);
	cci_write(imx363->regmap, IMX363_REG_DIGITAL_GAIN_GREENB, val, &ret);
	cci_write(imx363->regmap, IMX363_REG_DIGITAL_GAIN_RED, val, &ret);
	cci_write(imx363->regmap, IMX363_REG_DIGITAL_GAIN_BLUE, val, &ret);

	return ret;
}

static void imx363_adjust_exposure_range(struct imx363 *imx363,
					 const struct v4l2_mbus_framefmt *format)
{
	int exposure_max, exposure_def;

	/* Honour the VBLANK limits when setting exposure. */
	exposure_max = format->height + imx363->vblank->val -
		       IMX363_EXPOSURE_OFFSET;
	exposure_def = min(exposure_max, imx363->exposure->val);
	__v4l2_ctrl_modify_range(imx363->exposure, imx363->exposure->minimum,
				 exposure_max, imx363->exposure->step,
				 exposure_def);
}

static int imx363_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct imx363 *imx363 =
		container_of(ctrl->handler, struct imx363, ctrl_handler);
	struct i2c_client *client = v4l2_get_subdevdata(&imx363->sd);
	const struct v4l2_mbus_framefmt *format;
	struct v4l2_subdev_state *state;
	int ret = 0;

	state = v4l2_subdev_get_locked_active_state(&imx363->sd);
	format = v4l2_subdev_state_get_format(state, 0);

	/*
	 * The VBLANK control may change the limits of usable exposure, so check
	 * and adjust if necessary.
	 */
	if (ctrl->id == V4L2_CID_VBLANK)
		imx363_adjust_exposure_range(imx363, format);

	/*
	 * Applying V4L2 control value only happens
	 * when power is up for streaming
	 */
	if (pm_runtime_get_if_in_use(&client->dev) == 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_LINK_FREQ:
		/* Read-only, only ever set from imx363_set_pad_format(). */
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = cci_write(imx363->regmap,
				CCS_R_ANALOG_GAIN_CODE_GLOBAL, ctrl->val,
				NULL);
		break;
	case V4L2_CID_EXPOSURE:
		ret = cci_write(imx363->regmap,
				CCS_R_COARSE_INTEGRATION_TIME, ctrl->val,
				NULL);
		break;
	case V4L2_CID_DIGITAL_GAIN:
		ret = imx363_update_digital_gain(imx363, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = cci_write(imx363->regmap, CCS_R_TEST_PATTERN_MODE,
				ctrl->val, NULL);
		break;
	case V4L2_CID_WIDE_DYNAMIC_RANGE:
		if (!ctrl->val) {
			ret = cci_write(imx363->regmap, CCS_R_HDR_MODE, 0,
					NULL);
		} else {
			ret = cci_write(imx363->regmap, CCS_R_HDR_MODE,
					CCS_HDR_MODE_ENABLED, NULL);
			if (ret)
				break;
			ret = cci_write(imx363->regmap,
					CCS_R_EXPOSURE_RATIO,
					BIT(IMX363_EXPOSURE_RATIO_MAX), NULL);
		}
		break;
	case V4L2_CID_VBLANK:
		ret = cci_write(imx363->regmap, CCS_R_FRAME_LENGTH_LINES,
				format->height + ctrl->val, NULL);
		break;
	case V4L2_CID_VFLIP:
	case V4L2_CID_HFLIP:
		ret = cci_write(imx363->regmap, CCS_R_IMAGE_ORIENTATION,
				(imx363->hflip->val ?
				 CCS_IMAGE_ORIENTATION_HORIZONTAL_MIRROR : 0) |
				(imx363->vflip->val ?
				 CCS_IMAGE_ORIENTATION_VERTICAL_FLIP : 0),
				NULL);
		break;
	default:
		dev_info(&client->dev,
			 "ctrl(id:0x%x,val:0x%x) is not handled\n",
			 ctrl->id, ctrl->val);
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(&client->dev);

	return ret;
}

static const struct v4l2_ctrl_ops imx363_ctrl_ops = {
	.s_ctrl = imx363_set_ctrl,
};

static int imx363_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct imx363 *imx363 = to_imx363(sd);

	/* Only one bayer format (10 bit) is supported */
	if (code->index > 0)
		return -EINVAL;

	code->code = imx363_get_format_code(imx363);

	return 0;
}

static int imx363_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct imx363 *imx363 = to_imx363(sd);

	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != imx363_get_format_code(imx363))
		return -EINVAL;

	fse->min_width = supported_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = supported_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int imx363_set_pad_format(struct v4l2_subdev *sd,
				 const struct v4l2_subdev_client_info *ci,
				 struct v4l2_subdev_state *sd_state,
				 struct v4l2_subdev_format *fmt)
{
	struct imx363 *imx363 = to_imx363(sd);
	const struct imx363_link_cfg *link_cfg;
	struct v4l2_mbus_framefmt *format;
	const struct imx363_mode *mode;
	struct v4l2_rect *crop;
	s64 pixel_rate;
	s64 h_blank;

	mode = imx363_find_mode(&fmt->format);

	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.code = imx363_get_format_code(imx363);
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;
	fmt->format.ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->format.quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->format.xfer_func = V4L2_XFER_FUNC_NONE;

	format = v4l2_subdev_state_get_format(sd_state, 0);
	*format = fmt->format;

	crop = v4l2_subdev_state_get_crop(sd_state, 0);
	*crop = mode->crop;

	if (fmt->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return 0;

	__v4l2_ctrl_s_ctrl(imx363->link_freq, mode->link_freq_index);

	link_cfg = &link_freq_configs[mode->link_freq_index]
		    .link_cfg[imx363->lane_mode_idx];
	pixel_rate = link_freq_to_pixel_rate(link_freq_menu_items[mode->link_freq_index],
					     link_cfg);
	__v4l2_ctrl_modify_range(imx363->pixel_rate, pixel_rate, pixel_rate, 1,
				 pixel_rate);

	/* Update limits and set FPS to default */
	__v4l2_ctrl_modify_range(imx363->vblank, mode->vts_min - mode->height,
				 IMX363_VTS_MAX - mode->height, 1,
				 mode->vts_def - mode->height);
	__v4l2_ctrl_s_ctrl(imx363->vblank, mode->vts_def - mode->height);

	h_blank = link_freq_configs[mode->link_freq_index].pixels_per_line -
		  mode->width;
	__v4l2_ctrl_modify_range(imx363->hblank, h_blank, h_blank, 1, h_blank);

	return 0;
}

static int imx363_get_selection(struct v4l2_subdev *sd,
				const struct v4l2_subdev_client_info *ci,
				struct v4l2_subdev_state *sd_state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		sel->r = *v4l2_subdev_state_get_crop(sd_state, sel->pad);

		return 0;

	case V4L2_SEL_TGT_NATIVE_SIZE:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = IMX363_NATIVE_WIDTH;
		sel->r.height = IMX363_NATIVE_HEIGHT;

		return 0;

	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = IMX363_PIXEL_ARRAY_LEFT;
		sel->r.top = IMX363_PIXEL_ARRAY_TOP;
		sel->r.width = IMX363_PIXEL_ARRAY_WIDTH;
		sel->r.height = IMX363_PIXEL_ARRAY_HEIGHT;

		return 0;
	}

	return -EINVAL;
}

static int imx363_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *sd_state, u32 pad,
				 u64 streams_mask)
{
	struct imx363 *imx363 = to_imx363(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	const struct imx363_reg_list *reg_list;
	const struct imx363_mode *mode;
	int ret;

	mode = imx363_find_mode(v4l2_subdev_state_get_format(sd_state, 0));

	ret = pm_runtime_resume_and_get(&client->dev);
	if (ret < 0)
		return ret;

	ret = cci_write(imx363->regmap, CCS_R_SOFTWARE_RESET,
			CCS_SOFTWARE_RESET_ON, NULL);
	if (ret) {
		dev_err(&client->dev, "failed to reset sensor\n");
		goto err_rpm_put;
	}

	/* 12ms is required from poweron to standby */
	fsleep(12000);

	/* Setup PLL */
	reg_list = &link_freq_configs[mode->link_freq_index]
		    .link_cfg[imx363->lane_mode_idx].reg_list;
	ret = cci_multi_reg_write(imx363->regmap, reg_list->regs,
				  reg_list->num_of_regs, NULL);
	if (ret) {
		dev_err(&client->dev, "failed to set plls\n");
		goto err_rpm_put;
	}

	ret = cci_multi_reg_write(imx363->regmap, mode_common_regs,
				  ARRAY_SIZE(mode_common_regs), NULL);
	if (ret) {
		dev_err(&client->dev, "failed to set common regs\n");
		goto err_rpm_put;
	}

	ret = cci_write(imx363->regmap, IMX363_REG_CLK_BLANK_STOP,
			!!(imx363->csi2_flags & V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK),
			NULL);
	if (ret) {
		dev_err(&client->dev, "failed to set clock lane mode\n");
		goto err_rpm_put;
	}

	/* Apply default values of current mode */
	reg_list = &mode->reg_list;
	ret = cci_multi_reg_write(imx363->regmap, reg_list->regs,
				  reg_list->num_of_regs, NULL);
	if (ret) {
		dev_err(&client->dev, "failed to set mode\n");
		goto err_rpm_put;
	}

	/* Apply customized values from user */
	ret = __v4l2_ctrl_handler_setup(imx363->sd.ctrl_handler);
	if (ret)
		goto err_rpm_put;

	ret = cci_write(imx363->regmap, CCS_R_MODE_SELECT,
			CCS_MODE_SELECT_STREAMING, NULL);
	if (ret)
		goto err_rpm_put;

	/* The Bayer order changes with the flips, so they can't change now */
	__v4l2_ctrl_grab(imx363->vflip, true);
	__v4l2_ctrl_grab(imx363->hflip, true);

	return 0;

err_rpm_put:
	pm_runtime_put(&client->dev);

	return ret;
}

static int imx363_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state, u32 pad,
				  u64 streams_mask)
{
	struct imx363 *imx363 = to_imx363(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	ret = cci_write(imx363->regmap, CCS_R_MODE_SELECT,
			CCS_MODE_SELECT_SOFTWARE_STANDBY, NULL);
	if (ret)
		dev_err(&client->dev, "failed to set stream\n");

	__v4l2_ctrl_grab(imx363->vflip, false);
	__v4l2_ctrl_grab(imx363->hflip, false);

	pm_runtime_put(&client->dev);

	return ret;
}

static int imx363_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *sd_state)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.width = supported_modes[0].width,
			.height = supported_modes[0].height,
		},
	};

	return imx363_set_pad_format(sd, NULL, sd_state, &fmt);
}

static int imx363_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx363 *imx363 = to_imx363(sd);
	int ret;

	ret = regulator_bulk_enable(ARRAY_SIZE(imx363_supplies),
				    imx363->supplies);
	if (ret) {
		dev_err(dev, "failed to enable regulators\n");
		return ret;
	}

	usleep_range(400, 600);

	gpiod_set_value_cansleep(imx363->reset_gpio, 0);

	ret = clk_prepare_enable(imx363->clk);
	if (ret) {
		dev_err(dev, "failed to enable clock\n");
		gpiod_set_value_cansleep(imx363->reset_gpio, 1);
		regulator_bulk_disable(ARRAY_SIZE(imx363_supplies),
				       imx363->supplies);
		return ret;
	}

	/* values unknown, but 1000, 1200 is not enough, FIXME */
	usleep_range(IMX519_XCLR_MIN_DELAY_US,
		     IMX519_XCLR_MIN_DELAY_US + IMX519_XCLR_DELAY_RANGE_US);

	return 0;
}

static int imx363_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct imx363 *imx363 = to_imx363(sd);

	clk_disable_unprepare(imx363->clk);

	gpiod_set_value_cansleep(imx363->reset_gpio, 1);

	regulator_bulk_disable(ARRAY_SIZE(imx363_supplies), imx363->supplies);

	return 0;
}

static int imx363_identify_module(struct imx363 *imx363)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx363->sd);
	int ret;
	u64 val;

	ret = cci_read(imx363->regmap, CCS_R_SENSOR_MODEL_ID, &val, NULL);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "failed to read sensor model id\n");

	if (val != IMX363_SENSOR_MODEL_ID)
		return dev_err_probe(&client->dev, -EIO,
				     "sensor model id mismatch: %x != %llx\n",
				     IMX363_SENSOR_MODEL_ID, val);

	return 0;
}

static const struct v4l2_subdev_video_ops imx363_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops imx363_pad_ops = {
	.enum_mbus_code = imx363_enum_mbus_code,
	.enum_frame_size = imx363_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = imx363_set_pad_format,
	.get_selection = imx363_get_selection,
	.enable_streams = imx363_enable_streams,
	.disable_streams = imx363_disable_streams,
};

static const struct v4l2_subdev_ops imx363_subdev_ops = {
	.video = &imx363_video_ops,
	.pad = &imx363_pad_ops,
};

static const struct v4l2_subdev_internal_ops imx363_internal_ops = {
	.init_state = imx363_init_state,
};

static const struct media_entity_operations imx363_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
};

static int imx363_init_controls(struct imx363 *imx363)
{
	struct i2c_client *client = v4l2_get_subdevdata(&imx363->sd);
	const struct imx363_mode *mode = &supported_modes[0];
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl_handler *ctrl_hdlr;
	const struct imx363_link_cfg *link_cfg;
	s64 pixel_rate;
	int ret;

	ctrl_hdlr = &imx363->ctrl_handler;
	ret = v4l2_ctrl_handler_init(ctrl_hdlr, 13);
	if (ret)
		return ret;

	imx363->link_freq = v4l2_ctrl_new_int_menu(ctrl_hdlr, &imx363_ctrl_ops,
						   V4L2_CID_LINK_FREQ,
						   ARRAY_SIZE(link_freq_menu_items) - 1,
						   0, link_freq_menu_items);
	if (imx363->link_freq)
		imx363->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx363->hflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx363_ctrl_ops,
					  V4L2_CID_HFLIP, 0, 1, 1, 1);
	if (imx363->hflip)
		imx363->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	imx363->vflip = v4l2_ctrl_new_std(ctrl_hdlr, &imx363_ctrl_ops,
					  V4L2_CID_VFLIP, 0, 1, 1, 1);
	if (imx363->vflip)
		imx363->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	link_cfg = &link_freq_configs[mode->link_freq_index]
		    .link_cfg[imx363->lane_mode_idx];
	pixel_rate = link_freq_to_pixel_rate(link_freq_menu_items[mode->link_freq_index],
					     link_cfg);

	/* By default, PIXEL_RATE is read only */
	imx363->pixel_rate = v4l2_ctrl_new_std(ctrl_hdlr, &imx363_ctrl_ops,
					       V4L2_CID_PIXEL_RATE, pixel_rate,
					       pixel_rate, 1, pixel_rate);

	imx363->vblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx363_ctrl_ops,
					   V4L2_CID_VBLANK,
					   mode->vts_min - mode->height,
					   IMX363_VTS_MAX - mode->height, 1,
					   mode->vts_def - mode->height);

	imx363->hblank = v4l2_ctrl_new_std(ctrl_hdlr, &imx363_ctrl_ops,
					   V4L2_CID_HBLANK,
					   IMX363_PPL_DEFAULT - mode->width,
					   IMX363_PPL_DEFAULT - mode->width, 1,
					   IMX363_PPL_DEFAULT - mode->width);
	if (imx363->hblank)
		imx363->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	imx363->exposure = v4l2_ctrl_new_std(ctrl_hdlr, &imx363_ctrl_ops,
					     V4L2_CID_EXPOSURE,
					     IMX363_EXPOSURE_MIN,
					     IMX363_EXPOSURE_MAX,
					     IMX363_EXPOSURE_STEP,
					     IMX363_EXPOSURE_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx363_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  IMX363_ANA_GAIN_MIN, IMX363_ANA_GAIN_MAX,
			  IMX363_ANA_GAIN_STEP, IMX363_ANA_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx363_ctrl_ops, V4L2_CID_DIGITAL_GAIN,
			  IMX363_DGTL_GAIN_MIN, IMX363_DGTL_GAIN_MAX,
			  IMX363_DGTL_GAIN_STEP, IMX363_DGTL_GAIN_DEFAULT);

	v4l2_ctrl_new_std(ctrl_hdlr, &imx363_ctrl_ops,
			  V4L2_CID_WIDE_DYNAMIC_RANGE, 0, 1, 1, 0);

	v4l2_ctrl_new_std_menu_items(ctrl_hdlr, &imx363_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(imx363_test_pattern_menu) - 1,
				     0, 0, imx363_test_pattern_menu);

	if (ctrl_hdlr->error) {
		ret = ctrl_hdlr->error;
		dev_err(&client->dev, "control init failed (%d)\n", ret);
		goto error;
	}

	ret = v4l2_fwnode_device_parse(&client->dev, &props);
	if (ret)
		goto error;

	ret = v4l2_ctrl_new_fwnode_properties(ctrl_hdlr, &imx363_ctrl_ops,
					      &props);
	if (ret)
		goto error;

	imx363->sd.ctrl_handler = ctrl_hdlr;

	return 0;

error:
	v4l2_ctrl_handler_free(ctrl_hdlr);

	return ret;
}

static int imx363_get_hwcfg(struct imx363 *imx363, struct device *dev)
{
	struct v4l2_fwnode_endpoint ep = {
		.bus_type = V4L2_MBUS_CSI2_DPHY
	};
	struct fwnode_handle *endpoint;
	int ret;

	endpoint = fwnode_graph_get_next_endpoint(dev_fwnode(dev), NULL);
	if (!endpoint)
		return dev_err_probe(dev, -EINVAL, "endpoint node not found\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(endpoint, &ep);
	fwnode_handle_put(endpoint);
	if (ret)
		return dev_err_probe(dev, ret, "parsing endpoint failed\n");

	ret = v4l2_link_freq_to_bitmap(dev, ep.link_frequencies,
				       ep.nr_of_link_frequencies,
				       link_freq_menu_items,
				       ARRAY_SIZE(link_freq_menu_items),
				       &imx363->link_freq_bitmap);
	if (ret) {
		dev_err_probe(dev, ret, "link frequency not supported\n");
		goto out;
	}

	switch (ep.bus.mipi_csi2.num_data_lanes) {
	case 2:
		imx363->lane_mode_idx = IMX363_2_LANE_MODE;
		break;
	case 4:
		imx363->lane_mode_idx = IMX363_4_LANE_MODE;
		break;
	default:
		ret = dev_err_probe(dev, -EINVAL, "invalid data lanes: %u\n",
				    ep.bus.mipi_csi2.num_data_lanes);
		goto out;
	}

	imx363->csi2_flags = ep.bus.mipi_csi2.flags;

out:
	v4l2_fwnode_endpoint_free(&ep);

	return ret;
}

static int imx363_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct imx363 *imx363;
	unsigned long rate;
	int ret;

	imx363 = devm_kzalloc(dev, sizeof(*imx363), GFP_KERNEL);
	if (!imx363)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&imx363->sd, client, &imx363_subdev_ops);
	imx363->sd.internal_ops = &imx363_internal_ops;

	ret = imx363_get_hwcfg(imx363, dev);
	if (ret)
		return ret;

	imx363->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(imx363->regmap))
		return dev_err_probe(dev, PTR_ERR(imx363->regmap),
				     "failed to initialize CCI\n");

	ret = devm_regulator_bulk_get_const(dev, ARRAY_SIZE(imx363_supplies),
					    imx363_supplies, &imx363->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	imx363->clk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(imx363->clk))
		return dev_err_probe(dev, PTR_ERR(imx363->clk),
				     "failed to get clock\n");

	rate = clk_get_rate(imx363->clk);
	if (rate != IMX363_EXTCLK_FREQ)
		return dev_err_probe(dev, -EINVAL,
				     "external clock frequency %lu not supported\n",
				     rate);

	/* Request optional reset pin */
	imx363->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_HIGH);
	if (IS_ERR(imx363->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(imx363->reset_gpio),
				     "failed to get reset gpio\n");

	/* Will be powered off via pm_runtime_idle */
	ret = imx363_power_on(dev);
	if (ret)
		return ret;

	ret = imx363_identify_module(imx363);
	if (ret)
		goto error_power_off;

	ret = imx363_init_controls(imx363);
	if (ret)
		goto error_power_off;

	imx363->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	imx363->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	imx363->sd.entity.ops = &imx363_entity_ops;
	imx363->pad.flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&imx363->sd.entity, 1, &imx363->pad);
	if (ret) {
		dev_err_probe(dev, ret, "failed to init entity pads\n");
		goto error_handler_free;
	}

	imx363->sd.state_lock = imx363->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&imx363->sd);
	if (ret) {
		dev_err_probe(dev, ret, "subdev init error\n");
		goto error_media_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&imx363->sd);
	if (ret) {
		dev_err_probe(dev, ret, "failed to register sensor sub-device\n");
		goto error_subdev_cleanup;
	}

	pm_runtime_idle(dev);

	return 0;

error_subdev_cleanup:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	v4l2_subdev_cleanup(&imx363->sd);

error_media_entity:
	media_entity_cleanup(&imx363->sd.entity);

error_handler_free:
	v4l2_ctrl_handler_free(imx363->sd.ctrl_handler);

error_power_off:
	imx363_power_off(dev);

	return ret;
}

static void imx363_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(sd->ctrl_handler);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		imx363_power_off(&client->dev);
	pm_runtime_set_suspended(&client->dev);
}

static const struct dev_pm_ops imx363_pm_ops = {
	SET_RUNTIME_PM_OPS(imx363_power_off, imx363_power_on, NULL)
};

static const struct acpi_device_id imx363_acpi_ids[] = {
	{ "SONY363A" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(acpi, imx363_acpi_ids);

static const struct of_device_id imx363_dt_ids[] = {
	{ .compatible = "sony,imx363" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx363_dt_ids);

static struct i2c_driver imx363_i2c_driver = {
	.driver = {
		.name = "imx363",
		.pm = &imx363_pm_ops,
		.acpi_match_table = imx363_acpi_ids,
		.of_match_table	= imx363_dt_ids,
	},
	.probe = imx363_probe,
	.remove = imx363_remove,
};

module_i2c_driver(imx363_i2c_driver);

MODULE_AUTHOR("Yeh, Andy <andy.yeh@intel.com>");
MODULE_AUTHOR("Chiang, Alan");
MODULE_AUTHOR("Chen, Jason");
MODULE_DESCRIPTION("Sony IMX363 sensor driver");
MODULE_LICENSE("GPL");
