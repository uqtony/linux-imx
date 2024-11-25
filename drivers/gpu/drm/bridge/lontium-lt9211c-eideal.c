// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices EP172U MIPI/LVDS transmitter driver
 *
 * Copyright 2023 EIDEAL
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>

#include <video/videomode.h>

#define LT9211C_RXPLL_PIX_CLK 0x00
#define LT9211C_DESSCPLL_PIX_CLK 0x01
#define LT9211C_RXPLL_DEC_DDR_CLK 0x02
#define LT9211C_MLRX_BYTE_CLK 0x03

#define LT9211C_AD_MLTX_READ_CLK 0x08
#define LT9211C_AD_MLTX_WRITE_CLK 0x09
#define LT9211C_AD_DESSCPLL_PIX_CLK 0x10
#define LT9211C_AD_RXPLL_PIX_CLK 0x1a
#define LT9211C_AD_DESSCPLL_PCR_CLK 0x14
#define LT9211C_AD_MLRXA_BYTE_CLK 0x18
#define LT9211C_AD_MLRXB_BYTE_CLK 0x1e

enum lt9211c_model {
        MODEL_LONTIUM_LT9211C,
};

enum lt9211c_state {
	LT9211C_STATE_PREPARE,
	LT9211C_STATE_CHIPRX_VIDTIMING_CONFIG,
	LT9211C_STATE_CHIPRX_PLL_CONFIG,
	LT9211C_STATE_CHIPTX_CONFIG_VIDEO,
	LT9211C_STATE_CHIPTX_VIDEO_OUT,
};

struct lt9211c_mipi_rx_video_timing {
        u16 wc;
        u16 hact;
        u16 vact;
        u8 fmt;
        u8 pa_lpn;
        u8 frame_rate;
};

struct lt9211c_video_timing {
	u16 hfront_porch;
	u16 hsync_len;
	u16 hback_porch;
	u16 hactive;
	u16 htotal;

	u16 vfront_porch;
	u16 vsync_len;
	u16 vback_porch;
	u16 vactive;
	u16 vtotal;

	u8  framerate;
	u32 pclk_khz;
};

struct lt9211c_pcr_setting {
	u32 pcr_m;
	u32 pcr_k;
	u32 pcr_up_limit;
	u32 pcr_down_limit;
};

struct lt9211c {
	struct drm_bridge	bridge;
	struct drm_connector 	connector;
	struct device		*dev;
	struct regmap		*regmap;
	struct device_node	*host_node;
	struct mipi_dsi_device	*dsi;
	struct drm_bridge	*panel_bridge;
	struct gpio_desc	*rst_gpio;
	struct clk		*dsi_mclk;
	struct videomode 	videomode;
	struct delayed_work	delayed_work;
	u32			state;
	struct lt9211c_mipi_rx_video_timing mipi_rx_video_timing;
	struct lt9211c_video_timing lt9211c_video_timing;
	u32 			width_mm;
        u32 			height_mm;
};

static const struct regmap_config lt9211c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xffff,
};

static const struct display_timing lt9211c_default_timing = {
         .pixelclock = { 148500000, 148500000, 148500000 },
         .hactive = { 1920, 1920, 1920 },
         .hfront_porch = { 88, 88, 88 },
         .hback_porch = { 148, 148, 148 },
         .hsync_len = { 44, 44, 44 },
         .vactive = { 1080, 1080, 1080 },
         .vfront_porch = { 4, 4, 4 },
         .vback_porch = { 36, 36, 36 },
         .vsync_len = { 5, 5, 5 },
};

static const struct lt9211c_video_timing lt9211c_support_timing[] = {
 {24,    96,    40,     640,     800,     33,   2,   10,   480,    525,   60},  //video_640x480_60Hz
 {16,    62,    60,     720,     858,     9,    6,   30,   480,    525,   60},  //video_720x480_60Hz
 {12,    64,    88,     720,     864,     5,    5,   39,   576,    625,   50},  //video_720x576_50Hz
 {48,    128,   88,     800,     1056,    1,    4,   23,   600,    628,   60},  //video_800x600_60Hz
 {110,   40,    220,    1280,    1650,    5,    5,   20,   720,    750,   30},  //video_1280x720_30Hz
 {440,   40,    220,    1280,    1980,    5,    5,   20,   720,    750,   50},  //video_1280x720_50Hz
 {110,   40,    220,    1280,    1650,    5,    5,   20,   720,    750,   60},  //video_1280x720_60Hz
 {24,    136,   160,    1024,    1344,    3,    6,   29,   768,    806,   60},  //video_1024x768_60Hz
 {26,    110,   110,    1366,    1592,    13,   6,   13,   768,    800,   60},  //video_1366x768_60Hz
 {110,   40,    220,    1280,    1650,    5,    5,   20,   720,    750,   30},  //video_1280x720_30Hz
 {48,    32,    80,     1920,    2080,    5,    5,   20,   720,    750,   60},  //video_1920x720_60Hz
 {48,    112,   248,    1280,    1688,    1,    3,   38,   1024,   1066,  60},  //video_1028x1024_60Hz
 {88,    44,    148,    1920,    2200,    4,    5,   36,   1080,   1125,  30},  //video_1920x1080_30Hz
 {88,    44,    148,    1920,    2200,    4,    5,   36,   1080,   1125,  60},  //video_1920x1080_60Hz
 {88,    44,    148,    1920,    2200,    4,    5,   36,   1080,   1125,  90},  //video_1920x1080_90Hz
// {90,    44,    148,    1920,    2202,    14,    5,   36,   1080,   1135,  60},
 {64,    192,   304,    1600,    2160,    1,    3,   46,   1200,   1250,  60},  //video_1600x1200_60Hz
 {48,    32,    80,     1920,    2080,    3,    6,   26,   1200,   1235,  60},  //video_1920x1200_60Hz
 {32,    48,    80,     2048,    2208,    6,    3,   28,   1280,   1317,  60},  //video_2048x1280_60Hz
 {50,    48,    80,     2304,    2482,    6,    3,   32,   1440,   1481,  60},  //video_2304x1440_60Hz
 {48,    32,    80,     2560,    2720,    3,    5,   33,   1440,   1481,  60},  //video_2560x1440_60Hz
 {1276,  88,    296,    3840,    5500,    8,    10,  72,   2160,   2250,  24},  //video_3840x2160_24Hz
};

static u32 g_width_mm = 698;
static u32 g_height_mm = 393;

static struct lt9211c *bridge_to_lt9211c(struct drm_bridge *bridge)
{
        return container_of(bridge, struct lt9211c, bridge);
}

static struct lt9211c *connector_to_lt9211c(struct drm_connector *connector)
{
        return container_of(connector, struct lt9211c, connector);
}

static int lt9211c_read_chipid(struct lt9211c *ctx)
{
	u8 chipid_data[3];
	int ret = 0;
	int index = 0;

	ret = regmap_write(ctx->regmap, 0xff, 0x81);
	ret = regmap_bulk_read(ctx->regmap, 0x00, chipid_data, 3);

	if (ret < 0)
	{
		printk("%s: read chip id failed, ret=%d\n", __func__, ret);
		return ret;
	}
	while(index < ARRAY_SIZE(chipid_data))
	{
		printk("%s: read chip id request, req[0x%x]=0x%x\n", __func__, index, chipid_data[index]);
		index++;
	}
	return ret;
}

static int lt9211c_mipi_rx_phy_poweron(struct lt9211c *ctx)
{
	int ret = 0;
	struct reg_sequence pre_reg_cfg[] = {
		{0x00, 0x00},
		{0xff, 0x82},
		{0x01, 0x11}, //MIPI RX portA & B disable
	};

	// Select Port A
        struct reg_sequence select_porta_cfg[] = {
                {0x18,0x48}, //portA clk delay select 0
                {0x01,0x91}, //MIPI RX portA enable
                {0x02,0x00}, //[5][1]:0 mipi mode, no swap
                {0x03,0xee}, //port A & B eq current reference
                {0x09,0x21}, //[3]0: select link clk from port-A, [1]0: mlrx_clk2pll disable
                {0x04,0x44},
                {0x05,0xc4}, //port A clk lane eq sel
                {0x06,0x44},
                {0x13,0x0c}, //MIPI port A clk lane rterm & high speed en
        };

	struct reg_sequence suffix_reg_cfg[] = {
                {0xff,0xd0},
                {0x01,0x00}, //mipi rx data lane term enable time: 39ns;
                {0x02,0x0e}, //mipi rx hs settle time defult set: 0x05;
                {0x05,0x00}, //mipi rx lk lane term enable time: 39ns;
                {0x0a,0x59},
                {0x0b,0x20},
                {0xff,0x81},
                {0x09,0xde}, //mipi rx dphy reset
                {0x09,0xdf}, //mipi rx dphy release
        };

	u32 data;
	ret = regmap_write(ctx->regmap, 0xff, 0x0d0);
	ret = regmap_bulk_read(ctx->regmap, 0x00, &data, 1);
	pre_reg_cfg[0].def = data;

	if (ctx->dsi->lanes != 4)
		pre_reg_cfg[0].def |= ctx->dsi->lanes;

	ret = regmap_multi_reg_write(ctx->regmap, pre_reg_cfg, ARRAY_SIZE(pre_reg_cfg));

	if (ret < 0) {
                dev_err(ctx->dev, "#%s# failed to config pre-reg, ret==%d\n", __func__, ret);
                return -EPROBE_DEFER;
        }
	ret = regmap_multi_reg_write(ctx->regmap, select_porta_cfg, ARRAY_SIZE(select_porta_cfg));

	if (ctx->dsi->mode_flags & MIPI_DSI_MODE_VIDEO_BURST)
	{
		ret = regmap_write(ctx->regmap, 0x13, 0x00);
	}
	if (ret < 0) {
                dev_err(ctx->dev, "#%s# failed to config select portA, ret==%d\n", __func__, ret);
                return -EPROBE_DEFER;
        }
	ret = regmap_multi_reg_write(ctx->regmap, suffix_reg_cfg, ARRAY_SIZE(suffix_reg_cfg));
	if (ret < 0) {
                dev_err(ctx->dev, "#%s# failed to config suffix-reg, ret==%d\n", __func__, ret);
                return -EPROBE_DEFER;
        }
	return ret;
}

static int lt9211c_mipi_rx_clk_sel(struct lt9211c *ctx)
{
	int ret = 0;
        struct reg_sequence reg_cfg[] = {
		{0xff,0x85},
		{0xe9,0x88}, //sys clk sel from XTAL

		{0xff,0x81},
		{0x80,0x51}, //[7:6]rx sram rd clk src sel ad dessc pcr clk
                                   //[5:4]rx sram wr clk src sel mlrx bytr clk
                                   //[1:0]video check clk sel from desscpll pix clk
                // MIPIRX_PORT_SEL == PORTA
		{0x81,0x10}, //[5]0: mlrx byte clock select from ad_mlrxa_byte_clk
                                   //[4]1: rx output pixel clock select from ad_desscpll_pix_clk
                //  end MIPIRX_PORT_SEL == PORTA
		{0xff,0x86},
		{0x32,0x03}, //video check frame cnt set: 3 frame
	};
	ret = regmap_multi_reg_write(ctx->regmap, reg_cfg, ARRAY_SIZE(reg_cfg));
	if (ret < 0) {
                dev_err(ctx->dev, "#%s# failed to config clk sel, ret==%d\n", __func__, ret);
                return -EPROBE_DEFER;
        }
	return ret;
}

static int lt9211c_mipi_rx_input_sel(struct lt9211c *ctx)
{
	int ret = 0;
	struct reg_sequence reg_cfg[] = {
		{0xff,0xd0},
		// (MIPIRX_INPUT_SEL == MIPI_DSI)
		{0x04,0x00}, //[4]0: DSI enable
		{0x21,0x46}, //[7](dsi: hsync_level(for pcr adj) = hsync_level; csi:hsync_level(for pcr adj) = de_level)
		// end  (MIPIRX_INPUT_SEL == MIPI_DSI)
	};

	ret = regmap_multi_reg_write(ctx->regmap, reg_cfg, ARRAY_SIZE(reg_cfg));
        if (ret < 0) {
                dev_err(ctx->dev, "#%s# failed to config, ret==%d\n", __func__, ret);
                return -EPROBE_DEFER;
        }
	return ret;
}

static int lt9211c_mipi_rx_lane_set(struct lt9211c *ctx)
{
        int ret = 0;
	struct reg_sequence reg_cfg[] = {
		{0xff,0x85},
		{0x3f,0x08}, //MLRX HS/LP control conmand enable
		{0x40,0x04}, //[2:0]pa_ch0_src_sel ch4 data
		{0x41,0x03}, //[2:0]pa_ch1_src_sel ch3 data
		{0x42,0x02}, //[2:0]pa_ch2_src_sel ch2 data
		{0x43,0x01}, //[2:0]pa_ch3_src_sel ch1 data

		{0x45,0x04}, //[2:0]pb_ch0_src_sel ch9 data
		{0x46,0x03}, //[2:0]pb_ch1_src_sel ch8 data
		{0x47,0x02}, //[2:0]pb_ch2_src_sel ch7 data
		{0x48,0x01}, //[2:0]pb_ch3_src_sel ch6 data

		{0x44,0x00}, //[6]mlrx port A output select port A;[2:0]pa_ch4_src_sel ch0 data
		{0x49,0x00}, //[6]mlrx port B output select port A;[2:0]pb_ch4_src_sel ch5 data
	};
	ret = regmap_multi_reg_write(ctx->regmap, reg_cfg, ARRAY_SIZE(reg_cfg));
        if (ret < 0) {
                dev_err(ctx->dev, "#%s# failed to config, ret==%d\n", __func__, ret);
                return -EPROBE_DEFER;
        }
        return ret;
}

static u8 lt9211c_video_check_framerate_get(struct lt9211c *ctx)
{
	u8 framerate = 0;
	u32 frametime = 0;
	u32 data;
	int ret = 0;

	regmap_write(ctx->regmap, 0xff, 0x86);
	ret = regmap_bulk_read(ctx->regmap, 0x43, &frametime, 1);
	ret = regmap_bulk_read(ctx->regmap, 0x44, &data, 1);
	frametime = (frametime << 8) + data;
	ret = regmap_bulk_read(ctx->regmap, 0x45, &data, 1);
	frametime = (frametime << 8) + data;
	frametime = (25000000 * 2 / frametime + 1) / 2;
	framerate = (u8)frametime;
	return framerate;
}

static int lt9211c_mipi_rx_sot_get(struct lt9211c *ctx)
{
	int ret = 0;
	u8 lane_sot_num[4];
	u8 lane_sot_data[4];

	ret = regmap_write(ctx->regmap, 0xff, 0xd0);

	ret = regmap_bulk_read(ctx->regmap, 0x88, &lane_sot_num[0], 1);
	ret = regmap_bulk_read(ctx->regmap, 0x89, &lane_sot_data[0], 1);

	ret = regmap_bulk_read(ctx->regmap, 0x8a, &lane_sot_num[1], 1);
        ret = regmap_bulk_read(ctx->regmap, 0x8b, &lane_sot_data[1], 1);

	ret = regmap_bulk_read(ctx->regmap, 0x8c, &lane_sot_num[2], 1);
        ret = regmap_bulk_read(ctx->regmap, 0x8d, &lane_sot_data[2], 1);

	ret = regmap_bulk_read(ctx->regmap, 0x8e, &lane_sot_num[3], 1);
        ret = regmap_bulk_read(ctx->regmap, 0x8f, &lane_sot_data[3], 1);

	return ret;
}

static int lt9211c_mipi_rx_hs_settle_set(struct lt9211c *ctx)
{
        int ret = 0;
        struct reg_sequence reg_cfg[] = {
		{0xff,0xd0},
		//{0x02,0x05},
		{0x02,0x0a},
	};

	ret = regmap_multi_reg_write(ctx->regmap, reg_cfg, ARRAY_SIZE(reg_cfg));
        if (ret < 0) {
                dev_err(ctx->dev, "#%s# failed to config, ret==%d\n", __func__, ret);
                return -EPROBE_DEFER;
        }
	return ret;
}

static int lt9211c_mipi_rx_hact_get(struct lt9211c *ctx)
{
	int ret = 0;
	u8 data[5];

	ret = regmap_write(ctx->regmap, 0xff, 0xd0);
	ret = regmap_bulk_read(ctx->regmap, 0x82, data, 5);
	ctx->mipi_rx_video_timing.vact = (data[3] << 8)+ data[4];

	ctx->mipi_rx_video_timing.fmt = data[2] & 0x0f;

	ret = regmap_bulk_read(ctx->regmap, 0x9c, &ctx->mipi_rx_video_timing.pa_lpn, 1);

        ctx->mipi_rx_video_timing.wc = (data[0] << 8) + data[1];
	if (ctx->mipi_rx_video_timing.fmt == 0x0a)
		printk("#%s# [0x%x 0x%x 0x%x 0x%x]\n", __func__, data[0], data[1], data[3], data[4]);

	 switch (ctx->mipi_rx_video_timing.fmt)
	{
		case 0x01: //DSI-YUV422-10bpc
		case 0x0e: //CSI-YUV422-10bpc
			ctx->mipi_rx_video_timing.hact = ctx->mipi_rx_video_timing.wc * 5 / 2; //wc = hact * 20bpp/8
		break;
		case 0x02: //DSI-YUV422-12bpc
			ctx->mipi_rx_video_timing.hact = ctx->mipi_rx_video_timing.wc / 3; //wc = hact * 24bpp/8
		break;
		case 0x03: //YUV422-8bpc
			ctx->mipi_rx_video_timing.hact = ctx->mipi_rx_video_timing.wc / 2; //wc = hact * 16bpp/8
		break;
		case 0x04: //RGB10bpc
			ctx->mipi_rx_video_timing.hact = ctx->mipi_rx_video_timing.wc * 15 / 4; //wc = hact * 30bpp/8
		break;
		case 0x05: //RGB12bpc
			ctx->mipi_rx_video_timing.hact = ctx->mipi_rx_video_timing.wc * 9 / 2; //wc = hact * 36bpp/8
		break;
		case 0x06: //YUV420-8bpc
			ctx->mipi_rx_video_timing.hact = ctx->mipi_rx_video_timing.wc / 3; //wc = hact * 24bpp/8
		break;
		case 0x0a: //RGB8bpc
			ctx->mipi_rx_video_timing.hact = ctx->mipi_rx_video_timing.wc / 3; //wc = hact * 24bpp/8
		break;
		case 0x07: //RGB565
			ctx->mipi_rx_video_timing.hact = ctx->mipi_rx_video_timing.wc / 2; //wc = hact * 16bpp/8
		break;
		case 0x08: //RGB6bpc
		case 0x09: //RGB6bpc_losely
			ctx->mipi_rx_video_timing.hact = ctx->mipi_rx_video_timing.wc * 9 / 4; //wc = hact * 18bpp/8
		break;
		case 0x0b: //RAW8
			ctx->mipi_rx_video_timing.hact = ctx->mipi_rx_video_timing.wc / 1; //wc = hact * 8bpp/8
		break;
		case 0x0c: //RAW10
			ctx->mipi_rx_video_timing.hact = ctx->mipi_rx_video_timing.wc * 5 / 4 ; //wc = hact * 10bpp/8
		break;
		case 0x0d: //RAW12
			ctx->mipi_rx_video_timing.hact = ctx->mipi_rx_video_timing.wc * 3 / 2; //wc = hact * 12bpp/8
		break;
		default:
			ctx->mipi_rx_video_timing.hact = ctx->mipi_rx_video_timing.wc / 3; //wc = hact * 24bpp/8
		break;
	}
	return ret;
}// end of lt9211c_mipi_rx_hact_get

static int lt9211c_mipi_rx_video_timing_get(struct lt9211c *ctx)
{
	int ret = 0;

	lt9211c_mipi_rx_sot_get(ctx);
	lt9211c_mipi_rx_hs_settle_set(ctx);
	lt9211c_mipi_rx_hact_get(ctx);
	if ((ctx->mipi_rx_video_timing.hact < 400) ||
			(ctx->mipi_rx_video_timing.vact < 400))
	{
                dev_err(ctx->dev, "#%s# no video get\n", __func__);
                return -EPROBE_DEFER;
	}
	printk("#%s# hact=%d\n", __func__, ctx->mipi_rx_video_timing.hact);
	printk("#%s# vact=%d\n", __func__, ctx->mipi_rx_video_timing.vact);
	printk("#%s# fmt=0x%02x\n", __func__, ctx->mipi_rx_video_timing.fmt);
	printk("#%s# pa_lpn=0x%02x\n", __func__, ctx->mipi_rx_video_timing.pa_lpn);

	return ret;
}

static int lt9211c_mipi_rx_video_timing_set(struct lt9211c *ctx, struct lt9211c_video_timing video_timing)
{
	int ret = 0;

	ret = regmap_write(ctx->regmap, 0xff, 0xd0);
	ret = regmap_write(ctx->regmap, 0x0d, video_timing.vtotal >> 8);
	ret = regmap_write(ctx->regmap, 0x0e, video_timing.vtotal);
	ret = regmap_write(ctx->regmap, 0x0f, video_timing.vactive >> 8);
	ret = regmap_write(ctx->regmap, 0x10, video_timing.vactive);
	ret = regmap_write(ctx->regmap, 0x15, video_timing.vsync_len);
	ret = regmap_write(ctx->regmap, 0x17, video_timing.vfront_porch >> 8);
	ret = regmap_write(ctx->regmap, 0x18, video_timing.vfront_porch);

	ret = regmap_write(ctx->regmap, 0x11, video_timing.htotal >> 8);
        ret = regmap_write(ctx->regmap, 0x12, video_timing.htotal);
        ret = regmap_write(ctx->regmap, 0x13, video_timing.hactive >> 8);
        ret = regmap_write(ctx->regmap, 0x14, video_timing.hactive);
        ret = regmap_write(ctx->regmap, 0x4c, video_timing.hsync_len);
        ret = regmap_write(ctx->regmap, 0x19, video_timing.hfront_porch >> 8);
        ret = regmap_write(ctx->regmap, 0x1a, video_timing.hfront_porch);

	return ret;

}

static int lt9211c_mipi_rx_video_timing_sel(struct lt9211c *ctx)
{
	int ret = -1;
	int index = 0;
	int support_timing_array_size = 0;
	u8 current_framerate = 0;

	support_timing_array_size = sizeof(lt9211c_support_timing) / sizeof(lt9211c_support_timing[0]);

	current_framerate = lt9211c_video_check_framerate_get(ctx);
	dev_info(ctx->dev, "%s# current frame rate=%d\n", __func__, current_framerate);

	while (index < support_timing_array_size)
	{
		if ( (ctx->mipi_rx_video_timing.hact != lt9211c_support_timing[index].hactive) ||
			(ctx->mipi_rx_video_timing.vact != lt9211c_support_timing[index].vactive))
		{
			// resolution not match
			index++;
			continue;
		}
		if ( (current_framerate < (lt9211c_support_timing[index].framerate - 3)) ||
			(current_framerate > (lt9211c_support_timing[index].framerate + 3)))
		{
			// frame rate not match
			index++;
			continue;
		}

		memset(&ctx->lt9211c_video_timing, 0, sizeof(ctx->lt9211c_video_timing));

		ctx->lt9211c_video_timing.hfront_porch = lt9211c_support_timing[index].hfront_porch;
		ctx->lt9211c_video_timing.hsync_len = lt9211c_support_timing[index].hsync_len;
		ctx->lt9211c_video_timing.hback_porch = lt9211c_support_timing[index].hback_porch;
		ctx->lt9211c_video_timing.hactive = lt9211c_support_timing[index].hactive;
		ctx->lt9211c_video_timing.htotal = lt9211c_support_timing[index].htotal;
		ctx->lt9211c_video_timing.vfront_porch = lt9211c_support_timing[index].vfront_porch;
		ctx->lt9211c_video_timing.vsync_len = lt9211c_support_timing[index].vsync_len;
		ctx->lt9211c_video_timing.vback_porch = lt9211c_support_timing[index].vback_porch;
		ctx->lt9211c_video_timing.vactive = lt9211c_support_timing[index].vactive;
		ctx->lt9211c_video_timing.vtotal = lt9211c_support_timing[index].vtotal;
		ctx->lt9211c_video_timing.framerate = current_framerate;

		ctx->lt9211c_video_timing.pclk_khz = (u32)ctx->lt9211c_video_timing.htotal * (u32)ctx->lt9211c_video_timing.vtotal * lt9211c_support_timing[index].framerate / 1000;
		ret = lt9211c_mipi_rx_video_timing_set(ctx, ctx->lt9211c_video_timing);

		return ret;
	}
	return -ENOPARAM;
}// end lt9211c_mipi_rx_video_timing_sel

static int lt9211c_mipi_rx_dessc_pll_sdm_cal(struct lt9211c *ctx, struct lt9211c_pcr_setting pcr_setting)
{
	int ret = 0;
	struct reg_sequence pre_reg_cfg[] = {
		{0xff, 0xd0},
		{0x08, 0x00},//sel mipi rx sdm
	};
	u8 data;

	ret = regmap_multi_reg_write(ctx->regmap, pre_reg_cfg, ARRAY_SIZE(pre_reg_cfg));
	ret = regmap_write(ctx->regmap, 0x26, 0x80 | (u8)pcr_setting.pcr_m);
	ret = regmap_write(ctx->regmap, 0x2d, pcr_setting.pcr_up_limit); //PCR M overflow limit setting.
	ret = regmap_write(ctx->regmap, 0x31, pcr_setting.pcr_down_limit); //PCR M underflow limit setting.

	ret = regmap_write(ctx->regmap, 0x27, (u8)(pcr_setting.pcr_k >> 16));
	ret = regmap_write(ctx->regmap, 0x28, (u8)(pcr_setting.pcr_k >> 8));
	ret = regmap_write(ctx->regmap, 0x29, (u8)pcr_setting.pcr_k);

	ret = regmap_bulk_read(ctx->regmap, 0x26, &data, 1);
	ret = regmap_write(ctx->regmap, 0x26, data & 0x7f);
	return ret;
}

static int lt9211c_mipi_rx_dessc_pll_set(struct lt9211c *ctx)
{
	int ret = 0;
	u8 dessc_pll_pixel_clock_div = 0;
	struct lt9211c_pcr_setting pcr_setting;

	struct reg_sequence pre_reg_cfg[] = {
		{0xff,0x82},
		{0x26,0x20}, //[7:6]desscpll reference select Xtal clock as reference
                              //[4]1'b0: dessc-pll power down
		{0x27,0x40},  //prediv = 0;
        };
	struct reg_sequence suffix_reg_cfg[] = {
                {0xff,0x81},
                {0x03,0xfe}, // desscpll rst
        };

	ret = regmap_multi_reg_write(ctx->regmap, pre_reg_cfg, ARRAY_SIZE(pre_reg_cfg));

	if (ctx->lt9211c_video_timing.pclk_khz >= 352000)
	{
		ret = regmap_write(ctx->regmap, 0x2f, 0x04);
		dessc_pll_pixel_clock_div = 2;
	}
	else if (ctx->lt9211c_video_timing.pclk_khz >= 176000)
	{
		ret = regmap_write(ctx->regmap, 0x2f, 0x04);
		dessc_pll_pixel_clock_div = 2;
	}
	else if (ctx->lt9211c_video_timing.pclk_khz >= 88000)
	{
		ret = regmap_write(ctx->regmap, 0x2f, 0x05);
		dessc_pll_pixel_clock_div = 4;
	}
	else if (ctx->lt9211c_video_timing.pclk_khz >= 44000)
        {
                ret = regmap_write(ctx->regmap, 0x2f, 0x06);
                dessc_pll_pixel_clock_div = 8;
        }
	else if (ctx->lt9211c_video_timing.pclk_khz >= 22000)
        {
                ret = regmap_write(ctx->regmap, 0x2f, 0x07);
                dessc_pll_pixel_clock_div = 16;
        }
	 else
        {
                ret = regmap_write(ctx->regmap, 0x2f, 0x07);
                dessc_pll_pixel_clock_div = 16;
                ret = regmap_write(ctx->regmap, 0x2c, 0x01); //desscpll lowf pixck divsel: /2
        }

	memset(&pcr_setting, 0, sizeof(pcr_setting));

	pcr_setting.pcr_m = ctx->lt9211c_video_timing.pclk_khz * dessc_pll_pixel_clock_div / 25;
	pcr_setting.pcr_k = pcr_setting.pcr_m % 1000;
	pcr_setting.pcr_m = pcr_setting.pcr_m / 1000;
	pcr_setting.pcr_up_limit = pcr_setting.pcr_m + 1;
	pcr_setting.pcr_down_limit = pcr_setting.pcr_m - 1;
	pcr_setting.pcr_k <<= 14;

	lt9211c_mipi_rx_dessc_pll_sdm_cal(ctx, pcr_setting);
	ret = regmap_multi_reg_write(ctx->regmap, suffix_reg_cfg, ARRAY_SIZE(suffix_reg_cfg));
	usleep_range(1000, 1000);

	ret = regmap_write(ctx->regmap, 0x03, 0xff); //desscpll rst
	return ret;
} // end lt9211c_mipi_rx_dessc_pll_set

static int lt9211c_mipi_rx_pcr_calibration(struct lt9211c *ctx)
{
	int ret = 0;
	u8 pcr_cal_count = 0;
	struct reg_sequence pre_reg_cfg[] = {
		{0xff,0xd0},
		{0x0c,0x60},  //fifo position
		{0x1c,0x60},  //fifo position
		{0x24,0x70},  //pcr mode( de hs vs)

		{0x2d,0x30}, //M up limit
		{0x31,0x0a}, //M down limit

        /*stage1 hs mode*/
		{0x25,0xf0},  //line limit
		{0x2a,0x30},  //step in limit
		{0x21,0x4f},  //hs_step
		{0x22,0x00},

        /*stage2 hs mode*/
		{0x1e,0x01},  //RGD_DIFF_SND[7:4],RGD_DIFF_FST[3:0]
		{0x23,0x80},  //hs_step
    /*stage2 de mode*/
		{0x0a,0x02}, //de adjust pre line
		{0x38,0x02}, //de_threshold 1
		{0x39,0x04}, //de_threshold 2
		{0x3a,0x08}, //de_threshold 3
		{0x3b,0x10}, //de_threshold 4

		{0x3f,0x04}, //de_step 1
		{0x40,0x08}, //de_step 2
		{0x41,0x10}, //de_step 3
		{0x42,0x20}, //de_step 4

		{0x2b,0xa0}, //stable out

		{0xff,0xd0},   //enable HW pcr_m

		{0x26,0x97},
		{0x26,0x17},
		{0x27,0x0f},

		{0xff,0x81},  //pcr reset
		{0x20,0xbf}, // mipi portB div issue
		{0x20,0xff},
        };

	struct reg_sequence suffix_reg_cfg[] = {
		{0xff,0x81},
		{0x09,0xdb},
		{0x09,0xdf}, //pcr rst

		{0xff,0xd0},
		{0x08,0x80},
		{0x08,0x00},
	};

	ret = regmap_multi_reg_write(ctx->regmap, pre_reg_cfg, ARRAY_SIZE(pre_reg_cfg));
	usleep_range(5000, 5000);
	ret = regmap_write(ctx->regmap, 0x0b, 0x6f);
	ret = regmap_write(ctx->regmap, 0x0b, 0xff);

	if (ctx->lt9211c_video_timing.pclk_khz < 44000)
	{
		if (ctx->dsi->mode_flags & MIPI_DSI_MODE_VIDEO_BURST)
		//if (!(ctx->dsi->mode_flags & MIPI_DSI_MODE_VIDEO_BURST))
		{
			struct reg_sequence reg_cfg[] = {
				{0x0c,0x40}, //[7:0]rgd_vsync_dly(sram rd delay)
				{0x1b,0x00}, //pcr wr dly[15:0]
				{0x1c,0x40}, //pcr wr dly[7:0]
			};
			ret = regmap_multi_reg_write(ctx->regmap, reg_cfg, ARRAY_SIZE(reg_cfg));
		}
		else {
			struct reg_sequence reg_cfg[] = {
				{0x0c,0x60}, //[7:0]rgd_vsync_dly(sram rd delay)
				{0x1b,0x00}, //pcr wr dly[15:0]
				{0x1c,0x60}, //pcr wr dly[7:0]
                        };
			ret = regmap_multi_reg_write(ctx->regmap, reg_cfg, ARRAY_SIZE(reg_cfg));
		}
	}
	else {
		struct reg_sequence reg_cfg[] = {
			{0x0c,0x40}, //[7:0]rgd_vsync_dly(sram rd delay)
			{0x1b,0x00}, //pcr wr dly[15:0]
			{0x1c,0x40}, //pcr wr dly[7:0]
		};
		ret = regmap_multi_reg_write(ctx->regmap, reg_cfg, ARRAY_SIZE(reg_cfg));
	}
	ret = regmap_multi_reg_write(ctx->regmap, suffix_reg_cfg, ARRAY_SIZE(suffix_reg_cfg));
	usleep_range(10*1000, 10* 1000);
	pcr_cal_count = 0;

	while(1) {
		u8 data;

		usleep_range(500 * 1000, 500 * 1000);
		ret = regmap_bulk_read(ctx->regmap, 0x94, &data, 1);
		dev_err(ctx->dev, "%s# PCR unstable, m=0x%02x\n", __func__, data & 0x7f);
		ret = regmap_bulk_read(ctx->regmap, 0x87, &data, 1);
		if ( (data & 0x18) == 0x18)
			break;
		if (pcr_cal_count > 50)
			return -ENOPARAM;
		pcr_cal_count++;
	}
	dev_info(ctx->dev, "%s# PCR stable\n", __func__);

	return ret;
}// end lt9211c_mipi_rx_pcr_calibration

static int lt9211c_lvds_tx_phy_poweroff(struct lt9211c *ctx)
{
        struct reg_sequence reg_cfg[] = {
		{0xff,0x82},
		{0x36,0x00}, //lvds enable
		{0x37,0x00},
        };
        int ret = 0;

	ret = regmap_multi_reg_write(ctx->regmap, reg_cfg, ARRAY_SIZE(reg_cfg));
        if (ret < 0) {
                dev_err(ctx->dev, "#%s# failed to config, ret==%d\n", __func__, ret);
                return -EPROBE_DEFER;
        }
	return ret;
}

static int lt9211c_lvds_tx_phy_poweron(struct lt9211c *ctx)
{
	struct reg_sequence reg_cfg[] = {
		// LVDSTX_PORT_SEL  == DOU_PORT
		{0xff,0x82},
		{0x36,0x03}, //lvds enable
		{0x37,0x44}, //port rterm enable
		// end LVDSTX_PORT_SEL  == DOU_PORT
		{0x38,0x14},
		{0x39,0x31},
		{0x3a,0xc8},
		{0x3b,0x00},
		{0x3c,0x0f},

		{0x46,0x40},
		{0x47,0x40},
		{0x48,0x40},
		{0x49,0x40},
		{0x4a,0x40},
		{0x4b,0x40},
		{0x4c,0x40},
		{0x4d,0x40},
		{0x4e,0x40},
		{0x4f,0x40},
		{0x50,0x40},
		{0x51,0x40},

		{0xff,0x81},
		{0x03,0xbf}, //mltx reset
		{0x03,0xff}, //mltx release
        };
        int ret = 0;

	ret = regmap_multi_reg_write(ctx->regmap, reg_cfg, ARRAY_SIZE(reg_cfg));
        if (ret < 0) {
                dev_err(ctx->dev, "#%s# failed to config, ret==%d\n", __func__, ret);
                return -EPROBE_DEFER;
        }
        return ret;
}// end lt9211c_lvds_tx_phy_poweron

static u32 lt9211c_system_fm_clk_get(struct lt9211c *ctx, u8 clk_src);

static int lt9211c_lvds_tx_pll_ref_pixel_clk_get(struct lt9211c *ctx)
{
	int ret = 0;

	ctx->lt9211c_video_timing.pclk_khz = lt9211c_system_fm_clk_get(ctx, LT9211C_AD_DESSCPLL_PIX_CLK);
	return ret;
}

static int lt9211c_lvds_tx_pll_ref_pixel_clk_set(struct lt9211c *ctx)
{
	int ret = 0;
	struct reg_sequence reg_cfg[] = {
                {0xff,0x82},
                {0x30,0x00}, //[7]0:txpll normal work; txpll ref clk sel pix clk
        };

	ret = regmap_multi_reg_write(ctx->regmap, reg_cfg, ARRAY_SIZE(reg_cfg));
        if (ret < 0) {
                dev_err(ctx->dev, "#%s# failed to config, ret==%d\n", __func__, ret);
                return -EPROBE_DEFER;
        }
	return ret;
}

static int lt9211c_lvds_tx_pll_config(struct lt9211c *ctx)
{
	const static u8 BIT0_1 = 0x01;
	int ret = 0;
	u8 data;
	u8 pre_div = 0, serial_clk_div = 0, div_set = 0;
	u32 pixel_clk_div = 0, lvds_tx_phy_clk = 0;

	/* txphyclk = vco clk * ucSericlkDiv */
	// (LVDSTX_PORT_SEL == DOU_PORT)
	lvds_tx_phy_clk = ctx->lt9211c_video_timing.pclk_khz * 7 / 2; //2 port: byte clk = pix clk / 2
	ret = regmap_write(ctx->regmap, 0xff, 0x85);
	ret = regmap_bulk_read(ctx->regmap, 0x6f, &data, 1);
	ret = regmap_write(ctx->regmap, 0x6f, data | BIT0_1); //htotal -> 2n
	// end (LVDSTX_PORT_SEL == DOU_PORT)

	/*txpll prediv sel*/
	ret = regmap_write(ctx->regmap, 0xff, 0x82);
	if (ctx->lt9211c_video_timing.pclk_khz < 20000)
	{
		ret = regmap_write(ctx->regmap, 0x31, 0x28);//[2:0]3'b000: pre div set div1
		pre_div = 1;
	}
	else if (ctx->lt9211c_video_timing.pclk_khz < 40000)
	{
		ret = regmap_write(ctx->regmap, 0x31, 0x28);//[2:0]3'b000: pre div set div1
		pre_div = 1;

	}
	else if (ctx->lt9211c_video_timing.pclk_khz < 80000)
        {
                ret = regmap_write(ctx->regmap, 0x31, 0x29);
                pre_div = 2;

        }
	else if (ctx->lt9211c_video_timing.pclk_khz < 160000)
        {
                ret = regmap_write(ctx->regmap, 0x31, 0x2a);
                pre_div = 4;

        }
	else if (ctx->lt9211c_video_timing.pclk_khz < 320000)
        {
                ret = regmap_write(ctx->regmap, 0x31, 0x2b);
                pre_div = 8;

        }
	else
	{
		ret = regmap_write(ctx->regmap, 0x31, 0x2f);
                pre_div = 16;
	}

	/*txpll serick_divsel*/
	ret = regmap_write(ctx->regmap, 0xff, 0x82);
	if (lvds_tx_phy_clk >= 640000) //640M~1.28G
	{
		ret = regmap_write(ctx->regmap, 0x32, 0x42);
                serial_clk_div = 1; //sericlk div1 [6:4]:0x40
	}
	else if (lvds_tx_phy_clk >= 320000)
        {
                ret = regmap_write(ctx->regmap, 0x32, 0x02);
                serial_clk_div = 2; //sericlk div2 [6:4]:0x00
        }
	else if (lvds_tx_phy_clk >= 160000)
        {
                ret = regmap_write(ctx->regmap, 0x32, 0x12);
                serial_clk_div = 4; //sericlk div4 [6:4]:0x10
        }
	else if (lvds_tx_phy_clk >= 80000)
        {
                ret = regmap_write(ctx->regmap, 0x32, 0x22);
                serial_clk_div = 8; //sericlk div8 [6:4]:0x20
        }
	else //40M~80M
	{
		ret = regmap_write(ctx->regmap, 0x32, 0x32);
                serial_clk_div = 16; //sericlk div16 [6:4]:0x30
	}

	/* txpll_pix_mux_sel & txpll_pixdiv_sel*/
	ret = regmap_write(ctx->regmap, 0xff, 0x82);
	if (ctx->lt9211c_video_timing.pclk_khz < 150000)
        {
                ret = regmap_write(ctx->regmap, 0x33, 0x04); //pixclk > 150000, pixclk mux sel (vco clk / 3.5)
                pixel_clk_div = 7;
        }
	else
	{
		pixel_clk_div = (u8)((lvds_tx_phy_clk * serial_clk_div * 2) / (ctx->lt9211c_video_timing.pclk_khz * 7));

		if (pixel_clk_div <= (1 * 2))
		{
			ret = regmap_write(ctx->regmap, 0x33, 0x00);//pixclk div sel /7
		}
		else if (pixel_clk_div <= (2 * 2))
		{
			ret = regmap_write(ctx->regmap, 0x33, 0x01);//pixclk div sel / 14
		}
		else if (pixel_clk_div <= (4 * 2))
                {
                        ret = regmap_write(ctx->regmap, 0x33, 0x02);//pixclk div sel / 28
                }
		else if (pixel_clk_div <= (8 * 2))
                {
                        ret = regmap_write(ctx->regmap, 0x33, 0x03);//pixclk div sel / 56
                }
		else
		{
                        ret = regmap_write(ctx->regmap, 0x33, 0x03);//pixclk div sel / 56
		}
	}

	div_set = (u8)((lvds_tx_phy_clk * serial_clk_div) / (ctx->lt9211c_video_timing.pclk_khz / pre_div));

	ret = regmap_write(ctx->regmap, 0x34, 0x01); //txpll div set software output enable
	ret = regmap_write(ctx->regmap, 0x35, div_set);

	return ret;
}// end lt9211c_lvds_tx_pll_config

static int lt9211c_lvds_tx_pll_calibration(struct lt9211c *ctx)
{
	int ret = 0;
	int count = 0;
	u8 data;

	ret = regmap_write(ctx->regmap, 0xff, 0x81);
	ret = regmap_write(ctx->regmap, 0x0c, 0xfe); //txpll reset
	usleep_range(1000, 1000);
	ret = regmap_write(ctx->regmap, 0x0c, 0xff); //txpll release

	while (count <= 3)
	{
		struct reg_sequence reg_cfg[] = {
			{0xff,0x87},
			{0x0f,0x00},
			{0x0f,0x01},
		};
		ret = regmap_multi_reg_write(ctx->regmap, reg_cfg, ARRAY_SIZE(reg_cfg));
		usleep_range(20 * 1000, 20 * 1000);

		ret = regmap_bulk_read(ctx->regmap, 0x39, &data, 1);
		data &= 0x01;

		if (data == 0x01)
			break;
		count++;
	}
	ret = regmap_bulk_read(ctx->regmap, 0x39, &data, 1);
	if ( data & 0x04)
	{
		dev_info(ctx->dev, "%s# Tx Pll Lock\n", __func__);
		return 0;
	}
	dev_warn(ctx->dev, "%s# Tx Pll Unlocked\n", __func__);
	return -ENOPARAM;

}// end lt9211c_lvds_tx_pll_calibration

static int lt9211c_lvds_tx_port_set(struct lt9211c *ctx)
{
	int ret = 0;
	u8 data;

	ret = regmap_write(ctx->regmap, 0xff, 0x85);
	// (LVDSTX_PORT_SEL == DOU_PORT)
	ret = regmap_bulk_read(ctx->regmap, 0x6f, &data, 1);
	ret = regmap_write(ctx->regmap, 0x6f, data | 0x90);
	// end (LVDSTX_PORT_SEL == DOU_PORT)
	return ret;
}

static int lt9211c_lvds_tx_video_format_set(struct lt9211c *ctx, struct lt9211c_video_timing video_timing)
{
        const static unsigned int BIT3_0 = 0xf7;
        const static unsigned int BIT6_0 = 0xbf;
        int ret = 0;
        u8 data;

        ret = regmap_write(ctx->regmap, 0xff, 0x85);
        // (LVDSTX_MODE == SYNC_MODE)
        ret = regmap_bulk_read(ctx->regmap, 0x6e, &data, 1);
        ret = regmap_write(ctx->regmap, 0x6e, data & BIT3_0);
        // end (LVDSTX_MODE == SYNC_MODE)

	// (LVDSTX_DATAFORMAT == VESA)
        ret = regmap_bulk_read(ctx->regmap, 0x6f, &data, 1);
        ret = regmap_write(ctx->regmap, 0x6f, data & BIT6_0);
        // end (LVDSTX_DATAFORMAT == VESA)

        // (LVDSTX_COLORSPACE == RGB)
        // (LVDSTX_COLORDEPTH == DEPTH_8BIT)
        ret = regmap_bulk_read(ctx->regmap, 0x6f, &data, 1);
        ret = regmap_write(ctx->regmap, 0x6f, data | 0x04);
        // end (LVDSTX_COLORDEPTH == DEPTH_8BIT)
        // end (LVDSTX_COLORSPACE == RGB)

        // (LVDSTX_SYNC_INTER_MODE != ENABLED)
        ret = regmap_write(ctx->regmap, 0x68, 0x00);
        // end (LVDSTX_SYNC_INTER_MODE != ENABLED)

        return ret;
}// end lt9211c_lvds_tx_video_format_set

static int lt9211c_lvds_tx_lane_num_set(struct lt9211c *ctx)
{
	int ret = 0;
        struct reg_sequence reg_cfg[] = {
		{0xff,0x85},
		{0x4a,0x01}, //[0]hl_swap_en; [7:6]tx_pt0_src_sel: 0-pta;1-ptb
		{0x4b,0x00},
		{0x4c,0x10},
		{0x4d,0x20},
		{0x4e,0x50},
		{0x4f,0x30},
		// (LVDSTX_LANENUM  == FOUR_LANE)
		{0x50,0x46}, //[7:6]tx_pt1_src_sel: 0-pta;1-ptb
		{0x51,0x10},
		{0x52,0x20},
		{0x53,0x50},
		{0x54,0x30},
		{0x55,0x00}, //[7:4]pt1_tx4_src_sel
		{0x56,0x20},
		// end (LVDSTX_LANENUM  == FOUR_LANE)
        };

        ret = regmap_multi_reg_write(ctx->regmap, reg_cfg, ARRAY_SIZE(reg_cfg));
        if (ret < 0) {
                dev_err(ctx->dev, "#%s# failed to config, ret==%d\n", __func__, ret);
                return -EPROBE_DEFER;
        }
	return ret;
}// end lt9211c_lvds_tx_lane_num_set

static int lt9211c_lvds_tx_port_swap(struct lt9211c *ctx)
{
        static u8 BIT6_1 = 0x40;
        int ret = 0;
        u8 data;
        // PortA
        struct reg_sequence reg_cfg[] = {
                {0xff, 0x85},
                {0x4a, 0x01},
        };
        ret = regmap_multi_reg_write(ctx->regmap, reg_cfg, ARRAY_SIZE(reg_cfg));
        ret = regmap_bulk_read(ctx->regmap, 0x50, &data, 1);
        data &= BIT6_1;
        ret = regmap_write(ctx->regmap, 0x50, data);
        return ret;
}

static int lt9211c_lvds_tx_sw_reset(struct lt9211c *ctx)
{
	int ret = 0;
        struct reg_sequence reg_cfg[] = {
		{0xff, 0x81},
		{0x08, 0x6f},//LVDS TX SW reset
	};
	ret = regmap_multi_reg_write(ctx->regmap, reg_cfg, ARRAY_SIZE(reg_cfg));

	usleep_range(2000, 2000);

	ret = regmap_write(ctx->regmap, 0x08, 0x7f);
        return ret;
}

static int lt9211c_mipi_rx_dig_set(struct lt9211c *ctx)
{
	int ret = 0;
	ret = lt9211c_mipi_rx_input_sel(ctx);
	ret = lt9211c_mipi_rx_lane_set(ctx);
	return ret;
}

static int lt9211c_lvds_tx_dig_set(struct lt9211c *ctx, struct lt9211c_video_timing video_timing)
{
	int ret = 0;

	ret = lt9211c_lvds_tx_port_set(ctx);
	ret = lt9211c_lvds_tx_video_format_set(ctx, video_timing);
	ret = lt9211c_lvds_tx_lane_num_set(ctx);
	ret = lt9211c_lvds_tx_port_swap(ctx);
	ret = lt9211c_lvds_tx_sw_reset(ctx);
	return ret;
}

static int lt9211c_system_video_chk_clk_src_sel(struct lt9211c *ctx, u8 clk_src)
{
        struct reg_sequence reg_cfg[] = {
		{0x80,0xfc},
	};
	u32 data;
	int ret = 0;

	ret = regmap_write(ctx->regmap, 0xff, 0x81);
	ret = regmap_bulk_read(ctx->regmap, 0x80, &data, 1);
	reg_cfg[0].def &= data;

	ret = regmap_multi_reg_write(ctx->regmap, reg_cfg, ARRAY_SIZE(reg_cfg));
        if (ret < 0) {
                dev_err(ctx->dev, "#%s# failed to config, ret==%d\n", __func__, ret);
                return -EPROBE_DEFER;
        }
	ret = regmap_bulk_read(ctx->regmap, 0x80, &data, 1);
	ret = regmap_write(ctx->regmap, 0x80, data | clk_src);
	return ret;
}

static int lt9211c_system_video_chk_src_sel(struct lt9211c *ctx)
{
        const unsigned int MIPIDEBUG = 0x05;
        struct reg_sequence reg_cfg[] = {
                {0x3f,0xf8},
                // select MIPIDEBUG
                {0x3f,MIPIDEBUG},
                // end select MIPIDEBUG
        };
        u32 data;
        int ret = 0;

        ret = regmap_write(ctx->regmap, 0xff, 0x86);
        ret = regmap_bulk_read(ctx->regmap, 0x80, &data, 1);
        reg_cfg[0].def &= data;

        ret = regmap_multi_reg_write(ctx->regmap, reg_cfg, ARRAY_SIZE(reg_cfg));
        if (ret < 0) {
                dev_err(ctx->dev, "#%s# failed to config, ret==%d\n", __func__, ret);
                return -EPROBE_DEFER;
        }
        return ret;
}

static int lt9211c_system_act_rx_sel(struct lt9211c *ctx)
{
        const unsigned int MIPIRX = 0x01;
	const unsigned int BIT4_1 = 0x10;
        struct reg_sequence reg_cfg[] = {
                {0x30,0xf8},
        };
        u32 data;
        int ret = 0;

        ret = regmap_write(ctx->regmap, 0xff, 0x85);
        ret = regmap_bulk_read(ctx->regmap, 0x30, &data, 1);
        reg_cfg[0].def &= data;

        ret = regmap_multi_reg_write(ctx->regmap, reg_cfg, ARRAY_SIZE(reg_cfg));
        if (ret < 0) {
                dev_err(ctx->dev, "#%s# failed to config, ret==%d\n", __func__, ret);
                return -EPROBE_DEFER;
        }

        ret = regmap_bulk_read(ctx->regmap, 0x30, &data, 1);
	ret = regmap_write(ctx->regmap, 0x30, data | MIPIRX);
        ret = regmap_bulk_read(ctx->regmap, 0x30, &data, 1);
	ret = regmap_write(ctx->regmap, 0x30, data | BIT4_1);  //[5:4]01: MIPIRX
        return ret;
}


static int lt9211c_system_tx_sram_sel(struct lt9211c *ctx)
{
	const unsigned int BIT6_1 = 0x40;
	struct reg_sequence reg_cfg[] = {
                {0xff,0x85},
        };
	int ret = 0;
	u8 data;

	ret = regmap_multi_reg_write(ctx->regmap, reg_cfg, ARRAY_SIZE(reg_cfg));
        if (ret < 0) {
                dev_err(ctx->dev, "#%s# failed to config, ret==%d\n", __func__, ret);
                return -EPROBE_DEFER;
        }
	ret = regmap_bulk_read(ctx->regmap, 0x30, &data, 1);
	ret = regmap_write(ctx->regmap, 0x30, data & 0x3f);
	ret = regmap_bulk_read(ctx->regmap, 0x30, &data, 1);
	ret = regmap_write(ctx->regmap, 0x30, data | BIT6_1);
	return ret;
}

static u32 lt9211c_system_fm_clk_get(struct lt9211c *ctx, u8 clk_src)
{
	const static unsigned int BIT7_0 = 0x7f;
	const static unsigned int BIT7_1 = 0x80;
        int ret = 0;
	u32 return_data;
        u8 data;

	ret = regmap_write(ctx->regmap, 0xff, 0x86);
	ret = regmap_write(ctx->regmap, 0x90, clk_src);

	usleep_range(5000, 5000);

	ret = regmap_write(ctx->regmap, 0x90, clk_src | BIT7_1);

	ret = regmap_bulk_read(ctx->regmap, 0x98, &data, 1);
	return_data = data & 0x0f;
	ret = regmap_bulk_read(ctx->regmap, 0x99, &data, 1);
	return_data = (return_data << 8) + data;
	ret = regmap_bulk_read(ctx->regmap, 0x9a, &data, 1);
	return_data = (return_data << 8) + data;

	ret = regmap_bulk_read(ctx->regmap, 0x90, &data, 1);
	ret = regmap_write(ctx->regmap, 0x90, clk_src | BIT7_0);

	return return_data;
}

static int lt9211c_video_check_get(struct lt9211c *ctx, struct lt9211c_video_timing *video_timing)
{
	int ret = 0;
	u8 data;
	struct reg_sequence reg_cfg[] = {
		{0xff,0x81},
		{0x0b,0x7f},
		{0x0b,0xff},
        };

	ret = regmap_multi_reg_write(ctx->regmap, reg_cfg, ARRAY_SIZE(reg_cfg));
	usleep_range(80 * 1000, 80 * 1000);
	ret = regmap_write(ctx->regmap, 0xff, 0x86);

	ret = regmap_bulk_read(ctx->regmap, 0x60, &data, 1);
	video_timing->htotal = data << 8;
	ret = regmap_bulk_read(ctx->regmap, 0x61, &data, 1);
	video_timing->htotal += data;

	ret = regmap_bulk_read(ctx->regmap, 0x5c, &data, 1);
        video_timing->hactive = data << 8;
        ret = regmap_bulk_read(ctx->regmap, 0x5d, &data, 1);
        video_timing->hactive += data;

	ret = regmap_bulk_read(ctx->regmap, 0x58, &data, 1);
        video_timing->hfront_porch = data << 8;
        ret = regmap_bulk_read(ctx->regmap, 0x59, &data, 1);
        video_timing->hfront_porch += data;

	ret = regmap_bulk_read(ctx->regmap, 0x50, &data, 1);
        video_timing->hsync_len = data << 8;
        ret = regmap_bulk_read(ctx->regmap, 0x51, &data, 1);
        video_timing->hsync_len += data;

	ret = regmap_bulk_read(ctx->regmap, 0x54, &data, 1);
        video_timing->hback_porch = data << 8;
        ret = regmap_bulk_read(ctx->regmap, 0x55, &data, 1);
        video_timing->hback_porch += data;

	ret = regmap_bulk_read(ctx->regmap, 0x62, &data, 1);
        video_timing->vtotal = data << 8;
        ret = regmap_bulk_read(ctx->regmap, 0x63, &data, 1);
        video_timing->vtotal += data;

	ret = regmap_bulk_read(ctx->regmap, 0x5e, &data, 1);
        video_timing->vactive = data << 8;
        ret = regmap_bulk_read(ctx->regmap, 0x5f, &data, 1);
        video_timing->vactive += data;

	ret = regmap_bulk_read(ctx->regmap, 0x5a, &data, 1);
        video_timing->vfront_porch = data << 8;
        ret = regmap_bulk_read(ctx->regmap, 0x5b, &data, 1);
        video_timing->vfront_porch += data;

	ret = regmap_bulk_read(ctx->regmap, 0x52, &data, 1);
        video_timing->vsync_len = data << 8;
        ret = regmap_bulk_read(ctx->regmap, 0x53, &data, 1);
        video_timing->vsync_len += data;

	ret = regmap_bulk_read(ctx->regmap, 0x56, &data, 1);
        video_timing->vback_porch = data << 8;
        ret = regmap_bulk_read(ctx->regmap, 0x57, &data, 1);
        video_timing->vback_porch += data;

	return ret;
}// end lt9211c_video_check_get

static int lt9211c_mipi_rx_source_config(struct lt9211c *ctx)
{
	int ret = 0;

	ret = lt9211c_mipi_rx_phy_poweron(ctx);
	ret = lt9211c_mipi_rx_clk_sel(ctx);
	ret = lt9211c_system_video_chk_clk_src_sel(ctx, LT9211C_MLRX_BYTE_CLK);
	ret = lt9211c_system_video_chk_src_sel(ctx);
	ret = lt9211c_system_act_rx_sel(ctx);
	ret = lt9211c_mipi_rx_dig_set(ctx);
	return ret;
}

static int lt9211c_mipi_rx_video_timing_config(struct lt9211c *ctx)
{
        int ret = 0;

	ret = lt9211c_mipi_rx_video_timing_get(ctx);
	if (ret < 0) {
                return ret;
        }
	ret = lt9211c_mipi_rx_video_timing_sel(ctx);
	if (ret < 0){
		dev_err(ctx->dev, "%s#Select video timing failed\n", __func__);
	}
	return ret;
}

static int lt9211c_mipi_rx_pll_config(struct lt9211c *ctx)
{
	int ret = 0;

	ret = lt9211c_mipi_rx_dessc_pll_set(ctx);
	ret = lt9211c_mipi_rx_pcr_calibration(ctx);
	if (ret < 0) {
		dev_err(ctx->dev, "%s# PCR calibration failed\n", __func__);
		return ret;
	}
	ret = lt9211c_system_video_chk_clk_src_sel(ctx, LT9211C_DESSCPLL_PIX_CLK);
	ret = lt9211c_system_video_chk_src_sel(ctx);
	return ret;
}

static int lt9211c_mipi_rx_video_check_stable(struct lt9211c *ctx)
{
	int ret = 0;
	u8 data;

	ret = regmap_write(ctx->regmap, 0xff, 0x86);
	ret = regmap_bulk_read(ctx->regmap, 0x40, &data, 1);
	data &= 0x01;
	return data == 0x01 ? 0: -ENOPARAM;
}

static int lt9211c_lvds_tx_source_config(struct lt9211c *ctx)
{
	int ret = 0;

	ret = lt9211c_system_tx_sram_sel(ctx);
	ret = lt9211c_lvds_tx_phy_poweron(ctx);
	return ret;
}

static int lt9211c_lvds_tx_config_video(struct lt9211c *ctx)
{
	int ret = 0;

	ret = lt9211c_lvds_tx_pll_ref_pixel_clk_get(ctx);
	ret = lt9211c_lvds_tx_pll_ref_pixel_clk_set(ctx);
	ret = lt9211c_lvds_tx_pll_config(ctx);

	ret = lt9211c_lvds_tx_pll_calibration(ctx);
	return ret;
}

static int lt9211c_lvds_tx_config_video_out(struct lt9211c *ctx)
{
	int ret = 0;
	struct lt9211c_video_timing video_timing;

	memset(&video_timing, 0, sizeof(video_timing));

	ret = lt9211c_video_check_get(ctx, &video_timing);
	ret = lt9211c_lvds_tx_dig_set(ctx, video_timing);
	return ret;
}

static int lt9211c_module_config(struct lt9211c *ctx);

static void lt9211c_state_handler(struct work_struct *data)
{
	struct lt9211c *ctx = container_of(data, struct lt9211c, delayed_work.work);

	lt9211c_module_config(ctx);
}

static int lt9211c_init_work(struct lt9211c *ctx)
{
	ctx->state = LT9211C_STATE_PREPARE;

	INIT_DELAYED_WORK(&ctx->delayed_work, lt9211c_state_handler);
	return 0;
}

static int lt9211c_module_reset(struct lt9211c *ctx)
{
	const int delayed_msec = 60;
	gpiod_set_value_cansleep(ctx->rst_gpio, 0);
        usleep_range(delayed_msec * 1000, delayed_msec * 1000);
        gpiod_set_value_cansleep(ctx->rst_gpio, 1);
        usleep_range(delayed_msec * 1000, delayed_msec * 1000);
	return 0;
}

static int lt9211c_module_init(struct lt9211c *ctx)
{
	lt9211c_init_work(ctx);
	schedule_delayed_work(&ctx->delayed_work, msecs_to_jiffies(0));

	return 0;
}

static int lt9211c_module_config(struct lt9211c *ctx)
{
	int ret = 0;
	const int delayed_msec = 0;
	static int counter = 0;

	memset(&ctx->mipi_rx_video_timing, 0, sizeof(ctx->mipi_rx_video_timing));
	switch (ctx->state) {
		case LT9211C_STATE_PREPARE:
			counter = 0;
			lt9211c_module_reset(ctx);
			lt9211c_read_chipid(ctx);
			ret = lt9211c_mipi_rx_source_config(ctx);
			ret = lt9211c_lvds_tx_phy_poweroff(ctx);

			ctx->state = LT9211C_STATE_CHIPRX_VIDTIMING_CONFIG;
		case LT9211C_STATE_CHIPRX_VIDTIMING_CONFIG:
			ret = lt9211c_mipi_rx_video_timing_config(ctx);
			if (ret >= 0)
			{
				ctx->state = LT9211C_STATE_CHIPRX_PLL_CONFIG;
			}
			else {
				counter++;
				if (ctx->mipi_rx_video_timing.fmt == 0x0a)
					counter = 0;
				if (counter > 30)
					ctx->state = LT9211C_STATE_PREPARE;
				schedule_delayed_work(&ctx->delayed_work, msecs_to_jiffies(delayed_msec));
				return ret;
			}
		case LT9211C_STATE_CHIPRX_PLL_CONFIG:
			ret = lt9211c_mipi_rx_pll_config(ctx);
			if (ret < 0) {
				ctx->state = LT9211C_STATE_CHIPRX_VIDTIMING_CONFIG;
				schedule_delayed_work(&ctx->delayed_work, msecs_to_jiffies(delayed_msec));
				return ret;
			}
			counter = 0;
			while(lt9211c_mipi_rx_video_check_stable(ctx) < 0)
			{
				dev_info(ctx->dev, "%s# mipi rx video unstable\n", __func__);
				counter++;
				if (counter > 50)
					break;
			}
			dev_info(ctx->dev, "%s# mipi rx video stable\n", __func__);
			lt9211c_lvds_tx_source_config(ctx);
			ctx->state = LT9211C_STATE_CHIPTX_CONFIG_VIDEO;
		case LT9211C_STATE_CHIPTX_CONFIG_VIDEO:
			ret = lt9211c_lvds_tx_config_video(ctx);
			if (ret < 0) {
                                schedule_delayed_work(&ctx->delayed_work, msecs_to_jiffies(delayed_msec));
                                return ret;
                        }
			ctx->state = LT9211C_STATE_CHIPTX_VIDEO_OUT;
		case LT9211C_STATE_CHIPTX_VIDEO_OUT:
			ret = lt9211c_lvds_tx_config_video_out(ctx);
			dev_info(ctx->dev, "%s# start to play\n", __func__);
			break;

	}
	return ret;
}

static int lt9211c_get_modes(struct lt9211c *ctx,
                            struct drm_connector *connector)
{
        struct drm_display_mode *mode;

        mode = drm_mode_create(connector->dev);
        if (!mode) {
                DRM_ERROR("failed to create a new display mode\n");
                return 0;
        }

        drm_display_mode_from_videomode(&ctx->videomode, mode);
        mode->width_mm = ctx->width_mm;
        mode->height_mm = ctx->height_mm;
        connector->display_info.width_mm = mode->width_mm;
        connector->display_info.height_mm = mode->height_mm;

        mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
        drm_mode_probed_add(connector, mode);

        return 1;
}

static int lt9211c_connector_get_modes(struct drm_connector *connector)
{
        struct lt9211c *ctx = connector_to_lt9211c(connector);

        return lt9211c_get_modes(ctx, connector);
}

static enum drm_mode_status lt9211c_mode_valid(struct lt9211c *ctx,
                              const struct drm_display_mode *mode)
{
	return MODE_OK;
}

static enum drm_mode_status
lt9211c_connector_mode_valid(struct drm_connector *connector,
                             struct drm_display_mode *mode)
{
        struct lt9211c *ctx = connector_to_lt9211c(connector);

        return lt9211c_mode_valid(ctx, mode);
}

static struct drm_connector_helper_funcs lt9211c_connector_helper_funcs = {
        .get_modes = lt9211c_connector_get_modes,
        .mode_valid = lt9211c_connector_mode_valid,
};

static const struct drm_connector_funcs lt9211c_connector_funcs = {
        .fill_modes = drm_helper_probe_single_connector_modes,
        .destroy = drm_connector_cleanup,
        .reset = drm_atomic_helper_connector_reset,
        .atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
        .atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static int lt9211c_connector_init(struct lt9211c *ctx)
{
	struct drm_bridge *bridge = &ctx->bridge;
	int ret = 0;

	if (!bridge->encoder) {
                DRM_ERROR("Parent encoder object not found\n");
                return -ENODEV;
        }
	ret = drm_connector_init(bridge->dev, &ctx->connector,
                                 &lt9211c_connector_funcs,
                                 DRM_MODE_CONNECTOR_DSI);
	if (ret < 0) {
                DRM_ERROR("Failed to initialize connector with drm\n");
                return ret;
        }
	drm_connector_helper_add(&ctx->connector,
                                 &lt9211c_connector_helper_funcs);
	ret = drm_connector_attach_encoder(&ctx->connector, bridge->encoder);
	if (ret < 0) {
                DRM_ERROR("Failed to attach connector with encoder\n");
                return ret;
        }
	return 0;
}

static int lt9211c_attach_dsi(struct lt9211c *ctx)
{
	struct device *dev = ctx->dev;
	struct mipi_dsi_host *host;
        struct mipi_dsi_device *dsi;

	const struct mipi_dsi_device_info info = { .type = "lt9211c",
                                                   .channel = 0,
                                                   .node = NULL,
                                                 };
	int ret = 0;

	host = of_find_mipi_dsi_host_by_node(ctx->host_node);
        if (!host) {
                dev_err(dev, "failed to find dsi host\n");
                return -EPROBE_DEFER;
        }
	dsi = mipi_dsi_device_register_full(host, &info);
        if (IS_ERR(dsi)) {
                dev_err(dev, "failed to create dsi device\n");
                ret = PTR_ERR(dsi);
                goto err_dsi_device;
        }
	ctx->dsi = dsi;
	dsi->lanes = 4;
        dsi->format = MIPI_DSI_FMT_RGB888;
        dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST | MIPI_DSI_MODE_VIDEO_NO_HFP | MIPI_DSI_MODE_VIDEO_NO_HBP | MIPI_DSI_MODE_VIDEO_NO_HSA;
	ret = mipi_dsi_attach(dsi);
        if (ret < 0) {
                dev_err(dev, "failed to attach dsi to host\n");
                goto err_dsi_attach;
        }

	return 0;

err_dsi_attach:
        mipi_dsi_device_unregister(dsi);
err_dsi_device:
        return ret;
}

static int lt9211c_bridge_attach(struct drm_bridge *bridge,
				enum drm_bridge_attach_flags flags)
{
	struct lt9211c *ctx = bridge_to_lt9211c(bridge);
	int ret = 0;

	ret = lt9211c_connector_init(ctx);
	if (ret < 0)
		return ret;
	ret = lt9211c_attach_dsi(ctx);

	return ret;
}

static void lt9211c_detach_dsi(struct lt9211c *ctx)
{
	mipi_dsi_detach(ctx->dsi);
        mipi_dsi_device_unregister(ctx->dsi);
}

static void lt9211c_bridge_enable(struct drm_bridge *bridge)
{
	struct lt9211c *ctx = bridge_to_lt9211c(bridge);

	lt9211c_module_init(ctx);
}

static void lt9211c_bridge_disable(struct drm_bridge *bridge)
{
}

static int lt9211c_bridge_get_modes(struct drm_bridge *bridge,
                            struct drm_connector *connector)
{
	struct lt9211c *ctx = bridge_to_lt9211c(bridge);

        return lt9211c_get_modes(ctx, connector);
}

static enum drm_mode_status lt9211c_bridge_mode_valid(struct drm_bridge *bridge,
                                           const struct drm_display_info *info,
                                           const struct drm_display_mode *mode)
{
	struct lt9211c *ctx = bridge_to_lt9211c(bridge);
	return lt9211c_mode_valid(ctx, mode);
}

static void lt9211c_bridge_detach(struct drm_bridge *bridge)
{
	struct lt9211c *ctx = bridge_to_lt9211c(bridge);
	lt9211c_detach_dsi(ctx);
}

static const struct drm_bridge_funcs lt9211c_bridge_funcs = {
	.attach 		= lt9211c_bridge_attach,
	.enable			= lt9211c_bridge_enable,
	.get_modes 		= lt9211c_bridge_get_modes,
	.disable		= lt9211c_bridge_disable,
	.mode_valid		= lt9211c_bridge_mode_valid,
	.detach 		= lt9211c_bridge_detach,
};

static int lt9211c_dsi_parse_dt(struct device_node *np, struct lt9211c *ctx)
{
	ctx->host_node = of_graph_get_remote_node(np, 0, 0);
	if (!ctx->host_node)
                return -ENODEV;
	of_node_put(ctx->host_node);
	return 0;
}

static int lt9211c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct lt9211c *ctx;
	int retval = 0;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->dev = dev;

	lt9211c_dsi_parse_dt(dev->of_node, ctx);
	/* request reset pin */
	ctx->rst_gpio = devm_gpiod_get(ctx->dev, "rst", GPIOD_OUT_LOW);
	if (IS_ERR(ctx->rst_gpio))
		return PTR_ERR(ctx->rst_gpio);

	dev_warn(dev, "%s: Get rest gpio=%d, as out low\n", __func__, ctx->rst_gpio);
	/* request clock */
	ctx->dsi_mclk = devm_clk_get(dev, "dsi_mclk");
	if (IS_ERR(ctx->dsi_mclk)) {
                /* assuming clock enabled by default */
                ctx->dsi_mclk = NULL;
                dev_err(dev, "clock-frequency missing or invalid\n");
                return PTR_ERR(ctx->dsi_mclk);
        }
	retval = clk_prepare_enable(ctx->dsi_mclk);
	if (retval < 0) {
                dev_err(dev, "prepare enable clock failed, ret=%d\n", retval);
		return retval;
	}
	dev_warn(dev, "%s: Enable clock\n", __func__);
	if (retval)
		return retval;
	videomode_from_timing(&lt9211c_default_timing, &ctx->videomode);
	ctx->width_mm = g_width_mm;
	ctx->height_mm = g_height_mm;

	dev_set_drvdata(dev, ctx);
	i2c_set_clientdata(i2c, ctx);

#ifdef MODEL_LT9211C
	ctx->regmap =  devm_regmap_init_i2c(i2c, &lt9211c_regmap_config);
#else
	ctx->regmap =  devm_regmap_init_i2c(i2c, &lt9211c_regmap_config);
#endif
        if (IS_ERR(ctx->regmap)) {
                dev_err(ctx->dev, "regmap i2c init failed\n");
                return PTR_ERR(ctx->regmap);
        }
	ctx->bridge.funcs = &lt9211c_bridge_funcs;
	ctx->bridge.of_node = dev->of_node;
	ctx->bridge.type = DRM_MODE_CONNECTOR_DSI;
	drm_bridge_add(&ctx->bridge);

	return 0;
}

static void lt9211c_remove(struct i2c_client *i2c)
{
	struct lt9211c *ctx = i2c_get_clientdata(i2c);
	clk_disable_unprepare(ctx->dsi_mclk);
	drm_bridge_remove(&ctx->bridge);
}

static const struct i2c_device_id lt9211c_i2c_ids[] = {
	{ "lt9211c", MODEL_LONTIUM_LT9211C },
	{}
};
MODULE_DEVICE_TABLE(i2c, lt9211c_i2c_ids);

static const struct of_device_id lt9211c_of_ids[] = {
	{ .compatible = "lontium,lt9211c", .data = (void *)MODEL_LONTIUM_LT9211C},
	{}
};
MODULE_DEVICE_TABLE(of, lt9211c_of_ids);

static struct mipi_dsi_driver lt9211c_dsi_driver = {
        .driver.name = "lt9211c",
};

static struct i2c_driver lt9211c_driver = {
        .driver = {
                .name = "lt9211c",
                .of_match_table = lt9211c_of_ids,
        },
        .id_table = lt9211c_i2c_ids,
        .probe = lt9211c_probe,
        .remove = lt9211c_remove,
};

static int __init lt9211c_init(void)
{
	if (IS_ENABLED(CONFIG_DRM_MIPI_DSI))
                mipi_dsi_driver_register(&lt9211c_dsi_driver);
	return i2c_add_driver(&lt9211c_driver);
}
module_init(lt9211c_init);

static void __exit lt9211c_exit(void)
{
        i2c_del_driver(&lt9211c_driver);
	if (IS_ENABLED(CONFIG_DRM_MIPI_DSI))
                mipi_dsi_driver_unregister(&lt9211c_dsi_driver);
}
module_exit(lt9211c_exit);

MODULE_AUTHOR("Tony Shih <tony.s@eideal.c>");
MODULE_DESCRIPTION("LONTIUM LT9211C MIPI to LVDS transmitter driver");
MODULE_LICENSE("GPL");
