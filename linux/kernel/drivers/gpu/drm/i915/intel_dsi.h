/*
 * Copyright © 2013 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef _INTEL_DSI_H
#define _INTEL_DSI_H

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include "intel_drv.h"

#define dsi_18Bpp_loosely_packed	0
#define dsi_18Bpp_packed		1
#define dsi_16Bpp_packed		2
#define dsi_24Bpp_packed		3

#define DSI_18BPP_LOOSELY_PACKED		0
#define DSI_18BPP_PACKED				1
#define DSI_16BPP_PACKED				2
#define DSI_24BPP_PACKED				3

#define ColorConversionInBridge		0
#define ColorConversionInHostController	1

#define DSI_VIDEO_MODE		0
#define DSI_CMD_MODE		1

#define DSI_VIDEO_NBURST_SPULSE		1
#define DSI_VIDEO_NBURST_SEVENT		2
#define DSI_VIDEO_BURST			3

#define DSI_HSS_PACKET_SIZE 4
#define DSI_HSE_PACKET_SIZE 4
#define DSI_HSA_PACKET_EXTRA_SIZE 6
#define DSI_HBP_PACKET_EXTRA_SIZE 6
#define DSI_HACTIVE_PACKET_EXTRA_SIZE 6
#define DSI_HFP_PACKET_EXTRA_SIZE 6
#define DSI_HFP_PACKET_EXTRA_SIZE 6
#define DSI_EOTP_PACKET_SIZE 4

struct intel_dsi_device {
	unsigned int panel_id;
	const char *name;
	int type;
	unsigned int lane_count;
	const struct intel_dsi_dev_ops *dev_ops;
	void *dev_priv;
};

struct intel_dsi_dev_ops {
	bool (*init)(struct intel_dsi_device *dsi);

	void (*get_info)(int pipe, struct drm_connector *connector);

	void (*create_resources)(struct intel_dsi_device *dsi);

	void (*dpms)(struct intel_dsi_device *dsi, bool enable);

	void (*panel_reset)(struct intel_dsi_device *dsi);

	void (*disable_panel_power)(struct intel_dsi_device *dsi);

	/* send one time programmable commands */
	void (*send_otp_cmds)(struct intel_dsi_device *dsi);

	/* This callback must be able to assume DSI commands can be sent */
	void (*enable)(struct intel_dsi_device *dsi);

	/* This callback must be able to assume DSI commands can be sent */
	void (*disable)(struct intel_dsi_device *dsi);

	int (*mode_valid)(struct intel_dsi_device *dsi,
			  struct drm_display_mode *mode);

	bool (*mode_fixup)(struct intel_dsi_device *dsi,
			   const struct drm_display_mode *mode,
			   struct drm_display_mode *adjusted_mode);

	void (*mode_set)(struct intel_dsi_device *dsi,
			 struct drm_display_mode *mode,
			 struct drm_display_mode *adjusted_mode);

	enum drm_connector_status (*detect)(struct intel_dsi_device *dsi);

	bool (*get_hw_state)(struct intel_dsi_device *dev);

	struct drm_display_mode *(*get_modes)(struct intel_dsi_device *dsi);

	void (*destroy) (struct intel_dsi_device *dsi);

	void (*dump_regs)(struct intel_dsi_device *dsi);
};

struct intel_dsi {
	struct intel_encoder base;

	struct intel_dsi_device dev;

	struct intel_connector *attached_connector;

	/* if true, use HS mode, otherwise LP */
	bool hs;

	/* virtual channel */
	int channel;

	/* number of DSI lanes */
	unsigned int lane_count;

	enum pipe pipe;

	/* video mode pixel format for MIPI_DSI_FUNC_PRG register */
	u32 pixel_format;

	/* video mode format for MIPI_VIDEO_MODE_FORMAT register */
	u32 video_mode_format;

	/* eot for MIPI_EOT_DISABLE register */
	u32 eot_disable;

	u16 dsi_clock_freq;
	u8 operation_mode;
	u8 video_mode_type;
	u32 data_width;
	u8 dither;
	u32 port_bits;
	u8 escape_clk_div;
	u32 lp_rx_timeout;
	u8 turn_arnd_val;
	u16 init_count;
	u16 rst_timer_val;
	u16 hs_to_lp_count;
	u16 lp_byte_clk;
	u32 bw_timer;
	u16 clk_lp_to_hs_count;
	u16 clk_hs_to_lp_count;
	u32 video_frmt_cfg_bits;
	u32 dphy_reg;

	u8 backlight_off_delay; /*in ms*/
	u8 backlight_on_delay; /*in ms*/
	bool send_shutdown;
	u8 shutdown_pkt_delay; /*in ms*/
	enum panel_fitter pfit;
};

static inline struct intel_dsi *enc_to_intel_dsi(struct drm_encoder *encoder)
{
	return container_of(encoder, struct intel_dsi, base.base);
}

extern void vlv_enable_dsi_pll(struct intel_encoder *encoder);
extern void vlv_disable_dsi_pll(struct intel_encoder *encoder);

extern struct intel_dsi_dev_ops auo_b101uan01_dsi_display_ops;
extern struct intel_dsi_dev_ops panasonic_vvx09f006a00_dsi_display_ops;
extern struct intel_dsi_dev_ops auo_b080xat_dsi_display_ops;
extern struct intel_dsi_dev_ops jdi_lpm070w425b_dsi_display_ops;
extern struct intel_dsi_dev_ops vbt_generic_dsi_display_ops;
void intel_dsi_clear_device_ready(struct intel_encoder *encoder);

#define	MIPI_DSI_UNDEFINED_PANEL_ID			0x00
#define	MIPI_DSI_GENERIC_PANEL_ID			0x01
#define	MIPI_DSI_AUO_B101UAN01_PANEL_ID			0x02
#define	MIPI_DSI_PANASONIC_VXX09F006A00_PANEL_ID	0x03
#define	MIPI_DSI_AUO_B080XAT_PANEL_ID			0x04
#define	MIPI_DSI_JDI_LPM070W425B_PANEL_ID		0x05

#endif /* _INTEL_DSI_H */
