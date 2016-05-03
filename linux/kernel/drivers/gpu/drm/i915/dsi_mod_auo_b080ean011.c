/*
 * Copyright Â© 2013 Intel Corporation
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
 *
 * Author: Jani Nikula <jani.nikula@intel.com>
 *	   Shobhit Kumar <shobhit.kumar@intel.com>
 *     Lingyan Guo <lingyan.guo@intel.com>
 *
 *
 */

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>
#include <drm/i915_drm.h>
#include <linux/slab.h>
#include <video/mipi_display.h>
#include <asm/intel-mid.h>
#include "i915_drv.h"
#include "intel_drv.h"
#include "intel_dsi.h"
#include "intel_dsi_cmd.h"
#include "dsi_mod_auo_b080ean011.h"


static u8 auo_enable_ic_power[]      = {0xC3, 0x40, 0x00, 0x28};
static u8 auo_disable_ic_power[]      = {0xC3, 0x40, 0x00, 0x20};
static u8 auo_password[]      = {0xF0, 0x5A, 0x5A};

static u8 auo_init_sequence1[]      = {0xF1, 0x5A, 0x5A};
static u8 auo_init_sequence2[]      = {0xFC, 0xA5, 0xA5};
static u8 auo_init_sequence3[]      = {0xD0, 0x00, 0x10};
static u8 auo_init_sequence4[]      = {0xbc, 0x1, 0x4e,0xa0};
static u8 auo_init_sequence5[]      = {0xE1, 0x3, 0x10,0x1C,0X82,0X07};
static u8 auo_init_sequence6[]      = {0xB1, 0x10};
static u8 auo_init_sequence7[]      = {0xB2, 0x14, 0x22,0x2F,0X04};
static u8 auo_init_sequence8[]      = {0xF2, 0x02, 0xC,0x8,0X88,0X018};
static u8 auo_init_sequence9[]      = {0xB5, 0x1e};
static u8 auo_init_sequence10[]      = {0xB0, 0x04};
static u8 auo_init_sequence11[]      = {0xFD, 0x9};
static u8 auo_init_sequence12[]      = {0xF6, 0x63, 0x21,0x86,0X00,0X00,0};
static u8 auo_init_sequence13[]      = {0xD8, 0x5E, 0x4C,0x10};
static u8 auo_init_sequence14[]      = {0xF3, 0x1, 0xC0,0xE0,0X62,0XD0,0X81,0X35,0XF3,0X30,0X24,0};
static u8 auo_init_sequence15[]      = {0XF4,0X00,0X02,0X03 ,0X26 ,0X03 ,0X02 ,0X09 ,0X00 ,0X07 ,0X16 ,0X16 ,0X03 ,0X00
 ,0X08 ,0X08 ,0X03 ,0X00 ,0X00 ,0X12 ,0X1C ,0X1D ,0X1E ,0X01 ,0X09 ,0X01 ,0X04 ,0X02 ,0X61 ,0X74 ,0X75
 ,0X72 ,0X83 ,0X80 ,0X80,0X00 ,0X00 ,0X01 ,0X01 ,0X28 ,0X04 ,0X03 ,0X28 ,0X01 ,0XD1 ,0X32};
static u8 auo_init_sequence16[]      = {0XF5  ,0X9D  ,0X42  ,0X42  ,0X5F  ,0XAB  ,0X98  ,0X4F  ,0X0F
  ,0X33  ,0X43  ,0X04  ,0X59  ,0X54  ,0X52  ,0X05  ,0X40  ,0X60  ,0X40  ,0X60  ,0X40  ,0X27  ,0X26  ,0X52  ,0X25
  ,0X6D  ,0X18}; 

static u8 auo_init_sequence17[] ={0XEE ,0X3F ,0X3F ,0X3F 
,0X00 ,0X3F ,0X3F ,0X3F
 ,0X00 ,0X11 ,0X22};

static u8 auo_init_sequence18[] ={0XEF ,0X12 ,0X12 ,0X43 
,0X43 ,0X90 ,0X84 ,0X24
 ,0X81 ,0X00 ,0X21 ,0X21
 ,0X03 ,0X03 ,0X40 ,0X80 
,0X82 ,0X00};
static u8 auo_init_sequence19[]= {0XFA ,0X00 ,0X35 ,0X06 ,0X0A ,0X14 ,0X0D ,0X13 ,0X19 
	,0X1C ,0X25 ,0X2B ,0X32 ,0X3B ,0X39 ,0X3D ,0X38 ,0X3D ,0X25 ,0X30 ,0X26 ,0X2A ,0X2B ,0X1E ,0X22 
	,0X23 ,0X22 ,0X28 ,0X2D ,0X33 ,0X3B ,0X38 ,0X2D ,0X2D ,0X2A ,0X0C ,0X35 ,0X10 ,0X14 
	,0X1C ,0X14 ,0X1A ,0X1E ,0X1F ,0X27 ,0X2C ,0X33 ,0X3B ,0X38 ,0X30 ,0X30 ,0X30};

static u8 auo_init_sequence20[] = {0XFA ,0X00 ,0X35 ,0X06 ,0X0A ,0X14 ,0X0D ,0X13 ,0X19 
	,0X1C ,0X25 ,0X2B ,0X32 ,0X3B ,0X39 ,0X3D ,0X38 ,0X3D ,0X25 ,0X30 ,0X26 ,0X2A ,0X2B ,0X1E ,0X22 
	,0X23 ,0X22 ,0X28 ,0X2D ,0X33 ,0X3B ,0X38 ,0X2D ,0X2D ,0X2A ,0X0C ,0X35 ,0X10 ,0X14 
	,0X1C ,0X14 ,0X1A ,0X1E ,0X1F ,0X27 ,0X2C ,0X33 ,0X3B ,0X38 ,0X30 ,0X30 ,0X30};

static u8 auo_init_sequence21[] ={0XF7 ,0X0B ,0X0B ,0X09 
,0X09 ,0X0A ,0X0A ,0X08
 ,0X08 ,0X01 ,0X16 ,0X16
 ,0X17 ,0X17 ,0X07 ,0X01
 ,0X01 ,0X0B ,0X0B ,0X09
 ,0X09 ,0X0A ,0X0A ,0X08 
,0X08 ,0X01 ,0X16 ,0X16
 ,0X17 ,0X17 ,0X07 ,0X01
 ,0X01};

void b080ean011_send_otp_cmds(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	DRM_DEBUG_KMS("\n");
	dsi_vc_dcs_write(intel_dsi, 0, auo_password, sizeof(auo_password));
	dsi_vc_dcs_write(intel_dsi, 0, auo_init_sequence1, sizeof(auo_init_sequence1));
	dsi_vc_dcs_write(intel_dsi, 0, auo_init_sequence2, sizeof(auo_init_sequence2));
	dsi_vc_dcs_write(intel_dsi, 0, auo_init_sequence3, sizeof(auo_init_sequence3));
	dsi_vc_dcs_write(intel_dsi, 0, auo_init_sequence4, sizeof(auo_init_sequence4));
	dsi_vc_dcs_write(intel_dsi, 0, auo_init_sequence5, sizeof(auo_init_sequence5));
	dsi_vc_dcs_write(intel_dsi, 0, auo_init_sequence6, sizeof(auo_init_sequence6));
	dsi_vc_dcs_write(intel_dsi, 0, auo_init_sequence7, sizeof(auo_init_sequence7));
	dsi_vc_dcs_write(intel_dsi, 0, auo_init_sequence8, sizeof(auo_init_sequence8));
	dsi_vc_dcs_write(intel_dsi, 0, auo_init_sequence9, sizeof(auo_init_sequence9));
	dsi_vc_dcs_write(intel_dsi, 0, auo_init_sequence10, sizeof(auo_init_sequence10));
	dsi_vc_dcs_write(intel_dsi, 0, auo_init_sequence11, sizeof(auo_init_sequence11));
	dsi_vc_dcs_write(intel_dsi, 0, auo_init_sequence12, sizeof(auo_init_sequence12));
	dsi_vc_dcs_write(intel_dsi, 0, auo_init_sequence13, sizeof(auo_init_sequence13));
	dsi_vc_dcs_write(intel_dsi, 0, auo_init_sequence14, sizeof(auo_init_sequence14));
	dsi_vc_dcs_write(intel_dsi, 0, auo_init_sequence15, sizeof(auo_init_sequence15));
	dsi_vc_dcs_write(intel_dsi, 0, auo_init_sequence16, sizeof(auo_init_sequence16));
	dsi_vc_dcs_write(intel_dsi, 0, auo_init_sequence17, sizeof(auo_init_sequence17));
	dsi_vc_dcs_write(intel_dsi, 0, auo_init_sequence18, sizeof(auo_init_sequence18));
	dsi_vc_dcs_write(intel_dsi, 0, auo_init_sequence19, sizeof(auo_init_sequence19));
	dsi_vc_dcs_write(intel_dsi, 0, auo_init_sequence20, sizeof(auo_init_sequence20));
	dsi_vc_dcs_write(intel_dsi, 0, auo_init_sequence21, sizeof(auo_init_sequence21));


}


static void  b080ean011_get_panel_info(int pipe,
					struct drm_connector *connector)
{
	DRM_DEBUG_KMS("\n");
	if (!connector) {
		DRM_DEBUG_KMS("Cpt: Invalid input to get_info\n");
		return;
	}

	if (pipe == 0) {
		connector->display_info.width_mm = 192;
		connector->display_info.height_mm = 120;
	}

	return;
}

static void b080ean011_destroy(struct intel_dsi_device *dsi)
{
}

static void b080ean011_dump_regs(struct intel_dsi_device *dsi)
{
}

static void b080ean011_create_resources(struct intel_dsi_device *dsi)
{
}

static struct drm_display_mode *b080ean011_get_modes(
	struct intel_dsi_device *dsi)
{
	struct drm_display_mode *mode = NULL;
	DRM_DEBUG_KMS("\n");
	/* Allocate */
	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode) {
		DRM_DEBUG_KMS("Cpt panel: No memory\n");
		return NULL;
	}

	/* Hardcode 800*1280 */
	/*HFP = 16, HSYNC = 5, HBP = 59 */
	/*VFP = 8, VSYNC = 5, VBP = 3 */
	mode->hdisplay = 800;
	mode->hsync_start = mode->hdisplay + 24;
	mode->hsync_end = mode->hsync_start + 8;
	mode->htotal = mode->hsync_end  + 128;

	mode->vdisplay = 1280;
	mode->vsync_start = mode->vdisplay + 12;
	mode->vsync_end = mode->vsync_start + 8;
	mode->vtotal = mode->vsync_end + 12;

	mode->vrefresh = 60;
	mode->clock =  mode->vrefresh * mode->vtotal *
	mode->htotal / 1000;

	/* Configure */
	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	return mode;
}


static bool b080ean011_get_hw_state(struct intel_dsi_device *dev)
{
	DRM_DEBUG_KMS("\n");
	return true;
}

static enum drm_connector_status b080ean011_detect(
					struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
 	dev_priv->is_mipi = true;
	DRM_DEBUG_KMS("\n");
	return connector_status_connected;
}

static bool b080ean011_mode_fixup(struct intel_dsi_device *dsi,
		    const struct drm_display_mode *mode,
		    struct drm_display_mode *adjusted_mode) {
	return true;
}
#define DISP_RST_N 107
void b080ean011_panel_reset(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	DRM_DEBUG_KMS("\n");

	vlv_gpio_nc_write(dev_priv, GPIO_NC_9_PCONF0, 0x2000CC00);
	vlv_gpio_nc_write(dev_priv, GPIO_NC_9_PAD, 0x00000004);
	/* panel disable */
	vlv_gpio_nc_write(dev_priv, GPIO_NC_11_PCONF0, 0x2000CC00);
	vlv_gpio_nc_write(dev_priv, GPIO_NC_11_PAD, 0x00000004);
	usleep_range(100000, 120000);
	/* panel enable */
	vlv_gpio_nc_write(dev_priv, GPIO_NC_11_PCONF0, 0x2000CC00);
	vlv_gpio_nc_write(dev_priv, GPIO_NC_11_PAD, 0x00000005);
	usleep_range(100000, 120000);
	vlv_gpio_nc_write(dev_priv, GPIO_NC_9_PAD, 0x00000005);
}

static int b080ean011_mode_valid(struct intel_dsi_device *dsi,
		   struct drm_display_mode *mode)
{
	DRM_DEBUG_KMS("\n");
	return MODE_OK;
}

static void b080ean011_dpms(struct intel_dsi_device *dsi, bool enable)
{
//	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	DRM_DEBUG_KMS("\n");
}
static void b080ean011_enable(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	DRM_DEBUG_KMS("\n");
	dsi_vc_dcs_write(intel_dsi, 0, auo_password, sizeof(auo_password));
	dsi_vc_dcs_write_0(intel_dsi, 0, 0x11);
	msleep(30);
	dsi_vc_dcs_write_0(intel_dsi, 0, 0x29);
	msleep(30);
	dsi_vc_dcs_write(intel_dsi, 0, auo_enable_ic_power, sizeof(auo_enable_ic_power));
}
static void b080ean011_disable(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	DRM_DEBUG_KMS("\n");
	dsi_vc_dcs_write_0(intel_dsi, 0, 0x28);
	msleep(30);
	dsi_vc_dcs_write(intel_dsi, 0, auo_disable_ic_power, sizeof(auo_disable_ic_power));
	msleep(15);
	dsi_vc_dcs_write_0(intel_dsi, 0, 0x10);
	msleep(30);
}

bool b080ean011_init(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
//	struct drm_device *dev = intel_dsi->base.base.dev;
//	struct drm_i915_private *dev_priv = dev->dev_private;

	/* create private data, slam to dsi->dev_priv. could support many panels
	 * based on dsi->name. This panal supports both command and video mode,
	 * so check the type. */

	/* where to get all the board info style stuff:
	 *
	 * - gpio numbers, if any (external te, reset)
	 * - pin config, mipi lanes
	 * - dsi backlight? (->create another bl device if needed)
	 * - esd interval, ulps timeout
	 *
	 */

	DRM_DEBUG_KMS("Init: b080ean011 panel\n");

	if (!dsi) {
		DRM_DEBUG_KMS("Init: Invalid input to b080ean011\n");
		return false;
	}

	intel_dsi->eotp_pkt = 1;
	intel_dsi->operation_mode = DSI_VIDEO_MODE;
	intel_dsi->video_mode_type = DSI_VIDEO_NBURST_SEVENT;
	intel_dsi->pixel_format = VID_MODE_FORMAT_RGB888;
	intel_dsi->port_bits = 0;
	intel_dsi->turn_arnd_val = 0x14;
	intel_dsi->rst_timer_val = 0xffff;
	intel_dsi->bw_timer = 0x820;
	/*b044*/
	intel_dsi->hs_to_lp_count = 0x18;
	/*b060*/
	intel_dsi->lp_byte_clk = 0x03;
	/*b080*/
	intel_dsi->dphy_reg = 0x170d340b;
	/* b088 high 16bits */
	intel_dsi->clk_lp_to_hs_count = 0x1e;
	/* b088 low 16bits */
	intel_dsi->clk_hs_to_lp_count = 0x0d;
	/* BTA sending at the last blanking line of VFP is disabled */
	intel_dsi->video_frmt_cfg_bits = 1<<3;
	intel_dsi->lane_count = 4;

	intel_dsi->backlight_off_delay = 20;
	intel_dsi->send_shutdown = true;
	intel_dsi->shutdown_pkt_delay = 20;
	//dev_priv->mipi.panel_bpp = 24;
	return true;
}


/* Callbacks. We might not need them all. */
struct intel_dsi_dev_ops auo_b080ean011_dsi_display_ops = {
	.init = b080ean011_init,
	.get_info = b080ean011_get_panel_info,
	.create_resources = b080ean011_create_resources,
	.dpms = b080ean011_dpms,
	.mode_valid = b080ean011_mode_valid,
	.mode_fixup = b080ean011_mode_fixup,
	.panel_reset = b080ean011_panel_reset,
	.detect = b080ean011_detect,
	.get_hw_state = b080ean011_get_hw_state,
	.get_modes = b080ean011_get_modes,
	.destroy = b080ean011_destroy,
	.dump_regs = b080ean011_dump_regs,
	.enable = b080ean011_enable,
	.disable = b080ean011_disable,
	.send_otp_cmds = b080ean011_send_otp_cmds,
};
