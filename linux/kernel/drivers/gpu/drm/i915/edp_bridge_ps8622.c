/*
 * Copyright ? 2008 Intel Corporation
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
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 * Authors:
 *    Lingyan Guo <lingyan.guo@intel.com>
 *
 */


#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>

#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/syscalls.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/input/mt.h>
#include <linux/lnw_gpio.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>
#include <drm/drmP.h>
#include "intel_drv.h"
#include <drm/i915_drm.h>
#include "i915_drv.h"

#if 0
#undef DRM_DEBUG_KMS
#define DRM_DEBUG_KMS(fmt, args...)				\
	do {								\
		my_drm_ut_debug_printk(DRM_UT_KMS, DRM_NAME,		\
					 __func__, fmt, ##args);	\
	} while (0)
extern void my_drm_ut_debug_printk(unsigned int request_level,
			 const char *prefix,
			 const char *function_name,
			 const char *format, ...);
#endif

int ps8622_i2c_Read(struct i2c_client *client, u8 addr,
	char *writebuf, int writelen,
	char *readbuf, int readlen)
{
	int ret = 0;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
			 .addr = addr>>1,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
			 },
			{
			 .addr = addr>>1,
			 .flags = I2C_M_RD,
			 .len = readlen,
			 .buf = readbuf,
			 },
		};

		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			DRM_DEBUG_KMS(": i2c read error.\n");
	  }
	return ret;
}



int ps8622_read_reg(struct i2c_client *client, u8 addr, u8 regaddr, u8 *regvalue)
{
	int ret =  ps8622_i2c_Read(client, addr, &regaddr, 1, regvalue, 1);
	DRM_DEBUG_KMS("(0x%02x,0x%02x)=0x%02x ret:%d\n",addr, regaddr, *regvalue,ret);
  return ret;
}

static int ps8622_i2c_write(struct i2c_client *client, u8 addr, u8 reg, u8 val)
{
        int ret;
		int retry = 0;

        struct i2c_adapter *adap = client->adapter;
        struct i2c_msg msg;
        u8 data[] = {reg, val};

        msg.addr = addr>>1;
        msg.flags = 0;
        msg.len = sizeof(data);
        msg.buf = data;

		for (retry=0; retry<10; retry++) {
			ret = i2c_transfer(adap, &msg, 1);
			if (1 == ret)
				break;
			usleep_range(500, 2000);
		}
        if (ret < 0){
				 DRM_DEBUG_KMS("(0x%02x,0x%02x,0x%02x) I2C failed: %d\n",
                        addr , reg, val, ret);
				 return 1;
		}
		return 0;
}

static bool ps8622_send_config(struct i2c_client *client)
{
		int err = 0;
		bool ret = false;
		DRM_DEBUG_KMS(" Begin\n");
		if (client == NULL) {
				DRM_DEBUG_KMS("not i2c client\n");
				return false;
		}
		if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
				DRM_DEBUG_KMS("i2c_check_functionality() failed\n");
				return false;
		} else {
				DRM_DEBUG_KMS("i2c_check_functionality() ok\n");
		}
		usleep_range(2000, 5000);
		err |= ps8622_i2c_write(client, 0x14, 0xa1, 0x01); /* HPD low */
		/* SW setting */
		err |= ps8622_i2c_write(client, 0x18, 0x14, 0x01); /* [1:0] SW output 1.2V voltage
													* is lower to 96% */
		/* RCO SS setting */
		err |= ps8622_i2c_write(client, 0x18, 0xe3, 0x20); /* [5:4] = b01 0.5%, b10 1%,
													* b11 1.5% */
		err |= ps8622_i2c_write(client, 0x18, 0xe2, 0x80); /* [7] RCO SS enable */
		/* RPHY Setting */
		err |= ps8622_i2c_write(client, 0x18, 0x8a, 0x0c); /* [3:2] CDR tune wait cycle
													* before measure for fine tune
													* b00: 1us b01: 0.5us b10:2us
													* b11: 4us */
		err |= ps8622_i2c_write(client, 0x18, 0x89, 0x08); /* [3] RFD always on */
		err |= ps8622_i2c_write(client, 0x18, 0x71, 0x2d); /* CTN lock in/out:
													* 20000ppm/80000ppm.
													* Lock out 2 times. */
		/* 2.7G CDR settings */
		err |= ps8622_i2c_write(client, 0x18, 0x7d, 0x07); /* NOF=40LSB for HBR CDR
													* setting */
		err |= ps8622_i2c_write(client, 0x18, 0x7b, 0x00); /* [1:0] Fmin=+4bands */
		err |= ps8622_i2c_write(client, 0x18, 0x7a, 0xfd); /* [7:5] DCO_FTRNG=+-40% */
		/* 1.62G CDR settings */
		err |= ps8622_i2c_write(client, 0x18, 0xc0, 0x12); /* [5:2]NOF=64LSB [1:0]DCO
													* scale is 2/5 */
		err |= ps8622_i2c_write(client, 0x18, 0xc1, 0x92); /* Gitune=-37% */
		err |= ps8622_i2c_write(client, 0x18, 0xc2, 0x1c); /* Fbstep=100% */
		err |= ps8622_i2c_write(client, 0x18, 0x32, 0x80); /* [7] LOS signal disable */
		/* RPIO Setting */
		err |= ps8622_i2c_write(client, 0x18, 0x00, 0xb0); /* [7:4] LVDS driver bias
													* current : 75% (250mV swing)
													* */
		err |= ps8622_i2c_write(client, 0x18, 0x15, 0x40); /* [7:6] Right-bar GPIO output
													* strength is 8mA */
		/* EQ Training State Machine Setting */
		err |= ps8622_i2c_write(client, 0x18, 0x54, 0x10); /* RCO calibration start */
		/* Logic, needs more than 10 I2C command */
		err |= ps8622_i2c_write(client, 0x12, 0x02, 0x81); /* [4:0] MAX_LANE_COUNT set to
													* one lane */
		err |= ps8622_i2c_write(client, 0x12, 0x21, 0x81); /* [4:0] LANE_COUNT_SET set to
													* one lane (in case no-link
													* traing conflict) */
		err |= ps8622_i2c_write(client, 0x10, 0x52, 0x20);
		err |= ps8622_i2c_write(client, 0x10, 0xf1, 0x03); /* HPD CP toggle enable */
		err |= ps8622_i2c_write(client, 0x10, 0x62, 0x41);
		err |= ps8622_i2c_write(client, 0x10, 0xf6, 0x01); /* Counter number, add 1ms
													* counter delay */
		err |= ps8622_i2c_write(client, 0x14, 0xa1, 0x10);
		err |= ps8622_i2c_write(client, 0x10, 0x77, 0x06); /* [6]PWM function control by
													* DPCD0040f[7], default is PWM
													* block always works. */
		err |= ps8622_i2c_write(client, 0x10, 0x4c, 0x04); /* 04h Adjust VTotal tolerance
													* to fix the 30Hz no display
													* issue */
		err |= ps8622_i2c_write(client, 0x12, 0xc0, 0x00); /* DPCD00400='h00, Parade OUI =
													* 'h001cf8 */
		err |= ps8622_i2c_write(client, 0x12, 0xc1, 0x1c); /* DPCD00401='h1c */
		err |= ps8622_i2c_write(client, 0x12, 0xc2, 0xf8); /* DPCD00402='hf8 */
		err |= ps8622_i2c_write(client, 0x12, 0xc3, 0x44); /* DPCD403~408 = ASCII code
													* D2SLV5='h4432534c5635 */
		err |= ps8622_i2c_write(client, 0x12, 0xc4, 0x32); /* DPCD404 */
		err |= ps8622_i2c_write(client, 0x12, 0xc5, 0x53); /* DPCD405 */
		err |= ps8622_i2c_write(client, 0x12, 0xc6, 0x4c); /* DPCD406 */
		err |= ps8622_i2c_write(client, 0x12, 0xc7, 0x56); /* DPCD407 */
		err |= ps8622_i2c_write(client, 0x12, 0xc8, 0x35); /* DPCD408 */
		err |= ps8622_i2c_write(client, 0x12, 0xca, 0x01); /* DPCD40A, Initial Code major
													* revision '01' */
		err |= ps8622_i2c_write(client, 0x12, 0xcb, 0x07); /* DPCD40B, Initial Code minor
													* revision '07' */
		err |= ps8622_i2c_write(client, 0x12, 0xa5, 0xa0); /* DPCD720, internal PWM */
		err |= ps8622_i2c_write(client, 0x12, 0xa7, 0xff); /* FFh for 100% brightness,
													*  0h for 0% brightness */
		err |= ps8622_i2c_write(client, 0x12, 0xcc, 0x00); /* Set LVDS output as 6bit-VESA
													* mapping, single LVDS channel
													* */
		err |= ps8622_i2c_write(client, 0x18, 0x10, 0x16); /* Enable SSC set by register
													* */
		err |= ps8622_i2c_write(client, 0x18, 0x59, 0x60);
		err |= ps8622_i2c_write(client, 0x18, 0x54, 0x14);
		err |= ps8622_i2c_write(client, 0x10, 0x3C, 0x40);
		err |= ps8622_i2c_write(client, 0x10, 0x05, 0x0C);
		err |= ps8622_i2c_write(client, 0x14, 0xa1, 0x91); /* HPD high */

		usleep_range(2000, 5000);

		ret = err ? false : true;
		DRM_DEBUG_KMS(" End. ret:%d\n", ret);
		return ret;
}




static struct i2c_client *ps8622_client = NULL;
void dump_stack(void);
static int ps8622_bridge_remove(struct i2c_client *client)
{
	DRM_DEBUG_KMS("\n");
	dump_stack();

	ps8622_client = NULL;

	return 0;
}

static
int ps8622_bridge_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	DRM_DEBUG_KMS("\n");
	ps8622_client = client;
	return 0;
}

static
const struct i2c_device_id ps8622_bridge_id[] = {
	{ "i2c_disp_brig", 0 },
	{ }
};
struct i2c_driver ps8622_bridge_i2c_driver = {
	.driver = {
		.name = "i2c_disp_brig",
	},
	.id_table = ps8622_bridge_id,
	.probe = ps8622_bridge_probe,
	.remove = ps8622_bridge_remove,
};
struct i2c_board_info ps8622_i2c_info = {
		.type = "i2c_disp_brig",
		.addr = 0x10,
};

int ps8622_init(void) {
	int ret = 0;
	int i2c_busnum = 3; /*  MRD 7 in I2C 3 */
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	adapter = i2c_get_adapter(i2c_busnum);
	DRM_DEBUG_KMS("\n");
		if (!adapter) {
			DRM_DEBUG_KMS("i2c_get_adapter(%d) failed\n", i2c_busnum);
		return -EINVAL;
		}

	client = i2c_new_device(adapter, &ps8622_i2c_info);
	if (!client) {
		DRM_DEBUG_KMS("i2c_new_device() failed\n");
		i2c_put_adapter(adapter);
		return -EINVAL;
	}
	ret = i2c_add_driver(&ps8622_bridge_i2c_driver);
	if (ret) {
		DRM_DEBUG_KMS("add bridge I2C driver faild\n");
		i2c_unregister_device(client);
		return -EINVAL;
	}
	return 0;
}

void ps8622_send_init_cmd(struct intel_dp *intel_dp)
{
	if (intel_dp->bridge_setup_done) {
		DRM_DEBUG_KMS("Skip bridge setup\n");
		return;
	}

	if (ps8622_send_config(ps8622_client)) {
		DRM_DEBUG_KMS("bridge setup done\n");
		intel_dp->bridge_setup_done = true;
	}
}
