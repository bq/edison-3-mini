/*
 * platform_hm2056.c: hm2056 platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <asm/intel-mid.h>
#include <asm/intel_scu_ipcutil.h>
#include <media/v4l2-subdev.h>
#include <linux/regulator/consumer.h>
#include <linux/sfi.h>
#include <linux/mfd/intel_mid_pmic.h>
#include <linux/vlv2_plat_clock.h>
#include "platform_camera.h"
#include "platform_hm2056.h"

/* workround - pin defined for byt */
#define CAMERA_1_RESET 120
#define CAMERA_1_PWDN 124

#ifdef CONFIG_VLV2_PLAT_CLK
#define OSC_CAM0_CLK 0x1
#define CLK_19P2MHz 0x1
#endif
#ifdef CONFIG_CRYSTAL_COVE
#define VPROG_2P8V 0x66
#define VPROG_1P8V 0x5D
#define VPROG_ENABLE 0x3
#define VPROG_DISABLE 0x2
#endif

#define VPROG1_VAL 2800000
static int camera_reset;
static int camera_power_down;
static int camera_vprog1_on;

static int hm2056_flisclk_ctrl(struct v4l2_subdev *sd, int flag);

/*
 * camera sensor - hm2056 platform data
 */

static int hm2056_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;
	int pin;

	/*
	 * FIXME: WA using hardcoded GPIO value here.
	 * The GPIO value would be provided by ACPI table, which is
	 * not implemented currently.
	 */

    printk("%s, value %d\n", __func__, flag);
	pin = CAMERA_1_RESET;
	if (camera_reset < 0) {
		ret = gpio_request(pin, "camera_1_reset");
		if (ret) {
			pr_err("%s: failed to request gpio(pin %d)\n",
				__func__, pin);
			return ret;
		}
	}
	camera_reset = pin;
	ret = gpio_direction_output(pin, 1);
	if (ret) {
		pr_err("%s: failed to set gpio(pin %d) direction\n",
			__func__, pin);
		gpio_free(pin);
		return ret;
	}

	pin = CAMERA_1_PWDN;
	if (camera_power_down < 0) {
		ret = gpio_request(pin, "camera_1_power");
		if (ret) {
			pr_err("%s: failed to request gpio(pin %d)\n",
				__func__, pin);
			return ret;
		}
	}
	camera_power_down = pin;
	ret = gpio_direction_output(pin, 1);
	if (ret) {
		pr_err("%s: failed to set gpio(pin %d) direction\n",
			__func__, pin);
		gpio_free(pin);
		return ret;
	}

	if (flag) {
		gpio_set_value(camera_reset, 1);
		msleep(30);

		gpio_set_value(camera_power_down, 0);
		msleep(50);

	} else {
		gpio_set_value(camera_reset, 0);
		gpio_set_value(camera_power_down, 1);
		gpio_free(camera_reset);
		gpio_free(camera_power_down);

		camera_power_down = -1;
		camera_reset = -1;
	}

	return 0;
}

static int hm2056_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	int ret = 0;

#ifdef CONFIG_VLV2_PLAT_CLK
	if (flag) {
		int ret;
		ret = vlv2_plat_set_clock_freq(OSC_CAM0_CLK, CLK_19P2MHz);
		if (ret)
			return ret;
	}
	return vlv2_plat_configure_clock(OSC_CAM0_CLK, flag);
#endif

	return 0;
}

/*
 * The power_down gpio pin is to control hm2056's
 * internal power state.
 */
static int hm2056_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;

	if (flag) {
		if (!camera_vprog1_on) {
#ifdef CONFIG_CRYSTAL_COVE
			/*
			 * This should call VRF APIs.
			 *
			 * VRF not implemented for BTY, so call this
			 * as WAs
			 */
			ret = camera_set_pmic_power(CAMERA_1P8V, true);
			if (ret)
				return ret;
			ret = camera_set_pmic_power(CAMERA_2P8V, true);
#endif
			if (!ret)
				camera_vprog1_on = 1;
			msleep(10);
			return ret;
		}
	} else {
		if (camera_vprog1_on) {
#ifdef CONFIG_CRYSTAL_COVE
			ret = camera_set_pmic_power(CAMERA_2P8V, false);
			if (ret)
				return ret;
			ret = camera_set_pmic_power(CAMERA_1P8V, false);
#endif
			if (!ret)
				camera_vprog1_on = 0;
			return ret;
		}
	}

	return ret;
}

static int hm2056_csi_configure(struct v4l2_subdev *sd, int flag)
{
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
		ATOMISP_INPUT_FORMAT_RAW_8, atomisp_bayer_order_grbg/*atomisp_bayer_order_grbg*/, flag);
}

static struct camera_sensor_platform_data hm2056_sensor_platform_data = {
	.gpio_ctrl	= hm2056_gpio_ctrl,
	.flisclk_ctrl	= hm2056_flisclk_ctrl,
	.power_ctrl	= hm2056_power_ctrl,
	.csi_cfg	= hm2056_csi_configure,
};

void *hm2056_platform_data(void *info)
{
	camera_reset = -1;
	camera_power_down = -1;

	return &hm2056_sensor_platform_data;
}
