/*
 * intel_byt_cr_thermal.c - Intel Baytrail Platform Thermal Driver
 *
 * Copyright (C) 2014 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Shravan B M <shravan.k.b.m@intel.com>
 */

#define pr_fmt(fmt)  "intel_byt_cr_thermal: " fmt

#include <linux/pm.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/thermal.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <linux/mfd/intel_mid_pmic.h>

#include <asm/intel_mid_thermal.h>

#include <linux/iio/consumer.h>

#define DRIVER_NAME "byt_cr_thermal"

/* Number of Thermal sensors on the PMIC */
#define PMIC_THERMAL_SENSORS	2

/* ADC to Temperature conversion table length */
#define TABLE_LENGTH	22

static const int adc_code[2][TABLE_LENGTH] = {
	{6987, 5263, 4004, 3076, 2385,
	 1864, 1469, 1166, 932, 750,
	 607, 495, 405, 334, 277,
	 230, 192, 162, 136, 116,
	 98, 84 },
	{-20, -15, -10, -5, 0,
	 5, 10, 15, 20, 25,
	 30, 35, 40, 45, 50,
	 55, 60, 65, 70, 75,
	 80, 85},
};

static DEFINE_MUTEX(thrm_update_lock);

struct thermal_device_info {
	struct intel_mid_thermal_sensor *sensor;
	int sensor_index;
};

struct thermal_data {
	struct platform_device *pdev;
	struct iio_channel *iio_chan;
	struct thermal_zone_device **tzd;
	/* Caching information */
	bool is_initialized;
	unsigned long last_updated;
	int cached_vals[PMIC_THERMAL_SENSORS];
	/* Details obtained from platform data */
	int num_sensors;
	struct intel_mid_thermal_sensor *sensors;
};
static struct thermal_data *tdata;

static inline int adc_to_pmic_die_temp(unsigned int val)
{
	/* return temperature in mC */
	return  -267700 + val * 100;
}

/**
 * find_adc_code - searches the ADC code using binary search
 * @val: value to find in the array
 *
 * This function does binary search on an array sorted in 'descending' order
 * Can sleep
 */
static int find_adc_code(uint16_t val)
{
	int left = 0;
	int right = TABLE_LENGTH - 1;
	int mid;
	while (left <= right) {
		mid = (left + right)/2;
		if (val == adc_code[0][mid] ||
			(mid > 0 &&
			val > adc_code[0][mid] && val < adc_code[0][mid-1]))
			return mid;
		else if (val > adc_code[0][mid])
			right = mid - 1;
		else if (val < adc_code[0][mid])
			left = mid + 1;
	}
	return -EINVAL;
}

/**
 * adc_to_temp - converts the ADC code to temperature in mC
 * @direct: true if the sensor uses direct conversion
 * @adc_val: the ADC code to be converted
 * @tp: temperature return value
 *
 * Can sleep
 */
static int adc_to_temp(int direct, uint16_t adc_val, long *tp)
{
	int x0, x1, y0, y1;
	int nr, dr;		/* Numerator & Denominator */
	int indx;
	int x = adc_val;

	/* Direct conversion for pmic die temperature */
	if (direct) {
		*tp = adc_to_pmic_die_temp(adc_val);
		return 0;
	}

	indx = find_adc_code(adc_val);
	if (indx < 0)
		return -EINVAL;

	if (adc_code[0][indx] == adc_val) {
		*tp = adc_code[1][indx] * 1000;
		return 0;
	}

	/*
	 * The ADC code is in between two values directly defined in the
	 * table. So, do linear interpolation to calculate the temperature.
	 */
	x0 = adc_code[0][indx];
	x1 = adc_code[0][indx - 1];
	y0 = adc_code[1][indx];
	y1 = adc_code[1][indx - 1];

	/*
	 * Find y:
	 * Of course, we can avoid these variables, but keep them
	 * for readability and maintainability.
	 */
	nr = (x-x0)*y1 + (x1-x)*y0;
	dr =  x1-x0;

	if (!dr)
		return -EINVAL;

	/*
	 * We have to report the temperature in milli degree celsius.
	 * So, to reduce the loss of precision, do (Nr*1000)/Dr, instead
	 * of (Nr/Dr)*1000.
	 */
	*tp = (nr * 1000)/dr;
	return 0;
}

static int update_temp(struct thermal_zone_device *tzd, long *temp)
{
	int ret;
	struct thermal_device_info *td_info = tzd->devdata;
	int indx = td_info->sensor_index;

	if (!tdata->iio_chan)
		return -EINVAL;

	if (!tdata->is_initialized ||
			time_after(jiffies, tdata->last_updated + HZ)) {
		ret = iio_read_channel_all_raw(tdata->iio_chan,
						tdata->cached_vals);
		if (ret == -ETIMEDOUT) {
			dev_err(&tzd->device,
				"ADC sampling failed:%d Reading rslt regs\n",
				ret);
		}
		tdata->last_updated = jiffies;
		tdata->is_initialized = true;
	}
	if (tdata->cached_vals[indx] == 0) {
		*temp = 0;
		return ret;
	}
	ret = adc_to_temp(td_info->sensor->direct,
				tdata->cached_vals[indx], temp);
	if (ret)
		return ret;

	if (td_info->sensor->temp_correlation) {
		ret = td_info->sensor->temp_correlation(td_info->sensor,
							*temp, temp);
	}
	return ret;
}

static ssize_t show_temp(struct thermal_zone_device *tzd, long *temp)
{
	int ret;

	mutex_lock(&thrm_update_lock);
	ret = update_temp(tzd, temp);
	mutex_unlock(&thrm_update_lock);

	return ret;
}

static struct thermal_device_info *initialize_sensor(int index,
				struct intel_mid_thermal_sensor *sensor)
{
	struct thermal_device_info *td_info =
		kzalloc(sizeof(struct thermal_device_info), GFP_KERNEL);

	if (!td_info)
		return NULL;

	td_info->sensor = sensor;
	td_info->sensor_index = index;

	return td_info;
}

#ifdef CONFIG_DEBUG_THERMAL
static int read_slope(struct thermal_zone_device *tzd, long *slope)
{
	struct thermal_device_info *td_info = tzd->devdata;

	*slope = td_info->sensor->slope;

	return 0;
}

static int update_slope(struct thermal_zone_device *tzd, long slope)
{
	struct thermal_device_info *td_info = tzd->devdata;

	td_info->sensor->slope = slope;

	return 0;
}

static int read_intercept(struct thermal_zone_device *tzd, long *intercept)
{
	struct thermal_device_info *td_info = tzd->devdata;

	*intercept = td_info->sensor->intercept;

	return 0;
}

static int update_intercept(struct thermal_zone_device *tzd, long intercept)
{
	struct thermal_device_info *td_info = tzd->devdata;

	td_info->sensor->intercept = intercept;

	return 0;
}
#endif

static struct thermal_zone_device_ops tzd_ops = {
	.get_temp = show_temp,
#ifdef CONFIG_DEBUG_THERMAL
	.get_slope = read_slope,
	.set_slope = update_slope,
	.get_intercept = read_intercept,
	.set_intercept = update_intercept,
#endif
};

static int byt_cr_thermal_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int i, size, ret;
	struct intel_mid_thermal_platform_data *pdata;

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "Unable to fetch platform data\n");
		return -EINVAL;
	}

	tdata = kzalloc(sizeof(struct thermal_data), GFP_KERNEL);
	if (!tdata) {
		dev_err(&pdev->dev, "kzalloc failed\n");
		return -ENOMEM;
	}

	tdata->pdev = pdev;
	tdata->num_sensors = pdata->num_sensors;
	tdata->sensors = pdata->sensors;
	platform_set_drvdata(pdev, tdata);

	size = sizeof(struct thermal_zone_device *) * tdata->num_sensors;
	tdata->tzd = kzalloc(size, GFP_KERNEL);
	if (!tdata->tzd) {
		dev_err(&pdev->dev, "kzalloc failed\n");
		ret = -ENOMEM;
		goto exit_free;
	}

	/* Register with IIO to sample temperature values */
	tdata->iio_chan = iio_channel_get_all(&pdev->dev);
	if (IS_ERR(tdata->iio_chan)) {
		dev_err(&pdev->dev, "tdata->iio_chan is invalid\n");
		ret = -EINVAL;
		goto exit_tzd;
	}

	/* Check whether we got all the channels */
	ret = iio_channel_get_num(tdata->iio_chan);
	if (ret != PMIC_THERMAL_SENSORS) {
		dev_err(&pdev->dev, "incorrect number of channels:%d\n", ret);
		ret = -EFAULT;
		goto exit_iio;
	}

	/* Register each sensor with the generic thermal framework */
	for (i = 0; i < tdata->num_sensors; i++) {
		tdata->tzd[i] = thermal_zone_device_register(
				tdata->sensors[i].name,	0, 0,
				initialize_sensor(i, &tdata->sensors[i]),
				&tzd_ops, NULL, 0, 0);
		if (IS_ERR(tdata->tzd[i])) {
			ret = PTR_ERR(tdata->tzd[i]);
			dev_err(&pdev->dev,
				"registering thermal sensor %s failed: %d\n",
				tdata->sensors[i].name, ret);
			goto exit_reg;
		}
	}
	return 0;

exit_reg:
	while (--i >= 0)
		thermal_zone_device_unregister(tdata->tzd[i]);
exit_iio:
	iio_channel_release_all(tdata->iio_chan);
exit_tzd:
	kfree(tdata->tzd);
exit_free:
	kfree(tdata);
	return ret;
}

static int byt_cr_thermal_resume(struct device *dev)
{
	return 0;
}

static int byt_cr_thermal_suspend(struct device *dev)
{
	return 0;
}

static int byt_cr_thermal_remove(struct platform_device *pdev)
{
	int i;
	struct thermal_data *tdata = platform_get_drvdata(pdev);

	if (!tdata)
		return 0;

	for (i = 0; i < tdata->num_sensors; i++)
		thermal_zone_device_unregister(tdata->tzd[i]);

	iio_channel_release_all(tdata->iio_chan);
	kfree(tdata->tzd);
	kfree(tdata);

	return 0;
}

/*********************************************************************
 *		Driver initialization and finalization
 *********************************************************************/

static const struct dev_pm_ops thermal_pm_ops = {
	.suspend = byt_cr_thermal_suspend,
	.resume = byt_cr_thermal_resume,
};

static struct platform_driver byt_cr_thermal_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm = &thermal_pm_ops,
		},
	.probe = byt_cr_thermal_probe,
	.remove = byt_cr_thermal_remove,
};

static int byt_cr_thermal_module_init(void)
{
	return platform_driver_register(&byt_cr_thermal_driver);
}

static void byt_cr_thermal_module_exit(void)
{
	platform_driver_unregister(&byt_cr_thermal_driver);
}

late_initcall(byt_cr_thermal_module_init);
module_exit(byt_cr_thermal_module_exit);

MODULE_AUTHOR("Shravan B M <shravan.k.b.m@intel.com>");
MODULE_DESCRIPTION("Intel Baytrail CR Platform Thermal Driver");
MODULE_LICENSE("GPL");
