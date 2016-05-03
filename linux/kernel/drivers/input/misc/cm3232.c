/* drivers/input/misc/cm3232.c - cm3232 light sensor driver
 *
 * Copyright (C) 2014 Capella Microsystems, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/lightsensor.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
//#include <asm/mach-types.h>
#include <linux/cm3232.h>
#include <linux/fs.h>
//#include <asm/setup.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/acpi.h>
#include <linux/proc_fs.h>

#define D(x...) pr_info(x)

#define I2C_RETRY_COUNT 10

#define CALIBRATION_FILE_PATH	"/efs/cal_data"

#define LS_POLLING_DELAY 1000
#define CONFIG_ACPI
static void report_do_work(struct work_struct *w);
static DECLARE_DELAYED_WORK(report_work, report_do_work);
// xmtdf@ 2014.11.19    add proc --begin
#define CM3232_CONFIG_PROC_FILE     "cm3232_config"
unsigned char cm3232_config[2];
unsigned int justResume = 0;
struct hrtimer cm3232_timer;
static ssize_t cm3232_config_read_proc(struct file *file, char __user *page, size_t size, loff_t *ppos)
{
    char *ptr = page;
    int i;
    printk("cm3232_config_read_proc, justResume: %d \n", justResume);
    if (justResume == 0)
        ptr[0] = '0';
    else
        ptr[0] = '1';
    ptr[1]= 0;
    return 0;
}
static ssize_t cm3232_config_write_proc(struct file *filp, const char __user *buffer, size_t count, loff_t *off)
{
    s32 ret = 0;
    if (copy_from_user(&cm3232_config[0], buffer, count))
    {
        printk("copy from user fail\n");
        return -EFAULT;
    }
    printk("cm3232_config_write_proc, content: %d count %d\n", cm3232_config[0],count);
    if (cm3232_config[0] == '0')
        justResume = 0;
    else
        justResume = 1;
    return count;
}
static struct proc_dir_entry *cm3232_config_proc = NULL;
static const struct file_operations config_proc_ops = {
    .owner = THIS_MODULE,
    .read = cm3232_config_read_proc,
    .write = cm3232_config_write_proc,
};
static enum hrtimer_restart cm3232_ts_timer_handler(struct hrtimer *timer)
{
    justResume = 0;
    return HRTIMER_NORESTART;
}

// xmtdf@ 2014.11.19    add proc --end
struct cm3232_info {
	struct class *cm3232_class;
	struct device *ls_dev;
	struct input_dev *ls_input_dev;

	struct i2c_client *i2c_client;
	struct workqueue_struct *lp_wq;

	int als_enable;
	int als_enabled_before_suspend;
	
	int (*power)(int, uint8_t); /* power to the chip */
	
	uint32_t als_resolution; // represented using a fixed 10(-5) notation
	uint32_t cal_data; // represented using a fixed 10(-5) notation

	int lightsensor_opened;
	uint32_t current_lux_level;
	uint16_t current_adc;
	int polling_delay;
};
struct cm3232_info *lp_info;
int enable_log = 0;
bool cal_data_retrieved = false;
static struct mutex als_enable_mutex, als_disable_mutex, als_get_adc_mutex;
static int lightsensor_enable(struct cm3232_info *lpi);
static int lightsensor_disable(struct cm3232_info *lpi);

static int I2C_RxData(uint16_t slaveAddr, uint8_t *rxData, int length)
{
	uint8_t loop_i;
	uint8_t subaddr[1];

	subaddr[0] = CM3232_ALS_DATA;
  
	struct i2c_msg msg[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = 1,
		 .buf = subaddr,
		 },
		{
		 .addr = slaveAddr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++)
	{
		if (i2c_transfer(lp_info->i2c_client->adapter, msg, 2) > 0)
		{
			break;
		}

		msleep(10);
	}
	
	if (loop_i >= I2C_RETRY_COUNT)
	{
		printk(KERN_ERR "[ERR][CM3232 error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0; 
}

static int I2C_TxData(uint16_t slaveAddr, uint8_t *txData, int length)
{
	uint8_t loop_i;

	struct i2c_msg msg[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++)
	{
		if (i2c_transfer(lp_info->i2c_client->adapter, msg, 1) > 0)
		{
			break;
		}

		msleep(10);
	}

	if (loop_i >= I2C_RETRY_COUNT)
	{
		printk(KERN_ERR "[ERR][CM3232 error] %s retry over %d\n",
			__func__, I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int _cm3232_I2C_Read_Word(uint16_t slaveAddr, uint16_t* pdata)
{
	int ret = 0;
	uint8_t buffer[2];

	if (pdata == NULL)
	{
		return -EFAULT;
	}

	ret = I2C_RxData(slaveAddr, buffer, 2);
	if (ret < 0)
	{
		pr_err("[ERR][CM3232 error]%s: I2C_RxData fail, slave addr: 0x%x\n",
			__func__, slaveAddr);
		return ret;
	}
	*pdata = (buffer[1] << 8) | buffer[0];

#if 0
	/* Debug use */
	printk(KERN_DEBUG "[CM3232] %s: I2C_RxData[0x%x] = 0x%x\n",
		__func__, slaveAddr, *pdata);
#endif
	return ret;
}

static int _cm3232_I2C_Write_Byte(uint16_t SlaveAddress, uint8_t data)
{
	char buffer[2];
	int ret = 0;

	buffer[0] = CM3232_ALS_CMD; 
	buffer[1] = data;
#if 0
	/* Debug use */
	printk(KERN_DEBUG
	"[CM3232] %s: _cm3232_I2C_Write_Byte[0x%x, 0x%x, 0x%x]\n",
		__func__, SlaveAddress, buffer[0], buffer[1]);
#endif

	ret = I2C_TxData(SlaveAddress, buffer, 2);
	if (ret < 0)
	{
		pr_err("[ERR][CM3232 error]%s: I2C_TxData fail\n", __func__);
		return -EIO;
	}

	return ret;
}

static int get_ls_adc_value(uint16_t *als_step)
{
	int ret = 0;

	if (als_step == NULL)
	{
		return -EFAULT;
	}

	/* Read ALS data: */
	ret = _cm3232_I2C_Read_Word(CM3232_ADDR, als_step);
	if (ret < 0)
	{
		pr_err("[LS][CM3232 error]%s: _cm3232_I2C_Read_Word fail\n",
			__func__);
		return -EIO;
	}

	return ret;
}

static void report_lsensor_input_event(struct cm3232_info *lpi)
{
	uint16_t adc_value = 0;
	uint32_t lux_level;
	int ret = 0;
    //als_zero_try = 0;

	mutex_lock(&als_get_adc_mutex);

//als_data_try:
	ret = get_ls_adc_value(&adc_value);
	lux_level = (uint32_t)div64_u64((uint64_t)adc_value * lpi->als_resolution * lpi->cal_data, (uint64_t)100000 * 100000);
  /*  if(lux_level<20)
    {
        als_zero_try++;
        if(als_zero_try<3)
        {
            msleep(100);
            goto als_data_try;
        }
    }
    */
	//D("[LS][CM3232] %s: ADC=0x%03X, Lux Level=%d\n",
	//	__func__, adc_value, lux_level);

	lpi->current_lux_level = lux_level;
	lpi->current_adc = adc_value;
	input_report_abs(lpi->ls_input_dev, ABS_MISC, lux_level);
	input_sync(lpi->ls_input_dev);

	mutex_unlock(&als_get_adc_mutex);
}

static void report_do_work(struct work_struct *work)
{
	struct cm3232_info *lpi = lp_info;
	
 	if (enable_log)
	{
		D("[CM3232] %s\n", __func__);
	}

	report_lsensor_input_event(lpi);

	queue_delayed_work(lpi->lp_wq, &report_work, lpi->polling_delay);
}

static int als_power(int enable)
{
	struct cm3232_info *lpi = lp_info;

	if (lpi->power)
	{
		lpi->power(LS_PWR_ON, 1);
	}

	return 0;
}

static int lightsensor_get_cal_data(struct cm3232_info *lpi)
{
	struct file *cal_filp = NULL;
	int err = 0;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH, O_RDONLY, 0666);
	if (IS_ERR(cal_filp))
	{
		err = PTR_ERR(cal_filp);
		if (err != -ENOENT)
		{
			pr_err("%s: Can't open calibration data file\n", __func__);
		}
		set_fs(old_fs);
		return err;
	}

	err = cal_filp->f_op->read(cal_filp,
		(char *)&lpi->cal_data, sizeof(uint16_t), &cal_filp->f_pos);
	if (err != sizeof(uint16_t))
	{
		pr_err("%s: Can't read the calibration data from file\n", __func__);
		err = -EIO;
	}

	pr_info("%s: cal_data = %d\n",
		__func__, lpi->cal_data);

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	return err;
}

static int lightsensor_enable(struct cm3232_info *lpi)
{
	int ret = 0;
	uint8_t cmd = 0;

	mutex_lock(&als_enable_mutex);
	
	D("[LS][CM3232] %s\n", __func__);

	if (!cal_data_retrieved)
	{
		/* get calibration data */		
		ret = lightsensor_get_cal_data(lpi);
		if (ret < 0 && ret != -ENOENT)
		{
			pr_err("%s: lightsensor_get_cal_data() failed\n",
				__func__);
		}
		else
		{
			cal_data_retrieved = true;
		}
	}
	
	cmd = CM3232_ALS_IT_200MS | CM3232_ALS_HS_HIGH;
	ret = _cm3232_I2C_Write_Byte(CM3232_ADDR, cmd);
	if (ret < 0)
	{
		pr_err("[LS][CM3232 error]%s: set auto light sensor fail\n",
			__func__);
	}
	else
	{
		msleep(50);/*wait for 50 ms for the first report adc*/
		/* report an invalid value first to ensure we
		* trigger an event when adc_level is zero.
		*/

		input_report_abs(lpi->ls_input_dev, ABS_MISC, -1);
		input_sync(lpi->ls_input_dev);
		report_lsensor_input_event(lpi);
	}
	
	queue_delayed_work(lpi->lp_wq, &report_work, lpi->polling_delay);
	lpi->als_enable = 1;
	
	mutex_unlock(&als_enable_mutex);
	
	return ret;
}

static int lightsensor_disable(struct cm3232_info *lpi)
{
	int ret = 0;
	char cmd = 0;

	mutex_lock(&als_disable_mutex);

	D("[LS][CM3232] %s\n", __func__);

	cmd = CM3232_ALS_IT_200MS | CM3232_ALS_HS_HIGH | CM3232_ALS_SD;
	ret = _cm3232_I2C_Write_Byte(CM3232_ADDR, cmd);
	if (ret < 0)
	{
		pr_err("[LS][CM3232 error]%s: disable auto light sensor fail\n",
			__func__);
	}

 	cancel_delayed_work_sync(&report_work); 	
	lpi->als_enable = 0;
	
	mutex_unlock(&als_disable_mutex);
	
	return ret;
}

static int lightsensor_open(struct inode *inode, struct file *file)
{
	struct cm3232_info *lpi = lp_info;
	int rc = 0;

	D("[LS][CM3232] %s\n", __func__);
	
	if (lpi->lightsensor_opened)
	{
		pr_err("[LS][CM3232 error]%s: already opened\n", __func__);
		rc = -EBUSY;
	}
	lpi->lightsensor_opened = 1;
	
	return rc;
}

static int lightsensor_release(struct inode *inode, struct file *file)
{
	struct cm3232_info *lpi = lp_info;

	D("[LS][CM3232] %s\n", __func__);
	
	lpi->lightsensor_opened = 0;
	
	return 0;
}

static long lightsensor_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int rc, val;
	struct cm3232_info *lpi = lp_info;

	/*D("[CM3232] %s cmd %d\n", __func__, _IOC_NR(cmd));*/

	switch (cmd)
	{
		case LIGHTSENSOR_IOCTL_ENABLE:
			if (get_user(val, (unsigned long __user *)arg))
			{
				rc = -EFAULT;
				break;
			}
			D("[LS][CM3232] %s LIGHTSENSOR_IOCTL_ENABLE, value = %d\n",
				__func__, val);
			rc = val ? lightsensor_enable(lpi) : lightsensor_disable(lpi);
			break;
		case LIGHTSENSOR_IOCTL_GET_ENABLED:
			val = lpi->als_enable;
			D("[LS][CM3232] %s LIGHTSENSOR_IOCTL_GET_ENABLED, enabled %d\n",
				__func__, val);
			rc = put_user(val, (unsigned long __user *)arg);
			break;
		default:
			pr_err("[LS][CM3232 error]%s: invalid cmd %d\n",
				__func__, _IOC_NR(cmd));
			rc = -EINVAL;
	}

	return rc;
}

static const struct file_operations lightsensor_fops = {
	.owner = THIS_MODULE,
	.open = lightsensor_open,
	.release = lightsensor_release,
	.unlocked_ioctl = lightsensor_ioctl
};

static struct miscdevice lightsensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lightsensor",
	.fops = &lightsensor_fops
};

static ssize_t ls_adc_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret;
	struct cm3232_info *lpi = lp_info;

	D("[LS][CM3232] %s: ADC = 0x%04X, Lux Level = %d \n",
		__func__, lpi->current_adc, lpi->current_lux_level);
	ret = sprintf(buf, "ADC[0x%04X] => lux level %d\n",
		lpi->current_adc, lpi->current_lux_level);

	return ret;
}

static ssize_t ls_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct cm3232_info *lpi = lp_info;

	ret = sprintf(buf, "Light sensor Auto Enable = %d\n",
			lpi->als_enable);

	return ret;
}

static ssize_t ls_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ret = 0;
	int ls_auto;
	struct cm3232_info *lpi = lp_info;

	ls_auto = -1;
	sscanf(buf, "%d", &ls_auto);

	if (ls_auto != 0 && ls_auto != 1)
	{
		return -EINVAL;
	}

	if (ls_auto)
	{
        // xmtdf @ 2014.11.19      -- start
        justResume = 1;        
        hrtimer_start(&cm3232_timer, ktime_set(1, 0), HRTIMER_MODE_REL);
        // xmtdf @ 2014.11.19      -- end
		ret = lightsensor_enable(lpi);
	}
	else
	{
        hrtimer_cancel(&cm3232_timer);  // xmtdf
		ret = lightsensor_disable(lpi);
	}

	D("[LS][CM3232] %s: lpi->als_enable = %d, ls_auto=%d\n",
		__func__, lpi->als_enable, ls_auto);

	if (ret < 0)
	{
		pr_err("[LS][CM3232 error]%s: set auto light sensor fail\n",
			__func__);
	}

	return count;
}

static ssize_t ls_poll_delay_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct cm3232_info *lpi = lp_info;

	ret = sprintf(buf, "Light sensor Poll Delay = %d ms\n",
			jiffies_to_msecs(lpi->polling_delay));

	return ret;
}

static ssize_t ls_poll_delay_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int new_delay;
	struct cm3232_info *lpi = lp_info;

	sscanf(buf, "%d", &new_delay);
  
	D("new delay = %d ms, old delay = %d ms \n", 
		new_delay, jiffies_to_msecs(lpi->polling_delay));

	lpi->polling_delay = msecs_to_jiffies(new_delay);

	if (lpi->als_enable)
	{
		lightsensor_disable(lpi); 
		lightsensor_enable(lpi);
	}

	return count;
}

static uint8_t ALS_CONF = 0;
static ssize_t ls_conf_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "ALS_CONF = %x\n", ALS_CONF);
}
static ssize_t ls_conf_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	int value = 0;
	sscanf(buf, "0x%x", &value);

	ALS_CONF = value;
	printk(KERN_INFO "[LS]set ALS_CONF = %x\n", ALS_CONF);
	_cm3232_I2C_Write_Byte(CM3232_ADDR, ALS_CONF);
	
	return count;
}

static ssize_t ls_cal_data_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cm3232_info *lpi = lp_info;
	int ret;

	ret = sprintf(buf, "%d\n", lpi->cal_data);

	return ret;
}

static ssize_t ls_cal_data_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	uint32_t new_cal_data = 0;
	struct cm3232_info *lpi = lp_info;	
	struct file *cal_filp = NULL;
	mm_segment_t old_fs;
	int err = 0;

	sscanf(buf, "%d", &new_cal_data);
	if (new_cal_data != 0)
	{
		lpi->cal_data = new_cal_data;
	}
	else  // reset calibration data
	{
		lpi->cal_data = 2000000;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	cal_filp = filp_open(CALIBRATION_FILE_PATH,
			O_CREAT | O_TRUNC | O_WRONLY, 0666);
	if (IS_ERR(cal_filp))
	{
		pr_err("%s: Can't open calibration file\n", __func__);
		set_fs(old_fs);
		err = PTR_ERR(cal_filp);
		return err;
	}

	err = cal_filp->f_op->write(cal_filp,
		(char *)&lpi->cal_data, sizeof(uint32_t), &cal_filp->f_pos);
	if (err != sizeof(uint32_t))
	{
		pr_err("%s: Can't write the calibration data to file\n", __func__);
		err = -EIO;
	}

	filp_close(cal_filp, current->files);
	set_fs(old_fs);

	return count;
}

static struct device_attribute dev_attr_light_enable =
__ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP, ls_enable_show, ls_enable_store);

static struct device_attribute dev_attr_light_poll_delay =
__ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP, ls_poll_delay_show, ls_poll_delay_store);

static struct device_attribute dev_attr_light_conf =
__ATTR(conf, S_IRUGO | S_IWUSR | S_IWGRP, ls_conf_show, ls_conf_store);

static struct device_attribute dev_attr_light_adc =
__ATTR(adc, S_IRUGO | S_IWUSR | S_IWGRP, ls_adc_show, NULL);

static struct device_attribute dev_attr_light_cal_data =
__ATTR(cali, S_IRUGO | S_IWUSR | S_IWGRP, ls_cal_data_show, ls_cal_data_store);

static struct attribute *light_sysfs_attrs[] = {
&dev_attr_light_enable.attr,
&dev_attr_light_poll_delay.attr,
&dev_attr_light_conf.attr,
&dev_attr_light_adc.attr,
&dev_attr_light_cal_data.attr,
NULL
};

static struct attribute_group light_attribute_group = {
.attrs = light_sysfs_attrs,
};

static int lightsensor_setup(struct cm3232_info *lpi)
{
	int ret;

	lpi->ls_input_dev = input_allocate_device();
	if (!lpi->ls_input_dev)
	{
		pr_err("[LS][CM3232 error]%s: could not allocate ls input device\n",
			__func__);
		return -ENOMEM;
	}
	lpi->ls_input_dev->name = "cm3232-ls";
	set_bit(EV_ABS, lpi->ls_input_dev->evbit);
	input_set_abs_params(lpi->ls_input_dev, ABS_MISC, 0, 9, 0, 0);

	ret = input_register_device(lpi->ls_input_dev);
	if (ret < 0)
	{
		pr_err("[LS][CM3232 error]%s: can not register ls input device\n",
			__func__);
		goto err_free_ls_input_device;
	}
	
	ret = misc_register(&lightsensor_misc);
	if (ret < 0)
	{
		pr_err("[LS][CM3232 error]%s: can not register ls misc device\n",
			__func__);
		goto err_unregister_ls_input_device;
	}

	return ret;

err_unregister_ls_input_device:
	input_unregister_device(lpi->ls_input_dev);
err_free_ls_input_device:
	input_free_device(lpi->ls_input_dev);
	return ret;
}

static int cm3232_setup(struct cm3232_info *lpi)
{
	int ret = 0;

	als_power(1);
	msleep(5);

	ret = _cm3232_I2C_Write_Byte(CM3232_ADDR, CM3232_ALS_RESET);
	if (ret < 0)
	{
		return ret;
	}
	msleep(10);
	  
	ret = _cm3232_I2C_Write_Byte(CM3232_ADDR, CM3232_ALS_IT_200MS | CM3232_ALS_HS_HIGH);
	msleep(10);

	return ret;
}

#ifdef CONFIG_ACPI
static struct cm3232_platform_data cm3232_pdata = {
        .power = NULL,
};
#endif

static int setLightsensorStatus(struct file *file, const char *buffer,unsigned long count, void *data)
{
    char *buf;
    int ret;
    int int_1=0;
    int adc=0;
    int wait_t=0;
    int intr_f=0;
    int config_1=0;
    int pulse=0;
    int gian=0;
    u8 reg_cntrl = 0;
    if (count < 1)
        return -EINVAL;
    buf = kmalloc(count, GFP_KERNEL);
    if (!buf)
        return -ENOMEM;
    if (copy_from_user(buf, buffer, count))
    {
        kfree(buf);
        return -EFAULT;
    }
    if(buf[0]=='1')
    {
    }
    else if(buf[0]=='0')
    {
    }
	kfree(buf);
	return count;
}
static int cm3232_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret = 0;
	struct cm3232_info *lpi;
	struct cm3232_platform_data *pdata;
    struct proc_dir_entry *pent,*lent,*pxent;   

	//D("xmwuwh [CM3232] %s\n", __func__);

	lpi = kzalloc(sizeof(struct cm3232_info), GFP_KERNEL);
	if (!lpi)
	{
		return -ENOMEM;
	}
    //client->addr=0x10;
	lpi->i2c_client = client;
	lpi->i2c_client->addr=0x10;
#ifdef CONFIG_ACPI
	pdata = &cm3232_pdata;
#else
	pdata = client->dev.platform_data;
#endif	
	
	if (!pdata)
	{
		pr_err("[CM3232 error]%s: Assign platform_data error!!\n",
			__func__);
		ret = -EBUSY;
		goto err_platform_data_null;
	}
	//D("xmwuwh [CM3232]  1 %s\n", __func__);

	i2c_set_clientdata(client, lpi);

    
	//D("xmwuwh [CM3232]2  %s\n", __func__);
	lpi->power = pdata->power;

	lpi->polling_delay = msecs_to_jiffies(LS_POLLING_DELAY);

	lp_info = lpi;

	mutex_init(&als_enable_mutex);
	mutex_init(&als_disable_mutex);
	mutex_init(&als_get_adc_mutex);

	ret = lightsensor_setup(lpi);
    
	//D("xmwuwh [CM3232] 3  %s\n", __func__);
	if (ret < 0)
	{
		pr_err("[LS][CM3232 error]%s: lightsensor_setup error!!\n",
			__func__);
		goto err_lightsensor_setup;
	}

	lpi->als_resolution = 2200;
	lpi->cal_data = 2000000;

	lpi->lp_wq = create_singlethread_workqueue("cm3232_wq");
	if (!lpi->lp_wq)
	{
		pr_err("[CM3232 error]%s: can't create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}
    //D("xmwuwh [CM3232] 4  %s,ret=%d\n", __func__,ret);

	ret = cm3232_setup(lpi);
    
	
	if (ret < 0)
	{
		pr_err("[ERR][CM3232 error]%s: cm3232_setup error!\n", __func__);
		goto err_cm3232_setup;
	}
    //D("xmwuwh [CM3232] 5  %s,ret=%d\n", __func__,ret);

	lpi->cm3232_class = class_create(THIS_MODULE, "optical_sensors");
	if (IS_ERR(lpi->cm3232_class))
	{
		ret = PTR_ERR(lpi->cm3232_class);
		lpi->cm3232_class = NULL;
		goto err_create_class;
	}

	lpi->ls_dev = device_create(lpi->cm3232_class,
				NULL, 0, "%s", "lightsensor");
	if (unlikely(IS_ERR(lpi->ls_dev)))
	{
		ret = PTR_ERR(lpi->ls_dev);
		lpi->ls_dev = NULL;
		goto err_create_ls_device;
	}

	/* register the attributes */
	ret = sysfs_create_group(&lpi->ls_input_dev->dev.kobj,
	&light_attribute_group);
	if (ret)
	{
		pr_err("[LS][CM3232 error]%s: could not create sysfs group\n", __func__);
		goto err_sysfs_create_group_light;
	}

	lpi->als_enable = 0;
	lpi->als_enabled_before_suspend = 0;	
    // xmtdf@ 2014.11.19  create procdir   -- begin
    // Create proc file system
    cm3232_config_proc = proc_create(CM3232_CONFIG_PROC_FILE, 0666, NULL, &config_proc_ops);
    if (cm3232_config_proc == NULL)
    {
        pr_warn("create_proc_entry %s failed\n", CM3232_CONFIG_PROC_FILE);
    }
    else
    {
        pr_warn("create proc entry %s success", CM3232_CONFIG_PROC_FILE);
    }
    hrtimer_init(&cm3232_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    cm3232_timer.function = cm3232_ts_timer_handler;
	D("[CM3232] %s: Probe success!\n", __func__);

	return ret;

err_sysfs_create_group_light:
	device_unregister(lpi->ls_dev);
err_create_ls_device:
	class_destroy(lpi->cm3232_class);
err_create_class:
err_cm3232_setup:
	destroy_workqueue(lpi->lp_wq);
	mutex_destroy(&als_enable_mutex);
	mutex_destroy(&als_disable_mutex);
	mutex_destroy(&als_get_adc_mutex);
	input_unregister_device(lpi->ls_input_dev);
	input_free_device(lpi->ls_input_dev);
err_create_singlethread_workqueue:
  misc_deregister(&lightsensor_misc);
err_lightsensor_setup:
err_platform_data_null:
	kfree(lpi);
	return ret;
}

#ifdef CONFIG_PM
static int cm3232_suspend(struct device *dev)
{
	struct cm3232_info *lpi = lp_info;

	D("[LS][CM3232] %s\n", __func__);

	lpi->als_enabled_before_suspend = lpi->als_enable;
	if (lpi->als_enable)
	{
		lightsensor_disable(lpi);
	}

	return 0;
}

static int cm3232_resume(struct device *dev)
{
	struct cm3232_info *lpi = lp_info;

	D("[LS][CM3232] %s\n", __func__);

	if (lpi->als_enabled_before_suspend)
	{
		lightsensor_enable(lpi);
	}

	return 0;
}
#else
#define cm3232_suspend NULL
#define cm3232_resume NULL
#endif /* CONFIG_PM */
static UNIVERSAL_DEV_PM_OPS(cm3232_pm, cm3232_suspend, cm3232_resume, NULL);

#ifdef CONFIG_ACPI
static struct acpi_device_id cm3232_acpi_match[] = {
	{"CPLM3232", 0},
	{},
};

MODULE_DEVICE_TABLE(acpi, cm3232_acpi_match);
#endif

static const struct i2c_device_id cm3232_i2c_id[] = {
	{CM3232_I2C_NAME, 0},
	{}
};

static struct i2c_driver cm3232_driver = {
	.id_table = cm3232_i2c_id,
	.probe = cm3232_probe,	
	.driver = {
		.name = CM3232_I2C_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_ACPI
		.acpi_match_table = ACPI_PTR(cm3232_acpi_match),
#endif			
	},
};

static int __init cm3232_init(void)
{
    //printk("xmwuwh cm3232_init start!");
	return i2c_add_driver(&cm3232_driver);
}

static void __exit cm3232_exit(void)
{
	i2c_del_driver(&cm3232_driver);
}

module_init(cm3232_init);
module_exit(cm3232_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CM3232 Light Sensor Driver");
MODULE_AUTHOR("Capella Microsystems");


