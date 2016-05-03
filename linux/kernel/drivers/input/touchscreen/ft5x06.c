/*
 * Copyright (C) 2013  wenzengc <wenzeng.chen@intel.com>
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
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/module.h>
#include <linux/ratelimit.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/input/ft5x06.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

static struct i2c_client *this_client;
#define CONFIG_TOUCH_I2C_SPEED    400000       /* Hz */

#define MAX_SUPPORT_POINTS		5
#define EACH_POINT_LEN			6

#define FT5X06_REG_DEVIDE_MODE  	0x00
#define FT5X06_REG_ROW_ADDR             0x01
#define FT5X06_REG_TD_STATUS            0x02
#define FT5X06_REG_START_SCAN           0x02
#define FT5X06_REG_TOUCH_START  	0x03
#define FT5X06_REG_VOLTAGE              0x05
#define FT5X06_REG_CALB			0xA0
#define FT5X06_ID_G_PMODE		0xA5
#define FT5X06_REG_FW_VER     		0xA6
#define FT5X06_ID_G_FT5201ID            0xA8
#define FT5X06_NOISE_FILTER             0xB5
#define FT5X06_REG_POINT_RATE           0x88
#define FT5X06_REG_THGROUP              0x80

#define TOUCH_EVENT_DOWN		0x00
#define TOUCH_EVENT_UP			0x01
#define TOUCH_EVENT_ON			0x02
#define TOUCH_EVENT_RESERVED		0x03

#define MXT_WAKEUP_TIME			10	/*msec */

#define FT_1664S_NAME		"FT1664"
#define FT_3432S_NAME 		"FT3432"
#define FT_NAME				"FTSC1000"

static  int ft5x0x_reset_pin;
static  int ft5x0x_irq_pin;
static int ft5x0x_sysfs_debug;

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5x06_early_suspend(struct early_suspend *es);
static void ft5x06_late_resume(struct early_suspend *es);
#endif

struct ft5x06_ts_data {
	struct i2c_client *client;
	struct input_dev *input;
	u16 num_x;
	u16 num_y;

	struct mutex mutex;
	int threshold;
	int gain;
	int offset;
	int report_rate;
	int fw_version;
#ifdef CONFIG_HAS_EARLYSUSPEND
        struct early_suspend early_suspend;
#endif

};

static int ft5x06_read_reg(struct i2c_client *client, u8 reg)
{
	int ret;
	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
	return ret;
}


static int ft5x06_read_block(struct i2c_client *client, u8 reg,
			     u8 len, u8 * buf)
{
	int ret;
	ret = i2c_smbus_read_i2c_block_data(client, reg, len, buf);
	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
	return ret;
}

static int ft5x06_write_reg(struct i2c_client *client, u8 reg, u8 value)
{
	int ret;
	ret = i2c_smbus_write_byte_data(client, reg, value);
	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);
	return ret;
}




//#define CONFIG_SUPPORT_FTS_CTP_UPG

#ifdef CONFIG_SUPPORT_FTS_CTP_UPG

typedef enum
{
    ERR_OK,
    ERR_MODE,
    ERR_READID,
    ERR_ERASE,
    ERR_STATUS,
    ERR_ECC,
    ERR_DL_ERASE_FAIL,
    ERR_DL_PROGRAM_FAIL,
    ERR_DL_VERIFY_FAIL
}E_UPGRADE_ERR_TYPE;

typedef unsigned char         FTS_BYTE;     //8 bit
typedef unsigned short        FTS_WORD;    //16 bit
typedef unsigned int          FTS_DWRD;    //16 bit
typedef unsigned char         FTS_BOOL;    //8 bit

#define FTS_NULL                0x0
#define FTS_TRUE                0x01
#define FTS_FALSE              0x0

#define I2C_CTPM_ADDRESS       0x70 //no reference!


void delay_qt_ms(unsigned long  w_ms)
{
    unsigned long i;
    unsigned long j;

    for (i = 0; i < w_ms; i++)
    {
    	/* For rk3066 cpu  , 1.5G freq , 855 just 1 ms*/
        for (j = 0; j < 855; j++)
        {
            udelay(1);
        }
    }
}

static int ft5x0x_i2c_rxdata(char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
		},
		{
			.addr	= this_client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
		},
	};

	ret = i2c_transfer(this_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	return ret;
}

static int ft5x0x_i2c_txdata(char *txdata, int length)
{

	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= this_client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
		},
	};


	ret = i2c_transfer(this_client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;

}

FTS_BOOL i2c_read_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;

    ret=i2c_master_recv(this_client, pbt_buf, dw_lenth);

    if(ret<=0)
    {
        printk("[TSP]i2c_read_interface error\n");
        return FTS_FALSE;
    }

    return FTS_TRUE;
}

FTS_BOOL i2c_write_interface(FTS_BYTE bt_ctpm_addr, FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    int ret;
    ret=i2c_master_send(this_client, pbt_buf, dw_lenth);
    if(ret<=0)
    {
        printk("[TSP]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
        return FTS_FALSE;
    }

    return FTS_TRUE;
}

FTS_BOOL cmd_write(FTS_BYTE btcmd,FTS_BYTE btPara1,FTS_BYTE btPara2,FTS_BYTE btPara3,FTS_BYTE num)
{
    FTS_BYTE write_cmd[4] = {0};

    write_cmd[0] = btcmd;
    write_cmd[1] = btPara1;
    write_cmd[2] = btPara2;
    write_cmd[3] = btPara3;
    return i2c_write_interface(I2C_CTPM_ADDRESS, write_cmd, num);
}

FTS_BOOL byte_write(FTS_BYTE* pbt_buf, FTS_DWRD dw_len)
{
    return i2c_write_interface(I2C_CTPM_ADDRESS, pbt_buf, dw_len);
}

FTS_BOOL byte_read(FTS_BYTE* pbt_buf, FTS_BYTE bt_len)
{
    return i2c_read_interface(I2C_CTPM_ADDRESS, pbt_buf, bt_len);
}

#define    FTS_PACKET_LENGTH        128

static unsigned char CTPM_FW[]=
{
  #include "ft5x06_upgrade_app.h"
};

E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    FTS_BYTE reg_val[2] = {0};
    FTS_DWRD i = 0;

    FTS_DWRD  packet_number;
    FTS_DWRD  j;
    FTS_DWRD  temp;
    FTS_DWRD  lenght;
    FTS_BYTE  packet_buf[FTS_PACKET_LENGTH + 6];
    FTS_BYTE  auc_i2c_write_buf[10];
    FTS_BYTE bt_ecc;
    int      i_ret;
    

    unsigned char au_delay_timings[11] = {30, 33, 36, 39, 42, 45, 27, 24,21,18,15};
    j = 0;
    /*********Step 1:Reset  CTPM *****/
    /*write 0xaa to register 0xfc*/
    // ft5x0x_write_reg(0xfc,0xaa);
    //delay_qt_ms(50);
     /*write 0x55 to register 0xfc*/
    //ft5x0x_write_reg(0xfc,0x55);
     //delay_qt_ms(50); 
     // delay_qt_ms(50); 
     
 UPGR_START:
      
    printk("[TSP] Step 1: Reset CTPM test\n");
    gpio_direction_output(ft5x0x_reset_pin, 0);
    delay_qt_ms(50);
    gpio_direction_output(ft5x0x_reset_pin, 1);
   // delay_qt_ms(45); 
    delay_qt_ms(au_delay_timings[j]); 
     // delay_qt_ms(40);

    /*********Step 2:Enter upgrade mode *****/
   printk("[TSP] Step 2: Enter upgrade mode\n");
    auc_i2c_write_buf[0] = 0x55;
    auc_i2c_write_buf[1] = 0xaa;
    do
    {
        i ++;
        i_ret = ft5x0x_i2c_txdata(auc_i2c_write_buf, 2);
        delay_qt_ms(5);
    }while(i_ret <= 0 && i < 5 );

    /*********Step 3:check READ-ID***********************/
    printk("[TSP] Step 3: check READ-ID\n");
    delay_qt_ms(100);  
    cmd_write(0x90,0x00,0x00,0x00,4);
    byte_read(reg_val,2);
    /* if IC is ft5216 , id is 0x79 0x07 ; if ic is ft5606 ,ic is 0x79 0x06*/
    /* if IC if ft5406 , id is 0x79 0x03*/

    if (reg_val[0] == 0x79 && reg_val[1] == 0x03)
    {
        printk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    }
    else
    {
    	if (j < 10)
	{
	     j ++;
	     //msleep(200);
	     printk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
	     printk("[FTS]goto UPGR_START!\n");
	     goto UPGR_START; 
	}
	else
	{	
        printk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    	 gpio_direction_output(ft5x0x_reset_pin, 0);
        delay_qt_ms(50);
        gpio_direction_output(ft5x0x_reset_pin, 1);
        delay_qt_ms(40); 
        return ERR_READID;
	}
        //i_is_new_protocol = 1;
    }

     /*********Step 4:erase app*******************************/
    cmd_write(0x61,0x00,0x00,0x00,1);

    delay_qt_ms(1500);
    printk("[TSP] Step 4: erase. \n");

    /*********Step 5:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;
    printk("[TSP] Step 5: start upgrade. \n");
    dw_lenth = dw_lenth - 8;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
    for (j=0;j<packet_number;j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(lenght>>8);
        packet_buf[5] = (FTS_BYTE)lenght;

        for (i=0;i<FTS_PACKET_LENGTH;i++)
        {
            packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6+i];
        }

        byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
        delay_qt_ms(FTS_PACKET_LENGTH/6 + 1);
        if ((j * FTS_PACKET_LENGTH % 1024) == 0)
        {
              printk("[TSP] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
        }
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;

        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;

        for (i=0;i<temp;i++)
        {
            packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i];
            bt_ecc ^= packet_buf[6+i];
        }

        byte_write(&packet_buf[0],temp+6);
        delay_qt_ms(20);
    }

    //send the last six byte
    for (i = 0; i<6; i++)
    {
        temp = 0x6ffa + i;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        temp =1;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;
        packet_buf[6] = pbt_buf[ dw_lenth + i];
        bt_ecc ^= packet_buf[6];

        byte_write(&packet_buf[0],7);
        delay_qt_ms(20);
    }

    /*********Step 6: read out checksum***********************/
    /*send the opration head*/
    cmd_write(0xcc,0x00,0x00,0x00,1);
    byte_read(reg_val,1);
    printk("[TSP] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
    if(reg_val[0] != bt_ecc)
    {
        gpio_direction_output(ft5x0x_reset_pin, 0);
        delay_qt_ms(50);
        gpio_direction_output(ft5x0x_reset_pin, 1);
        delay_qt_ms(40);      
        return ERR_ECC;
    }

    /*********Step 7: reset the new FW***********************/
    //cmd_write(0x07,0x00,0x00,0x00,1);
    gpio_direction_output(ft5x0x_reset_pin, 0);
    delay_qt_ms(50);
    gpio_direction_output(ft5x0x_reset_pin, 1);
    /*make sure CTP startup normally */
    msleep(300);  
    return ERR_OK;
}

#define FTS_FACTORYMODE_VALUE		0x40
#define FTS_WORKMODE_VALUE		0x00

int fts_ctpm_auto_clb(void)
{
	unsigned char uc_temp = 0x00;
	unsigned char i = 0;

	printk("start auto clb.\n");
	/*start auto CLB */
	msleep(200);

	ft5x06_write_reg(this_client,0, FTS_FACTORYMODE_VALUE);
	/*make sure already enter factory mode */
	msleep(100);
	/*write command to start calibration */
	ft5x06_write_reg(this_client,2, 0x4);
	msleep(300);
	for (i = 0; i < 100; i++) {
		ft5x06_write_reg(this_client,0, &uc_temp);
		/*return to normal mode, calibration finish */
		if (0x0 == ((uc_temp & 0x70) >> 4))
			break;
	}

	//msleep(200);
	/*calibration OK */
	msleep(300);
	ft5x06_write_reg(this_client, 0,FTS_FACTORYMODE_VALUE);	/*goto factory mode for store */
	msleep(100);	/*make sure already enter factory mode */
	ft5x06_write_reg(this_client,2, 0x5);	/*store CLB result */
	msleep(300);
	ft5x06_write_reg(this_client,0, FTS_WORKMODE_VALUE);	/*return to normal mode */
	msleep(300);

	printk("end auto clb.\n");
	/*store CLB result OK */
	return 0;
}


int fts_ctpm_fw_upgrade_with_i_file(void)
{
   FTS_BYTE*     pbt_buf = FTS_NULL;
   int i_ret;

    //=========FW upgrade========================*/
   pbt_buf = CTPM_FW;
   /*call the upgrade function*/
   i_ret =  fts_ctpm_fw_upgrade(pbt_buf,sizeof(CTPM_FW));
   if (i_ret != 0)
   {
       //error handling ...
       //TBD
       printk("TP upgrade fail.\n");
   }
   else
   {
	fts_ctpm_auto_clb();
   }

   return i_ret;
}

unsigned char fts_ctpm_get_upg_ver(void)
{
    unsigned int ui_sz;
    ui_sz = sizeof(CTPM_FW);
    if (ui_sz > 2)
    {
        return CTPM_FW[ui_sz - 2];
    }
    else
    {
        //TBD, error handling?
        return 0xff; //default value
    }
}

#endif

static int init_num_evnet=0;
static int id_num[6]={0};

static irqreturn_t ft5x06_ts_isr(int irq, void *dev_id)
{
	struct ft5x06_ts_data *tsdata = dev_id;
	struct device *dev = &tsdata->client->dev;
	u8 rdbuf[31], number, touchpoint=0;
	int i, type, x, y, id, press;
	int ft_up_num=0;

	memset(rdbuf, 0, sizeof(rdbuf));
	mutex_lock(&tsdata->mutex);
	ft5x06_read_block(tsdata->client, FT5X06_REG_TD_STATUS,	31, rdbuf);

	number = rdbuf[0] & 0x0f;
	if(number >= 5)
		number = 5;
	init_num_evnet = number;

	for (i = 0; i < 5; i++) {
		u8 *buf = &rdbuf[EACH_POINT_LEN * i + 1];
		bool down;

		type = (buf[0] >> 6) & 0x03;
		/* ignore Reserved events */
		if (type == TOUCH_EVENT_RESERVED)
			continue;

		x = ((buf[0] << 8) | buf[1]) & 0x0fff;
		y = ((buf[2] << 8) | buf[3]) & 0x0fff;
		id = (buf[2] >> 4) & 0x0f;
		down = (type != TOUCH_EVENT_UP);

		if(ft5x0x_sysfs_debug==1)
			printk("id = %d..., x value = %d...., y value = %d......number = %d..... down = %d.....\n", id, x, y, number, down);

		if (!down)
			continue;

		id_num[ft_up_num] = id;
		ft_up_num++;
		
		press = 0xff;
		touchpoint ++;

		if(number == 1)
			input_report_key(tsdata->input, BTN_TOUCH, 1);
		input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, press);
		input_report_abs(tsdata->input, ABS_MT_POSITION_X, x);
		input_report_abs(tsdata->input, ABS_MT_POSITION_Y, y);
		input_report_abs(tsdata->input, ABS_MT_WIDTH_MAJOR, 1);
		input_report_abs(tsdata->input, ABS_MT_TRACKING_ID, id);
		input_mt_sync(tsdata->input);
	}

	if(!touchpoint)
	{
		input_report_key(tsdata->input, BTN_TOUCH, 0);
		input_report_abs(tsdata->input, ABS_MT_TOUCH_MAJOR, 0);
	}
	input_sync(tsdata->input);
	mutex_unlock(&tsdata->mutex);

	if(ft5x0x_sysfs_debug==1)
		printk("ft5x06_ts_isr touchpoint=%d,number=%d\n",touchpoint,number);

	return IRQ_HANDLED;
}

static void up_event_fun(struct ft5x06_ts_data *tsdata)
{
	int i;
	for (i = 0; i < init_num_evnet; i++) {
		input_mt_slot(tsdata->input, id_num[i]);
		input_mt_report_slot_state(tsdata->input, MT_TOOL_FINGER, 0);
	}
	input_report_key(tsdata->input, BTN_TOUCH, 0);
	input_sync(tsdata->input);
	init_num_evnet = 0;
}

static ssize_t ft5x06_thrsh_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ft5x06_ts_data *tsdata = i2c_get_clientdata(client);
	u8 value;
	u8 rdbuf[30] = {0};
	size_t count = 0;
	int i=0;

	mutex_lock(&tsdata->mutex);
	value = ft5x06_read_reg(client, FT5X06_REG_THGROUP);
	count = scnprintf(buf, PAGE_SIZE, "%x\n", value);

	mutex_unlock(&tsdata->mutex);

	return count;
}

static ssize_t ft5x06_rate_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ft5x06_ts_data *tsdata = i2c_get_clientdata(client);
	u8 value;
	size_t count = 0;

	mutex_lock(&tsdata->mutex);
	value = ft5x06_read_reg(client, FT5X06_REG_POINT_RATE);
	count = scnprintf(buf, PAGE_SIZE, "%x\n", value);
	mutex_unlock(&tsdata->mutex);

	return count;

}

static ssize_t ft5x06_rawdata_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{

}

static ssize_t ft5x06_thrsh_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ft5x06_ts_data *tsdata = i2c_get_clientdata(client);
	u8 value;
	int error;

	mutex_lock(&tsdata->mutex);
	error = kstrtouint(buf, 0, &value);
	if (error)
		goto out;

	ft5x06_write_reg(client, FT5X06_REG_THGROUP, value);

out:
	mutex_unlock(&tsdata->mutex);
	return error ? : count;

}

static ssize_t ft5x06_rate_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ft5x06_ts_data *tsdata = i2c_get_clientdata(client);
	u8 value;
	int error;

	mutex_lock(&tsdata->mutex);
	error = kstrtouint(buf, 0, &value);
	if (error)
		goto out;

	ft5x06_write_reg(client, FT5X06_REG_POINT_RATE, value);
out:
	mutex_unlock(&tsdata->mutex);
	return error ? : count;

}

static ssize_t ft5x06_ftdebug_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
   ft5x0x_sysfs_debug = 1;
   return 0;
}


/* sysfs */
static DEVICE_ATTR(threshold, 0644, ft5x06_thrsh_show, ft5x06_thrsh_store);
static DEVICE_ATTR(report_rate, 0644, ft5x06_rate_show, ft5x06_rate_store);
static DEVICE_ATTR(rawdata, 0644, ft5x06_rawdata_show, NULL);
static DEVICE_ATTR(ftdebug, 0644, ft5x06_ftdebug_show, NULL);

static struct attribute *ft5x06_attrs[] = {
	&dev_attr_threshold.attr,
	&dev_attr_report_rate.attr,
	&dev_attr_rawdata.attr,
	&dev_attr_ftdebug.attr,
	NULL
};

static const struct attribute_group ft5x06_attr_group = {
	.attrs = ft5x06_attrs
};

static int ft5x06_config(struct ft5x06_ts_data *tsdata)
{
	/* enable auto calibration */
	ft5x06_write_reg(tsdata->client, FT5X06_REG_CALB, 0x00);
	msleep(100);
	if (tsdata->threshold)
		ft5x06_write_reg(tsdata->client,
				 FT5X06_REG_THGROUP, tsdata->threshold);
	if (tsdata->report_rate)
		ft5x06_write_reg(tsdata->client,
				 FT5X06_REG_POINT_RATE, tsdata->report_rate);
	return 0;
}

static int ft5x06_ts_reset(struct i2c_client *client, int reset_pin)
{
	int error;

	if (gpio_is_valid(reset_pin)) {
		/* this pulls reset down, enabling the low active reset */
		error = gpio_request_one(reset_pin, GPIOF_OUT_INIT_LOW,
					 "ft5x06 reset");
		if (error) {
			dev_err(&client->dev,
				"Failed to request GPIO %d as reset pin, error %d\n",
				reset_pin, error);
			return error;
		}

		msleep(10);
		gpio_set_value(reset_pin, 1);
		msleep(20);
		gpio_free(reset_pin);
	}

	return 0;
}
static int ft5x06_get_defaults(struct ft5x06_ts_data *tsdata)
{
	u8 version = 0, threshold = 0, rate = 0;

	threshold = ft5x06_read_reg(tsdata->client, FT5X06_REG_THGROUP);
	rate = ft5x06_read_reg(tsdata->client, FT5X06_REG_POINT_RATE);
	version = ft5x06_read_reg(tsdata->client, FT5X06_REG_FW_VER);
	tsdata->fw_version = version;

	dev_info(&tsdata->client->dev,
		 "Rev.%02x, Thold.%x, Rate.%x\n", version, threshold, rate);

	if (threshold < 0 || rate < 0 || version < 0)
        return -1;
	else
        return 0;
}

static unsigned char ft5x0x_read_fw_ver(struct ft5x06_ts_data *tsdata)
{
	unsigned char ver;
	int ret;
	
	ver = ft5x06_read_reg(tsdata->client,FT5X06_REG_FW_VER);
	printk("FireWare version is [%x]\n",ver);
	ret=ver;
	if (  ret < 0)
	   ver  =  0xff ;
	return(ver);
}

static int ft5x06_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct ft5x06_ts_data *tsdata;
	struct input_dev *input;
	int error;
	unsigned char uc_reg_value;
	dev_info(&client->dev, "ft5x06....probe......\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		error = -ENODEV;
		goto exit_check_functionality_failed;
	}

#ifdef CONFIG_MRD8
	client->addr = 0x3E;
#else
	client->addr = 0x38;
#endif

	ft5x0x_reset_pin = FT5X06_RESET_PIN;
	ft5x0x_irq_pin = FT5X06_INT_PIN;
	error = ft5x06_ts_reset(client, ft5x0x_reset_pin);
	if (error)
		return error;
	msleep(100);
	
	if (gpio_is_valid(ft5x0x_irq_pin)) {
		error = gpio_request_one(ft5x0x_irq_pin,
					 GPIOF_IN, "ft5x06 irq");
		if (error) {
			dev_err(&client->dev,
				"Failed to request GPIO %d, error %d\n",
				ft5x0x_irq_pin, error);
			return error;
		}
	}

	tsdata = kzalloc(sizeof(*tsdata), GFP_KERNEL);
	input = input_allocate_device();
	if (!tsdata || !input) {
		dev_err(&client->dev, "failed to allocate driver data.\n");
		error = -ENOMEM;
		goto err_free_mem;
	}
      this_client = client;
	mutex_init(&tsdata->mutex);
	tsdata->client = client;
	tsdata->input = input;
	tsdata->num_x = X_MAX;
	tsdata->num_y = Y_MAX;
	tsdata->threshold = 0;
	tsdata->report_rate = 0;

	error = ft5x06_get_defaults(tsdata);
	if (error)
		goto err_free_mem;
	
#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
	gpio_request(ft5x0x_reset_pin, "NULL");
	uc_reg_value = ft5x0x_read_fw_ver(tsdata);
	if( uc_reg_value == 0xff)   /* unvalid version */
	{		
		gpio_free(ft5x0x_reset_pin);
		goto  err_free_mem;
	}
        if( uc_reg_value != 0xff )
	{
          /* compare to last version, if not equal,do update!*/        		   
              if((uc_reg_value < 0x14)&& ( uc_reg_value > 0x10))
              {
        	  	fts_ctpm_fw_upgrade_with_i_file();
              }	
              if( uc_reg_value == 0xa6 )
              {
                    fts_ctpm_fw_upgrade_with_i_file();	
              }	
		gpio_free(ft5x0x_reset_pin);
		uc_reg_value = ft5x0x_read_fw_ver(tsdata);
        }	
#endif

	input->name = client->name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;
#if 1
	set_bit(ABS_MT_TOUCH_MAJOR, input->absbit);
	set_bit(ABS_MT_POSITION_X, input->absbit);
	set_bit(ABS_MT_POSITION_Y, input->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input->absbit);
	set_bit(ABS_MT_TRACKING_ID, input->absbit);

	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, 10, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, tsdata->num_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, tsdata->num_y, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 0xFF, 0, 0);
	input_set_abs_params(input,	 ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);

	set_bit(BTN_TOUCH, input->keybit);
	set_bit(EV_ABS, input->evbit);
	set_bit(EV_KEY, input->evbit);
	set_bit(INPUT_PROP_DIRECT, input->propbit);
#else
	ft5x06_config(tsdata);

	__set_bit(EV_KEY, input->evbit);
	__set_bit(EV_ABS, input->evbit);
	__set_bit(BTN_TOUCH, input->keybit);
	input_mt_init_slots(input, MAX_SUPPORT_POINTS, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, tsdata->num_x, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, tsdata->num_y, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, 255, 0, 0);
#endif

	input_set_drvdata(input, tsdata);
	i2c_set_clientdata(client, tsdata);

	client->irq = gpio_to_irq(ft5x0x_irq_pin);
	dev_info(&client->dev, "ft5x06 client irq = %d......\n", client->irq);
	gpio_direction_input(ft5x0x_irq_pin);
	error = request_threaded_irq(client->irq, NULL, ft5x06_ts_isr,
				     IRQF_TRIGGER_FALLING |IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				     client->name, tsdata);
	if (error) {
		dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
		goto err_free_mem;
	}

	error = input_register_device(input);
	if (error)
		goto err_free_irq;

	device_init_wakeup(&client->dev, 1);

	error = sysfs_create_group(&client->dev.kobj, &ft5x06_attr_group);
	if (error) {
		dev_err(&client->dev, "fail to export sysfs entires\n");
		goto err_free_irq;
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
        tsdata->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
        tsdata->early_suspend.suspend = ft5x06_early_suspend;
        tsdata->early_suspend.resume = ft5x06_late_resume;
        register_early_suspend(&tsdata->early_suspend);
#endif

	dev_info(&client->dev,
		 "EDT FT5x06 initialized: IRQ pin %d, Reset pin %d.\n",
		 ft5x0x_irq_pin, ft5x0x_reset_pin);

	return 0;

err_free_irq:
	free_irq(client->irq, tsdata);
err_free_mem:
	input_free_device(input);
	kfree(tsdata);

	if (gpio_is_valid(ft5x0x_irq_pin))
		gpio_free(ft5x0x_irq_pin);
exit_check_functionality_failed:

	return error;
}

static int ft5x06_ts_remove(struct i2c_client *client)
{
	struct ft5x06_ts_data *tsdata = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &ft5x06_attr_group);
	free_irq(client->irq, tsdata);
	input_unregister_device(tsdata->input);

	if (gpio_is_valid(ft5x0x_irq_pin))
		gpio_free(ft5x0x_irq_pin);
	kfree(tsdata);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int ft5x06_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ft5x06_ts_data *tsdata = i2c_get_clientdata(client);

	mutex_lock(&tsdata->mutex);
	disable_irq(client->irq);
	/*set to hibernate mode. */
	ft5x06_write_reg(client, FT5X06_ID_G_PMODE, 0x03);
//	up_event_fun(tsdata);
	
	mutex_unlock(&tsdata->mutex);
	return 0;
}

static int ft5x06_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	/*reset to enter active mode. */
	ft5x06_ts_reset(client, ft5x0x_reset_pin);
	enable_irq(client->irq);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(ft5x06_ts_pm_ops, ft5x06_ts_suspend, ft5x06_ts_resume);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5x06_early_suspend(struct early_suspend *es)
{
	struct ft5x06_ts_data *tsdata;
	tsdata =container_of(es, struct ft5x06_ts_data, early_suspend);

	disable_irq(tsdata->client->irq);
	mutex_lock(&tsdata->mutex);
	/*set to hibernate mode. */
	ft5x06_write_reg(tsdata->client, FT5X06_ID_G_PMODE, 0x03);
//	up_event_fun(tsdata);
	
    mutex_unlock(&tsdata->mutex);
}

static void ft5x06_late_resume(struct early_suspend *es)
{
	struct ft5x06_ts_data *tsdata;

	tsdata =container_of(es, struct ft5x06_ts_data, early_suspend);

	/*reset to enter active mode */
	ft5x06_ts_reset(tsdata->client, ft5x0x_reset_pin);
	enable_irq(tsdata->client->irq);
}
#endif

static const struct i2c_device_id ft5x06_ts_id[] = {
	{"ft5x06", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ft5x06_ts_id);

static struct acpi_device_id ft5x06_acpi_match[] = {
	{FT_1664S_NAME, 0},
	{FT_3432S_NAME, 0},
	{FT_NAME, 0},
	{},
};

static struct i2c_driver ft5x06_ts_driver = {
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "ft5x06",
		   .pm = &ft5x06_ts_pm_ops,
		   .acpi_match_table = ACPI_PTR(ft5x06_acpi_match),
		   },
	.id_table = ft5x06_ts_id,
	.probe = ft5x06_ts_probe,
	.remove = ft5x06_ts_remove,
};

static int __init ft5x06_init(void)
{
	int ret;
	printk("==ft5x06_init==\n");
	ret = i2c_add_driver(&ft5x06_ts_driver);
	if (ret)
		printk(KERN_ERR "Unable to register ft5x06 i2c driver\n");

	return ret;
}

late_initcall(ft5x06_init);

static void __exit ft5x06_exit(void)
{
	i2c_del_driver(&ft5x06_ts_driver);
}

module_exit(ft5x06_exit);

MODULE_AUTHOR("Wenzeng Chen <wenzeng.chen@intel.com>");
MODULE_DESCRIPTION("FT5x06 I2C Touchscreen Driver");
MODULE_LICENSE("GPL");
