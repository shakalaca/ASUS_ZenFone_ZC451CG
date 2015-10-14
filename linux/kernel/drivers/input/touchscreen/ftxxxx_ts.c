/* drivers/input/touchscreen/ftxxxx_ts.c
*
* FocalTech ftxxxx TouchScreen driver.
*
* Copyright (c) 2014  Focaltech Ltd.
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
//#define DEBUG
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
//#include <mach/irqs.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/input/mt.h>
//add for switch dev
#include <linux/switch.h>
#include "ftxxxx_ts.h"

//#include <mach/gpio.h>
//#include <mach/map.h>
//#include <mach/regs-clock.h>
//#include <mach/regs-gpio.h>
//#include <plat/gpio-cfg.h>
#include <linux/gpio.h>

#define SYSFS_DEBUG
#define FTS_APK_DEBUG
#define FTS_CTL_IIC 
//#define FTS_GESTRUE 
//#define FTXXXX_ENABLE_IRQ 

#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
#endif

#ifdef FTS_GESTRUE
#include "ft_gesture_lib.h"
short pointnum = 0;
int gestrue_id = 0;
#endif

#ifdef SYSFS_DEBUG
#include "ftxxxx_ex_fun.h"
#endif

static int ft_fac_mode = 0; //0: out factory mode;1: in factory mode;
static int ft_glove_mode = 0; // 0: out glove mode;1 : in glove mode;
//add for switch dev
struct switch_dev touch_dev;
u8 fw;

struct ts_event {
	u16 au16_x[CFG_MAX_TOUCH_POINTS];	/*x coordinate */
	u16 au16_y[CFG_MAX_TOUCH_POINTS];	/*y coordinate */
	u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];	/*touch event:
												0 -- down; 1-- contact; 2 -- contact */
	u8 au8_finger_id[CFG_MAX_TOUCH_POINTS];	/*touch ID */
	u16 pressure;
	u8 touch_point;
	u8 Cur_touchpoint;   //	add by Lvyong	
};

struct ftxxxx_ts_data {
	unsigned int irq;
	unsigned int x_max;
	unsigned int y_max;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ts_event event;
	struct ftxxxx_platform_data *pdata;
	struct early_suspend early_suspend;
};

//#define TOUCH_MAX_X						0x700
//#define TOUCH_MAX_Y						0x400

#define ANDROID_INPUT_PROTOCOL_B

//#define FTXXXX_RESET_PIN	EXYNOS4_GPK3(3)//EXYNOS4X12_GPM0(1)//EXYNOS4_GPM0(1) //S5PV210_GPB(2)
//#define FTXXXX_RESET_PIN_NAME	"ftxxxx-reset"

//add by red_zhang@asus.com
static void ftxxxx_ts_suspend(struct early_suspend *handler);
static void ftxxxx_ts_resume(struct early_suspend *handler);
static struct mutex i2c_rw_access;
/*
*ftxxxx_i2c_Read-read data and write data by i2c
*@client: handle of i2c
*@writebuf: Data that will be written to the slave
*@writelen: How many bytes to write
*@readbuf: Where to store data read from slave
*@readlen: How many bytes to read
*
*Returns negative errno, else the number of messages executed
*
*
*/
int ftxxxx_i2c_Read(struct i2c_client *client, char *writebuf,
	int writelen, char *readbuf, int readlen)
{
	int ret;
	mutex_lock(&i2c_rw_access);//zax 20141219
	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = 0,
				.len = writelen,
				.buf = writebuf,
			},
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "[first]%s: i2c read error.\n", __func__);
	} else {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "[second]%s:i2c read error.\n", __func__);
	}
	mutex_unlock(&i2c_rw_access);//zax 20141219
	return ret;
}
/*write data by i2c*/
int ftxxxx_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;
	mutex_lock(&i2c_rw_access);//zax 20141219
	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = writelen,
			.buf = writebuf,
		},
	};

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s i2c write error.\n", __func__);
	mutex_unlock(&i2c_rw_access);//zax 20141219
	return ret;
}

#ifdef FTS_GESTRUE
static int ftxxxx_read_Touchdata(struct ftxxxx_ts_data *data)
{
	unsigned char buf[FTS_GESTRUE_POINTS * 2] = { 0 };   
	int ret = -1;    
	int i = 0;    
	buf[0] = 0xd3; 

	ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, 8);    
	if (ret < 0) 
	{        
		dev_err(&data->client->dev, "%s read touchdata failed.\n", __func__);        
		return ret;    
	} 

	if (0x24 == buf[0])
	{        
		gestrue_id = 0x24;
		printk(KERN_WARNING "The Gesture ID is %x %x \n",gestrue_id,pointnum);
		return -1;   
	}

	pointnum = (short)(buf[1]) & 0xff;

	buf[0] = 0xd3;

	ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, (pointnum * 4 + 8));    
	/* Read two times*/
	//ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, (8));    
	//ret = ftxxxx_i2c_Read(data->client, buf, 0, (buf+8), (8));    

	if (ret < 0) 
	{        
		dev_err(&data->client->dev, "%s read touchdata failed.\n", __func__);        
		return ret;    
	}

	gestrue_id = fetch_object_sample(buf,pointnum);

	printk(KERN_WARNING "The Gesture ID is %x %x \n",gestrue_id,pointnum);

	for(i = 0;i < pointnum;i++)
	{
		coordinate_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
			8 | (((s16) buf[1 + (4 * i)])& 0xFF);
		coordinate_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
			8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
	} 

	return -1;
}
#else

#define PRINTF_TOUCHDATA_EN   1//打印原始数据使能。

#if PRINTF_TOUCHDATA_EN == 1

#define TOUCHDATA_GESTURE_ID      1  //gesture ID
#define TOUCHDATA_POINT_NUM_ID    2  //point num

/*****************************************************************************************
Name      :    void Ft_Printf_Touchdata(struct ftxxxx_ts_data *data,u8* readbuf)

Input     :    (struct ftxxxx_ts_data *data,u8* readbuf)

Output    :    none

function  :    1 TP在需要使用打印保存功能的时候，可以在上报数据的时候，在手势字节里面填写(0XF0|N)
               一定要确保填写的(0XF0|N)和目前在用的关键字不一样。且 N<= 0xf.  N记录的意思为:除了
               报点数据外还需要继续读取其他数据的个数(N*FT_TOUCH_STEP). TP在配置N的时候要确保不
               影响正常读数据，避免读数据太多而丢包。
               
               
               2当TP配置为需要打印保存数据的时候，驱动会将读到的数据记录保存下来，可以通过 LOG的
               方式或文本的方式查看到原始数据。


******************************************************************************************/
static int FrameCnt = 0;

int  Ft_Printf_Touchdata(struct ftxxxx_ts_data *data, u8* readbuf)
{
	u8 buf[FT_TOUCH_STEP * 0x0f] = { 0 };
	int ret = -1;
	int i = 0;
	u8 temp;

	temp = readbuf[TOUCHDATA_GESTURE_ID];//获取手势字节位

	if((temp & 0xf0)!=0xf0) //判断是否需要开启打印和保存的功能
	{
		return 0;
	}
	else
	{
	       FrameCnt++;
		//记录保存 读到的原始数据 readbuf[ POINT_READ_BUF] 
		printk("[FTS Touchdata:%5d] ", FrameCnt);
		for(i=0; i<POINT_READ_BUF; i++)
	     	{
	     		printk("%02X ", readbuf[i]);
	     	}
		printk("\n");
	}

	temp &= 0xf; //判断是否还需要读取多少个点的数据。

	if(temp == 0) 
	{
	    printk("\n\n\n");
	    return 0;
	}
	else
	    printk("\n");

	buf[0] = POINT_READ_BUF;
    
	ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, (FT_TOUCH_STEP * temp));

	if (ret < 0) {
		dev_err(&data->client->dev, "%s Second read touchdata failed.\n",
			__func__);
		return ret;
	}
	else
	{
		//记录保存 读到的原始数据 buf[ POINT_READ_BUF] 
		printk("[FTS Touchdata Ex:%5d] ", FrameCnt);		
		for(i=0; i<(FT_TOUCH_STEP * temp); i++)
	     	{
	     		printk("%02X ", buf[i]);
	     	}
		printk("\n\n\n\n");
	}

}
#else
#define Ft_Printf_Touchdata(x,y)
#endif

/*Read touch point information when the interrupt  is asserted.*/
static int ftxxxx_read_Touchdata(struct ftxxxx_ts_data *data)
{
	struct ts_event *event = &data->event;
	u8 buf[POINT_READ_BUF] = { 0 };
	int ret = -1;
	int i = 0;
	u8 pointid = FT_MAX_ID;

	ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) {
		dev_err(&data->client->dev, "%s read touchdata failed.\n",
			__func__);
		return ret;
	}

	Ft_Printf_Touchdata(data,buf); //打印报点调试信息
	
	memset(event, 0, sizeof(struct ts_event));
       event->Cur_touchpoint =buf[2]&0x0F;  // add by Lvyong
	event->touch_point = 0;
	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++) {
		pointid = (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		if (pointid >= FT_MAX_ID)
			break;
		else
			event->touch_point++;
		event->au16_x[i] =
			(((s16) buf[FT_TOUCH_X_H_POS + FT_TOUCH_STEP * i]) & 0x0F) <<
			8 | (((s16) buf[FT_TOUCH_X_L_POS + FT_TOUCH_STEP * i])& 0xFF);
		event->au16_y[i] =
			(((s16) buf[FT_TOUCH_Y_H_POS + FT_TOUCH_STEP * i]) & 0x0F) <<
			8 | (((s16) buf[FT_TOUCH_Y_L_POS + FT_TOUCH_STEP * i]) & 0xFF);
		event->au8_touch_event[i] =
			buf[FT_TOUCH_EVENT_POS + FT_TOUCH_STEP * i] >> 6;
		if((event->au8_touch_event[i]==0 || event->au8_touch_event[i]==2)&&(event->Cur_touchpoint==0))//20141205
				return 1;//20141205
		event->au8_finger_id[i] =
			(buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
#if 0
		pr_info("id=%d event=%d x=%d y=%d\n", event->au8_finger_id[i],
			event->au8_touch_event[i], event->au16_x[i], event->au16_y[i]);
#endif
	}

	event->pressure = FT_PRESS;

	return 0;
}
#endif

/*
*report the point information
*/
static void ftxxxx_report_value(struct ftxxxx_ts_data *data)
{	
	struct ts_event *event = &data->event;
	int i;
	int j;
	int uppoint = 0;
	static u8 last_touchpoint=0; //add by Lvyong;
	//int ret = 0;

	for (i = 0; i < event->touch_point; i++)
	{
		//pr_info("[ftxxxx] event= %d, x= %d, y= %d, point= %d\n", event->au8_touch_event[i], event->au16_x[i], event->au16_y[i], event->touch_point);

		if (event->au8_touch_event[i]== 0 || event->au8_touch_event[i] == 2)
		{
			if (event->au16_x[i] == 58 && event->au16_y[i] == 900)
			{
	           for (j = 0; j < CFG_MAX_TOUCH_POINTS; j++) 
		       {
		           input_mt_slot(data->input_dev, j);
		           input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	           }
	           if(last_touchpoint == 0)
	           {		
				input_report_key(data->input_dev, KEY_BACK, 1);
				//input_mt_sync(data->input_dev);
				input_sync(data->input_dev);
				dev_dbg(&data->client->dev, "[ftxxxx] Press BACK KEY\n");
				}	
				return;
			}
			if (event->au16_x[i] == 266 && event->au16_y[i] == 900)
			{
				 for (j = 0; j < CFG_MAX_TOUCH_POINTS; j++) 
		         {
		            input_mt_slot(data->input_dev, j);
		            input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	             }
	             if(last_touchpoint == 0)
	             {
				  input_report_key(data->input_dev, KEY_HOME, 1);
			      //input_mt_sync(data->input_dev);
				  input_sync(data->input_dev);
				  dev_dbg(&data->client->dev, "[ftxxxx] Press HOME KEY\n");
				 }
				return;
			}
			if (event->au16_x[i] == 420 && event->au16_y[i] == 900)
			{
				 for (j = 0; j < CFG_MAX_TOUCH_POINTS; j++) 
		         {
		            input_mt_slot(data->input_dev, j);
		            input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	             }
	             if(last_touchpoint == 0)
	             {
				  input_report_key(data->input_dev, KEY_MENU, 1);
				  //input_mt_sync(data->input_dev);
				  input_sync(data->input_dev);
				  dev_dbg(&data->client->dev, "[ftxxxx] Press MENU KEY\n");
				 }
				return;
			}
		}
		else
		{
			last_touchpoint = 0;
			if (event->au16_x[i] == 58 && event->au16_y[i] == 900)
			{
				input_report_key(data->input_dev, KEY_BACK, 0);
				//input_mt_sync(data->input_dev);
				input_sync(data->input_dev);
//				printk("<Red_debug> Press BACK KEY\n");
				return;
			}
			if (event->au16_x[i] == 266 && event->au16_y[i] == 900)
			{
				input_report_key(data->input_dev, KEY_HOME, 0);
				//input_mt_sync(data->input_dev);
				input_sync(data->input_dev);
//				printk("<Red_debug> Press HOME KEY\n");
				return;
			}
			if (event->au16_x[i] == 420 && event->au16_y[i] == 900)
			{
				input_report_key(data->input_dev, KEY_MENU, 0);
				//input_mt_sync(data->input_dev);
				input_sync(data->input_dev);
//				printk("<Red_debug> Press MENU KEY\n");
				return;
			}
		}
	}

	//protocol B
	for (i = 0; i < event->touch_point; i++)
	{
//		printk("[ftxxxx] event= %d, x= %d, y= %d, point= %d\n", event->au8_touch_event[i], event->au16_x[i], event->au16_y[i], event->touch_point);
//		printk("[ftxxxx] finger_id=%d, event->pressure= %d, uppoint= %d\n", event->au8_finger_id[i], event->pressure, uppoint);

		input_mt_slot(data->input_dev, event->au8_finger_id[i]);

		if (event->au8_touch_event[i]== 0 || event->au8_touch_event[i] == 2)
		{
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);
//			input_report_abs(data->input_dev, ABS_MT_PRESSURE, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->au16_x[i]);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->au16_y[i]);			
		}
		else
		{
			uppoint++;
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
		}
	}

	 if((last_touchpoint>0)&&(event->Cur_touchpoint==0))    // add by lvyong
	{	/* release all touches */ 
		for (j = 0; j < CFG_MAX_TOUCH_POINTS; j++) 
		{
			input_mt_slot(data->input_dev, j);
			input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
		}
			last_touchpoint=0;
    } 
	if(event->touch_point == uppoint)
	{
		input_report_key(data->input_dev, BTN_TOUCH, 0);		
	}
	else
		input_report_key(data->input_dev, BTN_TOUCH, event->touch_point > 0);
	
	input_sync(data->input_dev);
       last_touchpoint=event->Cur_touchpoint ; 	//add by Lvyong
}

/*The ftxxxx device will signal the host about TRIGGER_FALLING.
*Processed when the interrupt is asserted.
*/
static irqreturn_t ftxxxx_ts_interrupt(int irq, void *dev_id)
{
	struct ftxxxx_ts_data *ftxxxx_ts = dev_id;
	int ret = 0;
	disable_irq_nosync(ftxxxx_ts->client->irq);
	
//	printk("<Red_debug> enter %s",__func__);

	ret = ftxxxx_read_Touchdata(ftxxxx_ts);
	if (ret == 0)
		ftxxxx_report_value(ftxxxx_ts);
	
	enable_irq(ftxxxx_ts->client->irq);

	return IRQ_HANDLED;
}

void ftxxxx_reset_tp(int HighOrLow)
{
	pr_info("set tp reset pin to %d\n", HighOrLow);	
	gpio_set_value(FTXXXX_RESET_PIN, HighOrLow);
}

void ftxxxx_Enable_IRQ(struct i2c_client *client, int enable)
{
	//if (FTXXXX_ENABLE_IRQ == enable)
	//if (FTXXXX_ENABLE_IRQ)
	//enable_irq(client->irq);
	//else
	//disable_irq_nosync(client->irq);
}

static int fts_init_gpio_hw(struct ftxxxx_ts_data *ftxxxx_ts)
{
	int ret = 0;

	ret = gpio_request(FTXXXX_RESET_PIN, FTXXXX_RESET_PIN_NAME);
	if (ret) {
		pr_err("%s: request GPIO %s for reset failed, ret = %d\n",
			__func__, FTXXXX_RESET_PIN_NAME, ret);
		return ret;
	}
		
	//s3c_gpio_setpull(FTXXXX_RESET_PIN, S3C_GPIO_PULL_NONE);  
	//s3c_gpio_cfgpin(FTXXXX_RESET_PIN, S3C_GPIO_SFN(1));//  S3C_GPIO_OUTPUT
	//gpio_set_value(FTXXXX_RESET_PIN,  0);
	gpio_direction_output(FTXXXX_RESET_PIN, 1);	//reset set high
	pr_info("[ftxxxx] gpio_request reset gpio Num: %d\n", FTXXXX_RESET_PIN);

	ret = gpio_request(FTXXXX_INT_PIN, FTXXXX_INT_PIN_NAME);
	if (ret) {
		pr_err("%s: request GPIO %s for reset failed, ret = %d\n",
			__func__, FTXXXX_INT_PIN_NAME, ret);
		return ret;
	}
	gpio_direction_input(FTXXXX_INT_PIN);
	pr_info("[ftxxxx] gpio_request int gpio Num: %d\n", FTXXXX_INT_PIN);

	return ret;
}

static void fts_un_init_gpio_hw(struct ftxxxx_ts_data *ftxxxx_ts)
{
	gpio_free(FTXXXX_INT_PIN);
	gpio_free(FTXXXX_RESET_PIN);
}

/*add for factory disable/enable tp*/
/****************************************************/
static ssize_t ftxxxx_ftdetp_store(struct device *dev,
										struct device_attribute *attr,
										const char *buf, size_t count)
{
	struct ftxxxx_ts_data *ftxxxx_ts = dev_get_drvdata(dev);
	u32 cmd=1; //0:disable tp;1:enable tp;

	printk("[ftxxxx]enter %s\n",__func__);
	cmd = simple_strtoul(buf,NULL,10);
	if(cmd==0)	// disable tp by set sleep command
	{
		ftxxxx_write_reg(ftxxxx_ts->client,0xa5,0x03);
		ft_fac_mode = 1;
	}
	else
	{
		gpio_request(162,"ts_rst");
		gpio_direction_output(162,0);
		msleep(20);
		gpio_direction_output(162,1);
		ft_fac_mode = 0;
	}
	return count;
}
/****************************************************/
//add for ft glove mode
#if FT_GLOVE_ON
/****************************************************/
static ssize_t ftxxxx_ftglove_store(struct device *dev,
										struct device_attribute *attr,
										const char *buf, size_t count)
{
	struct ftxxxx_ts_data *ftxxxx_ts = dev_get_drvdata(dev);
	u32 cmd=0; //0:disable glove mode;1:enable glove mode;
	cmd = simple_strtoul(buf,NULL,10);
	if(cmd==0)	
	{
		ftxxxx_write_reg(ftxxxx_ts->client,0xc0,0x00);
	}
	else
	{
		ftxxxx_write_reg(ftxxxx_ts->client,0xc0,0x01);
	}
	return count;
}

static ssize_t ftxxxx_ftglove_show(struct device *dev,
struct device_attribute *attr, char *buf)
{
	struct ftxxxx_ts_data *ftxxxx_ts = dev_get_drvdata(dev);
	u8 regvalue;
	u8 ret;
	
	ret = ftxxxx_read_reg(ftxxxx_ts->client,0xc0,&regvalue);
	if(ret < 0)
		return scnprintf(buf, PAGE_SIZE, "read glove mode fail\n");
	else
		return scnprintf(buf, PAGE_SIZE, "%d\n", regvalue);
}
/****************************************************/
#endif

static DEVICE_ATTR(ftdetp, S_IRUGO|S_IWUSR, NULL, ftxxxx_ftdetp_store);
#if FT_GLOVE_ON
static DEVICE_ATTR(ftglove, S_IRUGO|S_IWUSR, ftxxxx_ftglove_show, ftxxxx_ftglove_store);
#endif

static struct attribute *ft_attrs[] = {
#ifdef CONFIG_FACTORY_ITEMS
    &dev_attr_ftdetp.attr,
#endif
//add for ft glove mode
#if FT_GLOVE_ON
	&dev_attr_ftglove.attr,
#endif 
    NULL
};

static const struct attribute_group ft_attr_group = {
    .attrs = ft_attrs,
};
/*********************************************/
/*********************************************/
//add for switch dev
static ssize_t touch_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "0x%x\n",fw);
}
/*********************************************/

static int ftxxxx_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ftxxxx_platform_data *pdata = (struct ftxxxx_platform_data *)client->dev.platform_data;
	struct ftxxxx_ts_data *ftxxxx_ts;
	struct input_dev *input_dev;
	int err = 0;
	unsigned char uc_reg_value;
	unsigned char uc_reg_addr;	
	int flag=0;	

	printk("[Red_debug][ftxxxx] %s start\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	ftxxxx_ts = kzalloc(sizeof(struct ftxxxx_ts_data), GFP_KERNEL);

	if (!ftxxxx_ts) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	i2c_set_clientdata(client, ftxxxx_ts);

	ftxxxx_ts->irq = FTXXXX_INT_PIN;
	ftxxxx_ts->client = client;
	ftxxxx_ts->pdata = pdata;
	ftxxxx_ts->x_max = TOUCH_MAX_X - 1;
	ftxxxx_ts->y_max = TOUCH_MAX_Y - 1;
	ftxxxx_ts->pdata->gpio_reset = FTXXXX_RESET_PIN;
	ftxxxx_ts->pdata->gpio_irq = ftxxxx_ts->irq;
	client->irq = gpio_to_irq(ftxxxx_ts->irq);
	//client->irq = IRQ_EINT(13);
	
	pr_info("[ftxxxx] irq = %d\n", client->irq);

	if(fts_init_gpio_hw(ftxxxx_ts)<0)
		goto exit_init_gpio;	
#ifdef CONFIG_PM
	printk("[ftxxxx] config_pm exit\n");
#if 0
	err = gpio_request(pdata->gpio_reset, "ftxxxx reset");
	if (err < 0) {
		dev_err(&client->dev, "%s:failed to set gpio reset.\n",
			__func__);
		goto exit_request_reset;
	}
#endif
#endif
	mutex_init(&i2c_rw_access);
	err = request_threaded_irq(client->irq, NULL, ftxxxx_ts_interrupt,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT, client->dev.driver->name,
		ftxxxx_ts);
	if (err < 0) {
		dev_err(&client->dev, "ftxxxx_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}
	disable_irq(client->irq);

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	ftxxxx_ts->input_dev = input_dev;

	set_bit(KEY_BACK, input_dev->keybit);
	set_bit(KEY_HOME, input_dev->keybit);
	set_bit(KEY_MENU, input_dev->keybit);
	set_bit(KEY_POWER, input_dev->keybit);

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_mt_init_slots(input_dev, CFG_MAX_TOUCH_POINTS, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
		0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
		0, TOUCH_MAX_X, 0, 0);//ftxxxx_ts->x_max
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
		0, TOUCH_MAX_Y, 0, 0);//ftxxxx_ts->y_max
//	input_set_abs_params(input_dev, ABS_MT_PRESSURE,0, PRESS_MAX, 0, 0);	

	input_dev->name = TOUCH_INPUT_NAME;
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
			"ftxxxx_ts_probe: failed to register input device: %s\n",
			dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}
	/*make sure CTP already finish startup process */
	msleep(150);

#ifdef CONFIG_HAS_EARLYSUSPEND
	ftxxxx_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ftxxxx_ts->early_suspend.suspend = ftxxxx_ts_suspend;
	ftxxxx_ts->early_suspend.resume = ftxxxx_ts_resume;
	register_early_suspend(&ftxxxx_ts->early_suspend);
#endif
	
#ifdef SYSFS_DEBUG
	ftxxxx_create_sysfs(client);
#endif
#ifdef FTS_CTL_IIC
	if (ft_rw_iic_drv_init(client) < 0)
		dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n", __func__);
#endif
#ifdef FTS_APK_DEBUG
	ftxxxx_create_apk_debug_channel(client);
#endif

//	fts_ctpm_auto_upgrade(client);

	/*get some register information */
	uc_reg_addr = FTXXXX_REG_FW_VER;
	flag = ftxxxx_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	pr_info("[FTS] Firmware version = 0x%x\n", uc_reg_value);
	if(flag<0)
	{
		printk("[FTS]chip does not exits\n");
		goto exit_with_out_chip;
		
	}
	else printk("[FTS]chip exits\n");
	
	fts_ctpm_auto_upgrade(client);
	
	uc_reg_addr = FTXXXX_REG_FW_VER;
	ftxxxx_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	pr_info("[FTS] Firmware version = 0x%x\n", uc_reg_value);
	fw = uc_reg_value;

	uc_reg_addr = FTXXXX_REG_POINT_RATE;
	ftxxxx_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	pr_info("[FTS] report rate is %dHz.\n", uc_reg_value * 10);

	uc_reg_addr = FTXXXX_REG_THGROUP;
	ftxxxx_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	pr_info("[FTS] touch threshold is %d.\n", uc_reg_value * 4);

	uc_reg_addr = FTXXXX_REG_VENDOR_ID;
	ftxxxx_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	pr_info("[FTS] VENDOR ID = 0x%x\n", uc_reg_value);
	
	/* register switch device for touch information versions report */
	//add for switch dev
	touch_dev.name = "touch";
	touch_dev.print_name = touch_switch_name;
	if (switch_dev_register(&touch_dev) < 0) {
		printk("[FTS]%s: fail to register touch switch\n", __func__);
	}
	
#ifdef FTS_GESTRUE
	init_para(720,1280,100,0,0);	

	auc_i2c_write_buf[0] = 0xd0;
	auc_i2c_write_buf[1] = 0x01;
	ftxxxx_i2c_Write(client, auc_i2c_write_buf, 2);	//let fw open gestrue function

	auc_i2c_write_buf[0] = 0xd1;//double click and directions gestures.
	auc_i2c_write_buf[1] = 0xff;
	ftxxxx_i2c_Write(client, auc_i2c_write_buf, 2);

	//auc_i2c_write_buf[0] = 0xd2;//d a g c e m w o gesture
	//auc_i2c_write_buf[1] = 0xff;
	//ftxxxx_i2c_Write(client, auc_i2c_write_buf, 2);
	/*
	auc_i2c_write_buf[0] = 0xd0;
	auc_i2c_write_buf[1] = 0x00;
	ftxxxx_i2c_Write(client, auc_i2c_write_buf, 2);	//let fw close gestrue function 
	*/
#endif

	//add by red_zhang@asus.com
    err = sysfs_create_group(&client->dev.kobj, &ft_attr_group);
    if ( err ){
    	printk("[ftxxxx]Failure %d creating sysfs group\n", err);
    }	

	enable_irq(client->irq);

	printk("[ftxxxx] %s end\n", __func__);

	return 0;
exit_with_out_chip:
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&ftxxxx_ts->early_suspend);
#endif

exit_input_register_device_failed:
	input_free_device(input_dev);

exit_input_dev_alloc_failed:
	free_irq(client->irq, ftxxxx_ts);
#ifdef CONFIG_PM
#if 0
exit_request_reset:
	gpio_free(ftxxxx_ts->pdata->reset);
#endif
#endif

exit_init_gpio:
	fts_un_init_gpio_hw(ftxxxx_ts);

exit_irq_request_failed:
	i2c_set_clientdata(client, NULL);
	kfree(ftxxxx_ts);

exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static void ftxxxx_ts_suspend(struct early_suspend *handler)
{
	struct ftxxxx_ts_data *ts = container_of(handler, struct ftxxxx_ts_data, early_suspend);
	int i;
	int ret;
	u8 regvalue;

#if FT_GLOVE_ON
	//add for glove mode
	ret = ftxxxx_read_reg(ts->client,0xc0,&regvalue);
	if(regvalue == 0)
		ft_glove_mode = 0;
	else
		ft_glove_mode = 1;
#endif
	dev_dbg(&ts->client->dev, "[FTS]ftxxxx suspend\n");
	printk("[Red_debug] ft_fac_mode = %d\n", ft_fac_mode);
if(!ft_fac_mode)
{
	disable_irq_nosync(ts->client->irq);
	ftxxxx_write_reg(ts->client,0xa5,0x03);
	msleep(10);
	/*release add touches*/
	//add by Lvyong
	for (i = 0; i <CFG_MAX_TOUCH_POINTS; i++) {
		input_mt_slot(ts->input_dev, i);
		input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
	}
	input_mt_report_pointer_emulation(ts->input_dev, false);
	input_sync(ts->input_dev);
}
}

static void ftxxxx_ts_resume(struct early_suspend *handler)
{
	struct ftxxxx_ts_data *ts = container_of(handler, struct ftxxxx_ts_data, early_suspend);

	dev_dbg(&ts->client->dev, "[FTS]ftxxxx resume.\n");
	printk("[Red_debug] ft_fac_mode = %d\n", ft_fac_mode);
if(!ft_fac_mode)
{
	gpio_request(162,"ts_rst");
	gpio_direction_output(162,0);
	msleep(20);
	gpio_direction_output(162,1);
	msleep(260);
	enable_irq(ts->client->irq);
#if FT_GLOVE_ON	
	//add for glove mode
	msleep(300);
	if(ft_glove_mode == 1)
		ftxxxx_write_reg(ts->client,0xc0,0x01);
#endif
}
}

static int __devexit ftxxxx_ts_remove(struct i2c_client *client)
{
	struct ftxxxx_ts_data *ftxxxx_ts;
	ftxxxx_ts = i2c_get_clientdata(client);
	input_unregister_device(ftxxxx_ts->input_dev);

#ifdef CONFIG_PM
	gpio_free(ftxxxx_ts->pdata->gpio_reset);
#endif
#ifdef FTS_CTL_IIC
	ft_rw_iic_drv_exit();
#endif
#ifdef SYSFS_DEBUG
	ftxxxx_remove_sysfs(client);
#endif
#ifdef FTS_APK_DEBUG
	ftxxxx_release_apk_debug_channel();
#endif
	
#ifdef CONFIG_FACTORY_ITEMS
	sysfs_remove_group(&client->dev.kobj, &ft_attr_group);
#endif
	fts_un_init_gpio_hw(ftxxxx_ts);

	free_irq(client->irq, ftxxxx_ts);

	kfree(ftxxxx_ts);
	i2c_set_clientdata(client, NULL);

	return 0;
}

static const struct i2c_device_id ftxxxx_ts_id[] = {
	{FTXXXX_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ftxxxx_ts_id);

static struct i2c_driver ftxxxx_ts_driver = {
	.probe = ftxxxx_ts_probe,
	.remove = __devexit_p(ftxxxx_ts_remove),
	.id_table = ftxxxx_ts_id,
//	.suspend = ftxxxx_ts_suspend,
//	.resume = ftxxxx_ts_resume,
	.driver = {
		.name = FTXXXX_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init ftxxxx_ts_init(void)
{
	int ret;
	ret = i2c_add_driver(&ftxxxx_ts_driver);
	if (ret) {
		printk(KERN_WARNING "Adding ftxxxx driver failed "
			"(errno = %d)\n", ret);
	} else {
		pr_info("Successfully added driver %s\n",
			ftxxxx_ts_driver.driver.name);
	}
	return ret;
}

static void __exit ftxxxx_ts_exit(void)
{
	i2c_del_driver(&ftxxxx_ts_driver);
}

module_init(ftxxxx_ts_init);
module_exit(ftxxxx_ts_exit);

MODULE_AUTHOR("<OSTeam>");
MODULE_DESCRIPTION("FocalTech TouchScreen driver");
MODULE_LICENSE("GPL");
