/*
 * This file is part of the AP3426, AP3212C and AP3216C sensor driver.
 * AP3426 is combined proximity and ambient light sensor.
 * AP3216C is combined proximity, ambient light sensor and IRLED.
 *
 * Contact: John Huang <john.huang@dyna-image.com>
 *	    Templeton Tsai <templeton.tsai@dyna-image.com>
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 *
 * Filename: ap3426.c
 *
 * Summary:
 *	AP3426 device driver.
 *
 * Modification History:
 * Date     By       Summary
 * -------- -------- -------------------------------------------------------
 * 02/02/12 YC       1. Modify irq function to seperate two interrupt routine. 
 *					 2. Fix the index of reg array error in em write. 
 * 02/22/12 YC       3. Merge AP3426 and AP3216C into the same driver. (ver 1.8)
 * 03/01/12 YC       Add AP3212C into the driver. (ver 1.8)
 * 07/25/14 John	  Ver.2.1 , ported for Nexus 7
 * 08/21/14 Templeton AP3426 Ver 1.0, ported for Nexus 7
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <asm/intel-mid.h>
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/ap3426.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/hrtimer.h>
#include <linux/regulator/consumer.h>
#include <linux/proc_fs.h>


#include <linux/wakelock.h>
#define AP3426_DRV_NAME		"ap3426"
#define DRIVER_VERSION		"1"


#define PL_TIMER_DELAY 100
#define INTERRUPT_MODE

//#define LSC_DBG
#ifdef LSC_DBG
#define LDBG(s,args...)	{printk("LDBG: func [%s], line [%d], ",__func__,__LINE__); printk(s,## args);}
#else
#define LDBG(s,args...) {}
#endif
static void plsensor_work_handler(struct work_struct *w);
#ifndef INTERRUPT_MODE
    #ifdef HR_TIMER
        #define MS_TO_NS(x) (x * 1E6L)
        static struct hrtimer hr_timer;
        static enum hrtimer_restart ap3426_hrtimer_callback( struct hrtimer *timer );
        ktime_t ktime;
    #endif
static void pl_timer_callback(unsigned long pl_data);
#endif
struct ap3426_data {
    struct i2c_client *client;
    u8 reg_cache[AP3426_NUM_CACHABLE_REGS];//TO-DO
    u8 power_state_before_suspend;
    int irq;
    struct wake_lock ps_wake_lock;
    struct input_dev	*psensor_input_dev;
    struct input_dev	*lsensor_input_dev;
    struct input_dev	*hsensor_input_dev;
    struct workqueue_struct *plsensor_wq;
    struct work_struct plsensor_work;
#ifndef INTERRUPT_MODE
    struct timer_list pl_timer;
#endif
    int ps_opened;
	int ls_opened;
	uint8_t psensor_sleep_becuz_suspend;
	uint8_t lsensor_sleep_becuz_early_suspend;
	int once_ps_opened;
	int once_ls_opened;
	int last_initial_report_lux;
	uint8_t status_calling;
};
static DECLARE_WORK(ap3426_irq_work, plsensor_work_handler);

static struct ap3426_data *private_pl_data = NULL;
// AP3426 register
static u8 ap3426_reg[AP3426_NUM_CACHABLE_REGS] = 
{0x00,0x01,0x02,0x06,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,
    0x10,0x1A,0x1B,0x1C,0x1D,
    0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x28,0x29,0x2A,0x2B,0x2C,0x2D};

// AP3426 range
static int ap3426_range[4] = {34304,8576,2144,536};


//static u16 ap3426_threshole[8] = {28,444,625,888,1778,3555,7222,0xffff};
static u16 ap3426_threshole[10] = {100,220,444,888,1788,3555,7222,19555,35555,0xffff};
static u8 *reg_array;
static int *range;
static int reg_num = 0;

static int cali = 100;
static int misc_ps_opened = 0;
static int misc_ls_opened = 0;
static int proximity_state = 1;

#define ADD_TO_IDX(addr,idx)	{														\
    int i;												\
    for(i = 0; i < reg_num; i++)						\
    {													\
	if (addr == reg_array[i])						\
	{												\
	    idx = i;									\
	    break;										\
	}												\
    }													\
}

bool EnLSensorConfig_flag =0 ;
bool EnPSensorConfig_flag =0 ;

int lSensor_CALIDATA[2] = {0}; //input calibration data . Format : "200 lux -->lux value ; 1000 lux -->lux value"
int pSensor_CALIDATA[2] = {0}; //input calibration data . Format : "near 3cm :--> value ; far  5cm :--> value"

/*
 * register access helpers
 */

static int __ap3426_read_reg(struct i2c_client *client,
	u32 reg, u8 mask, u8 shift)
{
    struct ap3426_data *data = i2c_get_clientdata(client);
    u8 idx = 0xff;

    ADD_TO_IDX(reg,idx)
	return (data->reg_cache[idx] & mask) >> shift;
}

static int __ap3426_write_reg(struct i2c_client *client,
	u32 reg, u8 mask, u8 shift, u8 val)
{
    struct ap3426_data *data = i2c_get_clientdata(client);
    int ret = 0;
    u8 tmp;
    u8 idx = 0xff;

    ADD_TO_IDX(reg,idx)
	if (idx >= reg_num)
	    return -EINVAL;

    tmp = data->reg_cache[idx];
    tmp &= ~mask;
    tmp |= val << shift;

    ret = i2c_smbus_write_byte_data(client, reg, tmp);
    if (!ret)
	data->reg_cache[idx] = tmp;

    return ret;
}

/*
 * internally used functions
 */

/* range */
static int ap3426_get_range(struct i2c_client *client)
{
    u8 idx = __ap3426_read_reg(client, AP3426_REG_ALS_CONF,
	    AP3426_ALS_RANGE_MASK, AP3426_ALS_RANGE_SHIFT); 
    return range[idx];
}

static int ap3426_set_range(struct i2c_client *client, int range)
{
    return __ap3426_write_reg(client, AP3426_REG_ALS_CONF,
	    AP3426_ALS_RANGE_MASK, AP3426_ALS_RANGE_SHIFT, range);
}


/* mode */
static int ap3426_get_mode(struct i2c_client *client)
{
    int ret;

    ret = __ap3426_read_reg(client, AP3426_REG_SYS_CONF,
	    AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT);
    return ret;
}

static int ap3426_set_mode(struct i2c_client *client, int mode)
{
    int ret;

/*    if(mode == AP3426_SYS_PS_ENABLE) {
	misc_ps_opened = 1;
    } else if(mode == AP3426_SYS_ALS_ENABLE) {
	misc_ls_opened = 1;
    } else if(mode == AP3426_SYS_ALS_PS_ENABLE) {
	misc_ps_opened = 1;
	misc_ls_opened = 1;
    } else if(mode == AP3426_SYS_DEV_DOWN) {
	misc_ps_opened = 0;
	misc_ls_opened = 0;
    }*/
	
    if(mode & AP3426_SYS_PS_ENABLE) {
	misc_ps_opened = 1;
    } else {
    	misc_ps_opened = 0;
    } 

   if(mode & AP3426_SYS_ALS_ENABLE) {
	misc_ls_opened = 1;
    } else {
    	misc_ls_opened = 0;
    }

    ret = __ap3426_write_reg(client, AP3426_REG_SYS_CONF,
	    AP3426_REG_SYS_CONF_MASK, AP3426_REG_SYS_CONF_SHIFT, mode);

    return ret;
}

/* ALS low threshold */
static int ap3426_get_althres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __ap3426_read_reg(client, AP3426_REG_ALS_THDL_L,
	    AP3426_REG_ALS_THDL_L_MASK, AP3426_REG_ALS_THDL_L_SHIFT);
    msb = __ap3426_read_reg(client, AP3426_REG_ALS_THDL_H,
	    AP3426_REG_ALS_THDL_H_MASK, AP3426_REG_ALS_THDL_H_SHIFT);
    return ((msb << 8) | lsb);
}

static int ap3426_set_althres(struct i2c_client *client, int val)
{

    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3426_REG_ALS_THDL_L_MASK;

    err = __ap3426_write_reg(client, AP3426_REG_ALS_THDL_L,
	    AP3426_REG_ALS_THDL_L_MASK, AP3426_REG_ALS_THDL_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap3426_write_reg(client, AP3426_REG_ALS_THDL_H,
	    AP3426_REG_ALS_THDL_H_MASK, AP3426_REG_ALS_THDL_H_SHIFT, msb);

    return err;
}

/* ALS high threshold */
static int ap3426_get_ahthres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __ap3426_read_reg(client, AP3426_REG_ALS_THDH_L,
	    AP3426_REG_ALS_THDH_L_MASK, AP3426_REG_ALS_THDH_L_SHIFT);
    msb = __ap3426_read_reg(client, AP3426_REG_ALS_THDH_H,
	    AP3426_REG_ALS_THDH_H_MASK, AP3426_REG_ALS_THDH_H_SHIFT);
    return ((msb << 8) | lsb);
}

static int ap3426_set_ahthres(struct i2c_client *client, int val)
{
    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3426_REG_ALS_THDH_L_MASK;

    err = __ap3426_write_reg(client, AP3426_REG_ALS_THDH_L,
	    AP3426_REG_ALS_THDH_L_MASK, AP3426_REG_ALS_THDH_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap3426_write_reg(client, AP3426_REG_ALS_THDH_H,
	    AP3426_REG_ALS_THDH_H_MASK, AP3426_REG_ALS_THDH_H_SHIFT, msb);

    return err;
}

/* PX low threshold */
static int ap3426_get_plthres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __ap3426_read_reg(client, AP3426_REG_PS_THDL_L,
	    AP3426_REG_PS_THDL_L_MASK, AP3426_REG_PS_THDL_L_SHIFT);
    msb = __ap3426_read_reg(client, AP3426_REG_PS_THDL_H,
	    AP3426_REG_PS_THDL_H_MASK, AP3426_REG_PS_THDL_H_SHIFT);
    return ((msb << 8) | lsb);
}

static int ap3426_set_plthres(struct i2c_client *client, int val)
{
    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3426_REG_PS_THDL_L_MASK;

    err = __ap3426_write_reg(client, AP3426_REG_PS_THDL_L,
	    AP3426_REG_PS_THDL_L_MASK, AP3426_REG_PS_THDL_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap3426_write_reg(client, AP3426_REG_PS_THDL_H,
	    AP3426_REG_PS_THDL_H_MASK, AP3426_REG_PS_THDL_H_SHIFT, msb);

    return err;
}

//<asus-annacheng20150331><<for crosstalk
static int ap3426_set_crosstalk(struct i2c_client *client, int val)
{
    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3426_REG_PS_CAL_L_MASK;
//	printk("%s:ASUS_PSENSOR SETCALI_DATA : msb :  %d ,lsb:  %d \n", 
//			__func__, msb,lsb);
    err = __ap3426_write_reg(client, AP3426_REG_PS_CAL_L,
	    AP3426_REG_PS_CAL_L_MASK, AP3426_REG_PS_CAL_L_SHIFT, lsb);
    if (err)
	return err;

    err = __ap3426_write_reg(client, AP3426_REG_PS_CAL_H,
	    AP3426_REG_PS_CAL_H_MASK, AP3426_REG_PS_CAL_H_SHIFT, msb);

    return err;
}
//<asus-annacheng20150331><<for crosstalk

/* PX high threshold */
static int ap3426_get_phthres(struct i2c_client *client)
{
    int lsb, msb;
    lsb = __ap3426_read_reg(client, AP3426_REG_PS_THDH_L,
	    AP3426_REG_PS_THDH_L_MASK, AP3426_REG_PS_THDH_L_SHIFT);
    msb = __ap3426_read_reg(client, AP3426_REG_PS_THDH_H,
	    AP3426_REG_PS_THDH_H_MASK, AP3426_REG_PS_THDH_H_SHIFT);
    return ((msb << 8) | lsb);
}

static int ap3426_set_phthres(struct i2c_client *client, int val)
{
    int lsb, msb, err;

    msb = val >> 8;
    lsb = val & AP3426_REG_PS_THDH_L_MASK;
	LDBG("ZXTEST ap3426_set_phthres lsb = %x\n", lsb);
	
    err = __ap3426_write_reg(client, AP3426_REG_PS_THDH_L,
	    AP3426_REG_PS_THDH_L_MASK, AP3426_REG_PS_THDH_L_SHIFT, lsb);
    if (err)
	return err;
	LDBG("ZXTEST ap3426_set_phthres msb = %x\n", msb);
    err = __ap3426_write_reg(client, AP3426_REG_PS_THDH_H,
	    AP3426_REG_PS_THDH_H_MASK, AP3426_REG_PS_THDH_H_SHIFT, msb);
	LDBG("ZXTEST ap3426_set_phthres end = %x\n", msb);
    return err;
}

static int ap3426_get_adc_value(struct i2c_client *client)
{
    unsigned int lsb, msb, val;
#ifdef LSC_DBG
    unsigned int tmp,range;
#endif

    lsb = i2c_smbus_read_byte_data(client, AP3426_REG_ALS_DATA_LOW);

    if (lsb < 0) {
	return lsb;
    }

    msb = i2c_smbus_read_byte_data(client, AP3426_REG_ALS_DATA_HIGH);

    if (msb < 0)
	return msb;

#ifdef LSC_DBG
    range = ap3426_get_range(client);
    tmp = (((msb << 8) | lsb) * range) >> 16;
    tmp = tmp * cali / 100;
//    LDBG("ALS val=%d lux\n",tmp);
#endif
    val = msb << 8 | lsb;

    return val;
}


static int ap3426_get_object(struct i2c_client *client)
{
    int val;

    val = i2c_smbus_read_byte_data(client, AP3426_OBJ_COMMAND);
//    LDBG("val=%x\n", val);
    val &= AP3426_OBJ_MASK;

    return val >> AP3426_OBJ_SHIFT;
}

static int ap3426_get_intstat(struct i2c_client *client)
{
    int val;

    val = i2c_smbus_read_byte_data(client, AP3426_REG_SYS_INTSTATUS);
    val &= AP3426_REG_SYS_INT_MASK;

    return val >> AP3426_REG_SYS_INT_SHIFT;
}


static int ap3426_get_px_value(struct i2c_client *client)
{
    int lsb, msb;

    lsb = i2c_smbus_read_byte_data(client, AP3426_REG_PS_DATA_LOW);

    if (lsb < 0) {
	return lsb;
    }

//    LDBG("%s, IR = %d\n", __func__, (u32)(lsb));
    msb = i2c_smbus_read_byte_data(client, AP3426_REG_PS_DATA_HIGH);

    if (msb < 0)
	return msb;

//    LDBG("%s, IR = %d\n", __func__, (u32)(msb));

    return (u32)(((msb & AL3426_REG_PS_DATA_HIGH_MASK) << 8) | (lsb & AL3426_REG_PS_DATA_LOW_MASK));
}

static void ap3426_change_ls_threshold(struct i2c_client *client);

static int ap3426_lsensor_enable(struct i2c_client *client)
{
    int ret = 0,mode;

    mode = ap3426_get_mode(client);
    if((mode & AP3426_SYS_ALS_ENABLE) == 0){
	mode |= AP3426_SYS_ALS_ENABLE;
	ret = ap3426_set_mode(client,mode);
    }

    return ret;
}

static int ap3426_lsensor_disable(struct i2c_client *client)
{
    int ret = 0,mode;

    mode = ap3426_get_mode(client);
    if(mode & AP3426_SYS_ALS_ENABLE){
	mode &= ~AP3426_SYS_ALS_ENABLE;
	if(mode == AP3426_SYS_DEV_RESET)
	    mode = 0;
	ret = ap3426_set_mode(client,mode);
    }

    return ret;
}


static int lsensor_open(struct inode *inode, struct file *file)
{
	int rc = 0;
	printk("[LS][AP3426] %s\n", __func__);
	
	if (private_pl_data -> ls_opened) {
		pr_err("[LS][AP3426 error]%s: already opened\n", __func__);
		rc = -EBUSY;
	}

	private_pl_data -> ls_opened = 1;
	return rc;
}

static int lsensor_release(struct inode *inode, struct file *file)
{
	printk("[LS][AP3426] %s\n", __func__);
	private_pl_data -> ls_opened = 0;
	return 0;
}

//<-------ward_du------->
//<-- ASUS-Bevis_Chen - -->

//=========================================

//     Calibration Formula:

//     y = f(x) 

//  -> ax - by = constant_k

//     a is f(x2) - f(x1) , b is x2 - x1

////=========================================            

int static calibration_light_ap3426(int x_big, int x_small, int report_lux){        

                    int y_big = 1000;
                    int y_small = 200;
                    int constant_k;

					    if (x_small == 1) {
					        x_small = 0;
					        y_small = 0;
					    }
					    constant_k = (y_big - y_small)*x_small - (x_big - x_small)*y_small;
					    if ( report_lux*(y_big - y_small) < constant_k){
                              return 0; 
                        }else {   
                             return ((report_lux*(y_big - y_small) - constant_k) / (x_big - x_small));			
                        }
}

static long lsensor_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
	int rc, val;
	char encalibration_flag = 0 ;
	void __user *argp = (void __user *)arg;
	/*D("[ap3426] %s cmd %d\n", __func__, _IOC_NR(cmd));*/
	switch (cmd) {
	case LIGHTSENSOR_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg)) {
			rc = -EFAULT;
			break;
		}
		printk("[LS][AP3426] %s LIGHTSENSOR_IOCTL_ENABLE, value = %d\n",
			__func__, val);

		rc = val ? ap3426_lsensor_enable(private_pl_data -> client) : ap3426_lsensor_disable(private_pl_data -> client);
		break;
	case LIGHTSENSOR_IOCTL_GET_ENABLED:
		val = misc_ls_opened;
	   	printk("[LS][AP3426] %s LIGHTSENSOR_IOCTL_GET_ENABLED, enabled %d\n",
			__func__, val);

		rc = put_user(val, (unsigned long __user *)arg);
		break;
	//<----------- ASUS-Bevis_Chen + --------------->
	/*case ASUS_LIGHTSENSOR_IOCTL_START:	
	        printk("%s:ASUS ASUS_LIGHTSENSOR_IOCTL_START  \n", __func__);
	        break;
	case ASUS_LIGHTSENSOR_IOCTL_CLOSE:				
	 	  printk("%s:ASUS ASUS_LIGHTSENSOR_IOCTL_CLOSE \n", __func__);
	        break;*/
	case ASUS_LIGHTSENSOR_IOCTL_GETDATA:
        printk("%s:ASUS ASUS_LIGHTSENSOR_IOCTL_GETDATA \n", __func__);
		rc = 0 ;
		uint16_t adc_value = 0;	
        adc_value = ap3426_get_adc_value(private_pl_data -> client);

		int report_lux = adc_value;		
		if(EnLSensorConfig_flag == 1 ){//calibration enable 
			if(lSensor_CALIDATA[0] > 0&&lSensor_CALIDATA[1] > 0 
  	        	&&lSensor_CALIDATA[1] >lSensor_CALIDATA[0] ){ //in case of zero divisor error
                            
            	printk("AP3426---Before calibration, ASUS_LIGHTSENSOR_IOCTL_GETDATA,  report_lux is %d\n", report_lux);
				report_lux = calibration_light_ap3426(lSensor_CALIDATA[1], lSensor_CALIDATA[0], report_lux * 100 / ( 6553600/ap3426_get_range(private_pl_data -> client)));
				printk("AP3426---After calibration, ASUS_LIGHTSENSOR_IOCTL_GETDATA, report_lux is %d\n", report_lux);
            }else{
				rc = -EINVAL;
				printk("%s:ASUS input lSensor_CALIDATA was invalid .error !!!!!\n",__func__);
			}
		}else{
#ifndef CONFIG_ASUS_FACTORY_SENSOR_MODE // in user build and no calibration data			   
        	report_lux = report_lux*7;
			printk("AP3426--- NO calibration data, use default config\n"); 
#else
	        printk("AP3426--- NO calibration data, Factory branch , NO default config\n"); 	
#endif // end of CONFIG_ASUS_FACTORY_SENSOR_MODE				
			report_lux = report_lux * 100 / ( 6553600/ap3426_get_range(private_pl_data -> client));
			printk("AP3426---After convertion to lux, ASUS_LIGHTSENSOR_IOCTL_GETDATA, report_lux is %d\n", report_lux);
		}

        if ( copy_to_user(argp, &report_lux, sizeof(report_lux) ) ) {
        	printk("%s:ASUS failed to copy lightsense data to user space.\n",__func__);
            rc = -EFAULT;			
            goto end;
	    }		
		printk("%s:ASUS_LIGHTSENSOR_IOCTL_GETDATA end\n", __func__);
		break;
	case ASUS_LIGHTSENSOR_SETCALI_DATA:

		printk("%s:ASUS ASUS_LIGHTSENSOR_SETCALI_DATA \n", __func__);
		rc = 0 ;
		memset(lSensor_CALIDATA, 0, 2*sizeof(int));

		if (copy_from_user(lSensor_CALIDATA, argp, sizeof(lSensor_CALIDATA)))
		{
			rc = -EFAULT;
			goto end;
		}	

		printk("%s:ASUS_LIGHTSENSOR SETCALI_DATA : lSensor_CALIDATA[0] :  %d ,lSensor_CALIDATA[1]:  %d \n", 
			__func__, lSensor_CALIDATA[0],lSensor_CALIDATA[1]);

		if(lSensor_CALIDATA[0] <= 0||lSensor_CALIDATA[1] <= 0 
			||lSensor_CALIDATA[0] >= lSensor_CALIDATA[1] )
			rc =  -EINVAL;


		break;
	case ASUS_LIGHTSENSOR_EN_CALIBRATION:
		printk("%s:ASUS ASUS_LIGHTSENSOR_EN_CALIBRATION \n", __func__);
		rc = 0 ;
		if (copy_from_user(&encalibration_flag , argp, sizeof(encalibration_flag )))
		{
			rc = -EFAULT;
			goto end;
		}	
		EnLSensorConfig_flag =  encalibration_flag ;

		printk("%s: ASUS_LIGHTSENSOR_EN_CALIBRATION : EnLSensorConfig_flag is : %d  \n",__func__,EnLSensorConfig_flag); 
		break;		
	//<----------- ASUS-Bevis_Chen - ------------->

	default:
		pr_err("[LS][AP3426 error]%s: invalid cmd %d\n", __func__, _IOC_NR(cmd));
		rc = -EINVAL;
	}

end:
	
    return rc;
}


static const struct file_operations lsensor_fops = {
	.owner = THIS_MODULE,
	.open = lsensor_open,
	.release = lsensor_release,
	.unlocked_ioctl = lsensor_ioctl
};

static struct miscdevice lsensor_misc_ap3426 = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lightsensor",
	.fops = &lsensor_fops
};


static int ap3426_register_lsensor_device(struct i2c_client *client, struct ap3426_data *data)
{
    struct input_dev *input_dev;
    int rc;

    LDBG("allocating input device lsensor\n");
    input_dev = input_allocate_device();
    if (!input_dev) {
	dev_err(&client->dev,"%s: could not allocate input device for lsensor\n", __FUNCTION__);
	rc = -ENOMEM;
	goto done;
    }
    data->lsensor_input_dev = input_dev;
    input_set_drvdata(input_dev, data);
    input_dev->name = "AP3426LightSensor-level";
    input_dev->dev.parent = &client->dev;
    set_bit(EV_ABS, input_dev->evbit);
    input_set_abs_params(input_dev, ABS_MISC, 0, 8, 0, 0);

    rc = input_register_device(input_dev);
    if (rc < 0) {
	pr_err("%s: could not register input device for lsensor\n", __FUNCTION__);
	goto done;
    }

	rc = misc_register(&lsensor_misc_ap3426);

	if (rc < 0) {
		pr_err( "[PS][AP3426 error]%s: could not register ps misc device\n", __func__);
	goto done;
    }
done:
    return rc;
}


static void ap3426_unregister_lsensor_device(struct i2c_client *client, struct ap3426_data *data)
{
    input_unregister_device(data->lsensor_input_dev);
}
static int ap3426_register_heartbeat_sensor_device(struct i2c_client *client, struct ap3426_data *data)
{
    struct input_dev *input_dev;
    int rc;

    LDBG("allocating input device heartbeat sensor\n");
    input_dev = input_allocate_device();
    if (!input_dev) {
	dev_err(&client->dev,"%s: could not allocate input device for heartbeat sensor\n", __FUNCTION__);
	rc = -ENOMEM;
	goto done;
    }
    data->hsensor_input_dev = input_dev;
    input_set_drvdata(input_dev, data);
    input_dev->name = "heartbeat";
    input_dev->dev.parent = &client->dev;
    set_bit(EV_ABS, input_dev->evbit);
    input_set_abs_params(input_dev, ABS_WHEEL, 0, 8, 0, 0);

    rc = input_register_device(input_dev);
    if (rc < 0) {
	pr_err("%s: could not register input device for heartbeat sensor\n", __FUNCTION__);
	goto done;
    }
done:
    return rc;
}

static void ap3426_unregister_heartbeat_device(struct i2c_client *client, struct ap3426_data *data)
{
    input_unregister_device(data->hsensor_input_dev);
}
static void ap3426_change_ls_threshold(struct i2c_client *client)
{
    struct ap3426_data *data = i2c_get_clientdata(client);
    int value;
	int i,lowTH,highTH,temp=10;
    value = ap3426_get_adc_value(client);
	//pr_err( "[	LS]anna adc  value=%d \n",value);
    if(value > 100){

		for (i=1;i<10;i++)
		{
			if(value <=ap3426_threshole[i])break;
		}
		temp = temp *i;
	
		if(value >(0xffff -(9*temp*3)) ){
			highTH = 0xffff ;
			lowTH = value -temp*3;
		}else{
			if(i ==8 ){
				//printk( "[	LS]anna adc  temp=%d \n",temp);
				temp =  temp*2 ;//need big 
			}else if(i ==9){
				temp =  temp*3;
			}
			
			highTH = value + temp;
			lowTH = value -temp;
			
			if(lowTH < 100 ){  
				lowTH = 100;
			}
		}
		
    }
    else if((value>40)&&(value <= 100)){

		lowTH  = value -5;
		highTH = value+ 5;
		if(lowTH<40) lowTH = 40;
		if(highTH > 100)highTH =100;
    }else{
		if(value <= 1){
			lowTH = 0;
		}else{
			lowTH  = value -1;
		}
		highTH =value+ 1;
	}
	//pr_err( "[	LS]anna adc  lowTH=%d \n",lowTH);
	//pr_err( "[	LS]anna adc  highTH=%d \n",highTH);
	ap3426_set_althres(client,lowTH);
	ap3426_set_ahthres(client,highTH);

	if(EnLSensorConfig_flag == 1 ){//calibration enable 
		if(lSensor_CALIDATA[0] > 0&&lSensor_CALIDATA[1] > 0 
  	        &&lSensor_CALIDATA[1] >lSensor_CALIDATA[0] ){ //in case of zero divisor error
                            
//            printk("AP3426---Before calibration, ASUS_LIGHTSENSOR_IOCTL_GETDATA,  report_lux is %d\n", value);
            value = calibration_light_ap3426(lSensor_CALIDATA[1], lSensor_CALIDATA[0], value * 100 / ( 6553600/ap3426_get_range(private_pl_data -> client)));
//            printk("AP3426---After calibration, ASUS_LIGHTSENSOR_IOCTL_GETDATA, report_lux is %d\n", value);

        }else{
//			printk("%s:ASUS input lSensor_CALIDATA was invalid .error !!!!!\n",__func__);
		}
	}else{
#ifndef CONFIG_ASUS_FACTORY_SENSOR_MODE // in user build and no calibration data			   
//        value = value*15;
//		printk("AP3426--- NO calibration data, use default config\n"); 
#else
//	    printk("AP3426--- NO calibration data, Factory branch , NO default config\n"); 	
#endif // end of CONFIG_ASUS_FACTORY_SENSOR_MODE				
		value = value * 100 / ( 6553600/ap3426_get_range(private_pl_data -> client))*(2000/30)/10;
//		printk("AP3426---After convertion to lux, ASUS_LIGHTSENSOR_IOCTL_GETDATA, report_lux is %d\n", report_lux);

	}

	if (data->once_ls_opened) {
		if (value % 2 == 1)
			value -= 1;
		if (value == data->last_initial_report_lux)
			value += 2;
		data->last_initial_report_lux = value;
		data->once_ls_opened--;
	}
	else {
		if (value % 2 == 0)
			value += 1;
	}
    input_report_abs(data->lsensor_input_dev, ABS_MISC, value);
    input_sync(data->lsensor_input_dev);

}

static int ap3426_psensor_enable(struct i2c_client *client)
{
    int ret = 0,mode;

    mode = ap3426_get_mode(client);
    if((mode & AP3426_SYS_PS_ENABLE) == 0){
	mode |= AP3426_SYS_PS_ENABLE;
	ret = ap3426_set_mode(client,mode);
    }

    return ret;
}

static int ap3426_psensor_disable(struct i2c_client *client)
{
    int ret = 0,mode;

    mode = ap3426_get_mode(client);
    if(mode & AP3426_SYS_PS_ENABLE){
	mode &= ~AP3426_SYS_PS_ENABLE;
	if(mode == AP3426_SYS_DEV_RESET)
	    mode = AP3426_SYS_DEV_DOWN;
	ret = ap3426_set_mode(client,mode);
    }
    return ret;
}


static int psensor_open(struct inode *inode, struct file *file)
{
	printk("[PS][AP3426] %s\n", __func__);
	if (private_pl_data -> ps_opened)
		return -EBUSY;

	private_pl_data -> ps_opened = 1;
	return 0;
}

static int psensor_release(struct inode *inode, struct file *file)
{
	printk("[PS][AP3426] %s\n", __func__);
	private_pl_data -> ps_opened = 0;

//	return ap3426_psensor_disable(private_pl_data -> client);
	return 0;
}

static long psensor_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int val;
	int rc ;
	char enPcalibration_flag = 0 ;
	void __user *argp = (void __user *)arg;
	printk("[PS][AP3426] %s cmd %d\n", __func__, _IOC_NR(cmd));

	switch (cmd) {
	case PROXIMITYSENSOR_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg))
			return -EFAULT;

		if (val){
			return ap3426_psensor_enable(private_pl_data -> client);
		}else{
			return ap3426_psensor_disable(private_pl_data -> client);
		}

		break;
	case PROXIMITYSENSOR_IOCTL_GET_ENABLED:
		return put_user(misc_ps_opened, (unsigned long __user *)arg);
		break;
	case ASUS_PSENSOR_IOCTL_GETDATA:
		printk("%s:ASUS ASUS_PSENSOR_IOCTL_GETDATA \n", __func__);
		rc = 0 ;
		uint16_t px_value;
		int ret;
		ret = ap3426_get_px_value(private_pl_data -> client);
		if (ret < 0) {
			printk("%s:ASUS failed to get_px_value. \n",__func__);
			rc = -EIO;
			goto pend;
		}

		px_value = (uint16_t)ret;
		if ( copy_to_user(argp, &px_value, sizeof(px_value) ) ) {
	     		printk("%s:ASUS failed to copy psense data to user space.\n",__func__);
			rc = -EFAULT;			
			goto pend;
	    	}		

		printk("%s:ASUS_PSENSOR_IOCTL_GETDATA end\n", __func__);
		break;
	case ASUS_PSENSOR_SETCALI_DATA:
		printk("%s:ASUS ASUS_PSENSOR_SETCALI_DATA \n", __func__);
		rc = 0 ;
		memset(pSensor_CALIDATA, 0, 2*sizeof(int));
		if (copy_from_user(pSensor_CALIDATA, argp, sizeof(pSensor_CALIDATA))){
			rc = -EFAULT;
			goto pend;
		}	
		printk("%s:ASUS_PSENSOR SETCALI_DATA : pSensor_CALIDATA[0] :  %d ,pSensor_CALIDATA[1]:  %d \n", 
			__func__, pSensor_CALIDATA[0],pSensor_CALIDATA[1]);
		
		//<asus-annacheng20150331>>>for crosstalk
		if( (pSensor_CALIDATA[1] == 0) && ( pSensor_CALIDATA[0] >= 0  && pSensor_CALIDATA[0] < 512 ) ) {

			ap3426_set_plthres(private_pl_data -> client,246);  //5m    //<asus-annach20150331>
			ap3426_set_phthres(private_pl_data -> client,622);//3             
			if (ap3426_set_crosstalk(private_pl_data -> client, pSensor_CALIDATA[0])) {
				rc = -EFAULT;
				pr_err("[PS][AP3426 error]%s: ap3426_set_plthres error\n", __func__);
				goto pend;
			}
			goto pend;
		}
		//<asus-annacheng20150331><<for crosstalk	

		if(pSensor_CALIDATA[0] <= 0||pSensor_CALIDATA[1] <= 0 
			||pSensor_CALIDATA[0] <= pSensor_CALIDATA[1] ) {
			rc =  -EINVAL;
			goto pend;
		}

		if (ap3426_set_plthres(private_pl_data -> client, pSensor_CALIDATA[1])) {
			rc = -EFAULT;
			pr_err("[PS][AP3426 error]%s: ap3426_set_plthres error\n", __func__);
			goto pend;
		}
		
	 	if (ap3426_set_phthres(private_pl_data -> client, pSensor_CALIDATA[0])) {
			rc = -EFAULT;
			pr_err("[PS][AP3426 error]%s: ap3426_set_phthres error\n", __func__);
			goto pend;
		}
		break;

	case ASUS_PSENSOR_EN_CALIBRATION:
		printk("%s:ASUS ASUS_PSENSOR_EN_CALIBRATION \n", __func__);
		rc = 0 ;
		if (copy_from_user(&enPcalibration_flag , argp, sizeof(enPcalibration_flag ))){
			rc = -EFAULT;
			goto pend;
		}	
		EnPSensorConfig_flag =  enPcalibration_flag ;
		//anna<< for repeat k crosstlak
		if(EnPSensorConfig_flag == 0){	
			ap3426_set_crosstalk(private_pl_data -> client,EnPSensorConfig_flag )	;		
		}
		//anna>>
		printk("%s: ASUS_PSENSOR_EN_CALIBRATION : EnPSensorConfig_flag is : %d  \n",__func__,EnPSensorConfig_flag); 
		break;		
	//<----------- ASUS-Bevis_Chen - ------------->
	default:
		pr_err("[PS][AP3426 error]%s: invalid cmd %d\n",
			__func__, _IOC_NR(cmd));
		return -EINVAL;
	}

	pend:
 		return rc;
}

static const struct file_operations psensor_fops = {
	.owner = THIS_MODULE,
	.open = psensor_open,
	.release = psensor_release,
	.unlocked_ioctl = psensor_ioctl
};

struct miscdevice psensor_misc_ap3426 = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "cm3602",
	.fops = &psensor_fops
};


static int ap3426_register_psensor_device(struct i2c_client *client, struct ap3426_data *data)
{
    struct input_dev *input_dev;
    int rc;

    LDBG("allocating input device psensor\n");
    input_dev = input_allocate_device();
    if (!input_dev) {
	dev_err(&client->dev,"%s: could not allocate input device for psensor\n", __FUNCTION__);
	rc = -ENOMEM;
	goto done;
    }
    data->psensor_input_dev = input_dev;
    input_set_drvdata(input_dev, data);
    input_dev->name = "proximity";
    input_dev->dev.parent = &client->dev;
    set_bit(EV_ABS, input_dev->evbit);
    input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);

    rc = input_register_device(input_dev);
    if (rc < 0) {
	pr_err("%s: could not register input device for psensor\n", __FUNCTION__);
	goto done;
    }

    rc = misc_register(&psensor_misc_ap3426);

    if (rc < 0) {
    	pr_err( "[PS][AP3426 error]%s: could not register ps misc device\n", __func__);
 	goto done;
    }

    return 0;

done:
    return rc;
}

static void ap3426_unregister_psensor_device(struct i2c_client *client, struct ap3426_data *data)
{
    input_unregister_device(data->psensor_input_dev);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend ap3426_early_suspend;
static void ap3426_suspend_early(struct early_suspend *h)
{

   // if (misc_ps_opened)
//	ap3426_psensor_disable(private_pl_data -> client);
	LDBG("ap3426:  ap3426_suspend_early\n");
    if (misc_ls_opened){
	 ap3426_lsensor_disable(private_pl_data -> client);
	 private_pl_data ->lsensor_sleep_becuz_early_suspend = 1;
    }
}

static void ap3426_resume_early(struct early_suspend *h)
{
	LDBG("ap3426:  ap3426_resume_early\n");
    if ((!misc_ls_opened) && private_pl_data ->lsensor_sleep_becuz_early_suspend){
	 ap3426_lsensor_enable(private_pl_data -> client);
	 private_pl_data ->lsensor_sleep_becuz_early_suspend = 0;
    }
	
  //  if (misc_ps_opened)
//	ap3426_psensor_enable(private_pl_data -> client);
}
#endif


static int ap3426_suspend_normal(struct i2c_client *client , pm_message_t mesg)
{   
	struct ap3426_data *data = private_pl_data;
	int status_calling = misc_ps_opened; //<asus-wx20150429>
   	 pr_err("ap3426:  ap3426_suspend_normal\n");
	data->status_calling = status_calling; //<asus-wx20150429+>

	if (status_calling) {
	         pr_err("ap3426:  ap3426_suspend_normal incall\n");
		enable_irq_wake(data->irq);
	}
	enable_irq_wake(data->irq);
	 if (misc_ls_opened){
	 	ap3426_lsensor_disable(private_pl_data -> client);
	 	private_pl_data ->lsensor_sleep_becuz_early_suspend = 1;
    	}
	
	return 0;
}

static int ap3426_resume_normal(struct i2c_client *client)
{
	struct ap3426_data *data = private_pl_data;
	int status_calling = data->status_calling; 
        pr_err("ap3426:  ap3426_resume_normal\n");

	if (status_calling) {
		if (proximity_state == 0) {
			wake_lock_timeout(&(data->ps_wake_lock), 1 * HZ);
		}
	pr_err("ap3426:  ap3426_resume_normal incall\n");
		disable_irq_wake(data->irq);
	}
 
   if ((!misc_ls_opened) && private_pl_data ->lsensor_sleep_becuz_early_suspend){
	 	ap3426_lsensor_enable(private_pl_data -> client);
		 private_pl_data ->lsensor_sleep_becuz_early_suspend = 0;
    }
	return 0;
}

/* range */
static ssize_t ap3426_show_range(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);
    return sprintf(buf, "%i\n", ap3426_get_range(data->client));
}

static ssize_t ap3426_store_range(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);
    unsigned long val;
    int ret;

    if ((strict_strtoul(buf, 10, &val) < 0) || (val > 3))
	return -EINVAL;

    ret = ap3426_set_range(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}

static DEVICE_ATTR(range, S_IWUSR | S_IRUGO,
	ap3426_show_range, ap3426_store_range);


/* mode */
static ssize_t ap3426_show_mode(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);
    return sprintf(buf, "%d\n", ap3426_get_mode(data->client));
}

static ssize_t ap3426_store_mode(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);
    unsigned long val;
    int ret;
	int lastmode;
	int pl_enabled;

	disable_irq_nosync(data->client->irq);

    if ((strict_strtoul(buf, 10, &val) < 0) || (val % 10 > 7) || (val / 10 > 2))
    	{
		return -EINVAL;
    	}

 	pr_err( " ap3426_store_mode (%d)\n", val);
	pl_enabled = val / 10;
	val %= 10;
	lastmode = ap3426_get_mode(data->client);
    ret = ap3426_set_mode(data->client, val);

    if (ret < 0)
	return ret;
/* <anna -->
    if ((pl_enabled == 2) && (val & AP3426_SYS_PS_ENABLE)) {
		data->once_ps_opened = 1;
    }
	else {
		data->once_ps_opened = 0;
	}
    if (((lastmode & AP3426_SYS_ALS_ENABLE) == 0)
		&& ((val & AP3426_SYS_ALS_ENABLE) == 1)){
		data->once_ls_opened = 1;
    }
    if (((lastmode & AP3426_SYS_ALS_ENABLE) == 1)
		&& ((val & AP3426_SYS_ALS_ENABLE) == 0)){
		data->once_ls_opened = 0;
    }
    */
	enable_irq(data->client->irq);
/*
	LDBG("Starting timer to fire in 200ms (%ld)\n", jiffies );
    ret = mod_timer(&data->pl_timer, jiffies + msecs_to_jiffies(PL_TIMER_DELAY));

    if(ret) 
	LDBG("Timer Error\n");
	*/
    return count;
}

static DEVICE_ATTR(mode, S_IRUGO | S_IWUSR | S_IWGRP,
	ap3426_show_mode, ap3426_store_mode);


static int ap3426_ps_init(struct i2c_client *client)
{
    int ret;

    //LDBG("Init PS \n");
   // ret = ap3426_set_plthres(client, 100);
  //  ret = ap3426_set_phthres(client, 500);
    ret = __ap3426_write_reg(client, AP3426_REG_PS_LEDD,
        AP3426_REG_PS_LEDD_MASK, AP3426_REG_PS_LEDD_SHIFT, 0x03);
    ret = __ap3426_write_reg(client, AP3426_REG_PS_MEAN,
        AP3426_REG_PS_MEAN_MASK, AP3426_REG_PS_MEAN_SHIFT, 0);
 //   ret = __ap3426_write_reg(client, AP3426_REG_PS_INTEGR,
   //     AP3426_REG_PS_INTEGR_MASK, AP3426_REG_PS_INTEGR_SHIFT, 0x08);
    ret = __ap3426_write_reg(client, AP3426_REG_SYS_INTCTRL,0xFF, 0x00, 0x89);
 //   ret = __ap3426_write_reg(client, AP3426_REG_SYS_CONF,
   //     AP3426_REG_SYS_INT_PS_MASK, 0x00, AP3426_SYS_PS_ENABLE);
  ret = __ap3426_write_reg(client, AP3426_REG_PS_CONF, 
        AP3426_REG_PS_CONF_MASK, AP3426_REG_PS_CONF_SHIFT, 4);
 
    if(ret < 0)
        LDBG("Init PS Failed\n");

    return ret;
}
static int ap3426_als_init(struct i2c_client *client)
{
    int ret;

    //LDBG("Init ALS \n");
    ret = ap3426_set_althres(client, 5000);
    ret = ap3426_set_ahthres(client, 13000);
   // ret = ap3426_set_range(client, AP3426_ALS_RANGE_2);
    ret = __ap3426_write_reg(client, AP3426_REG_SYS_INTCTRL,0xFF, 0x00, 0x89);
    ret = __ap3426_write_reg(client, AP3426_REG_SYS_CONF,
        AP3426_REG_SYS_INT_AL_MASK, 0x00, AP3426_SYS_ALS_ENABLE);
    //ret = __ap3426_write_reg(client, AP3426_REG_ALS_PERSIS,0xFF, 0x00, 0x2); 
 
    if(!ret < 0)
        LDBG("Init ALS Failed\n");

    return ret;
}  
/* lux */
static ssize_t ap3426_show_lux(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);

    /* No LUX data if power down */
    if (ap3426_get_mode(data->client) == AP3426_SYS_DEV_DOWN)
	return sprintf((char*) buf, "%s\n", "Please power up first!");

    return sprintf(buf, "%d\n", ap3426_get_adc_value(data->client));
}

static DEVICE_ATTR(lux, S_IRUGO, ap3426_show_lux, NULL);


/* Px data */
static ssize_t ap3426_show_pxvalue(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);

    /* No Px data if power down */
    if (ap3426_get_mode(data->client) == AP3426_SYS_DEV_DOWN)
	return -EBUSY;

    return sprintf(buf, "%d\n", ap3426_get_px_value(data->client));
}

static DEVICE_ATTR(pxvalue, S_IRUGO, ap3426_show_pxvalue, NULL);


/* proximity object detect */
static ssize_t ap3426_show_object(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);
    return sprintf(buf, "%d\n", ap3426_get_object(data->client));
}

static DEVICE_ATTR(object, S_IRUGO, ap3426_show_object, NULL);


/* ALS low threshold */
static ssize_t ap3426_show_althres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);
    return sprintf(buf, "%d\n", ap3426_get_althres(data->client));
}

static ssize_t ap3426_store_althres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3426_set_althres(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}

static DEVICE_ATTR(althres, S_IWUSR | S_IRUGO,
	ap3426_show_althres, ap3426_store_althres);


/* ALS high threshold */
static ssize_t ap3426_show_ahthres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);
    return sprintf(buf, "%d\n", ap3426_get_ahthres(data->client));
}

static ssize_t ap3426_store_ahthres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3426_set_ahthres(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}

static DEVICE_ATTR(ahthres, S_IWUSR | S_IRUGO,
	ap3426_show_ahthres, ap3426_store_ahthres);

/* Px low threshold */
static ssize_t ap3426_show_plthres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);
    return sprintf(buf, "%d\n", ap3426_get_plthres(data->client));
}

static ssize_t ap3426_store_plthres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3426_set_plthres(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}

static DEVICE_ATTR(plthres, S_IWUSR | S_IRUGO,
	ap3426_show_plthres, ap3426_store_plthres);

/* Px high threshold */
static ssize_t ap3426_show_phthres(struct device *dev,
	struct device_attribute *attr, char *buf)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);
    return sprintf(buf, "%d\n", ap3426_get_phthres(data->client));
}

static ssize_t ap3426_store_phthres(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);
    unsigned long val;
    int ret;

    if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

    ret = ap3426_set_phthres(data->client, val);
    if (ret < 0)
	return ret;

    return count;
}

static DEVICE_ATTR(phthres, S_IWUSR | S_IRUGO,
	ap3426_show_phthres, ap3426_store_phthres);


/* calibration */
static ssize_t ap3426_show_calibration_state(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
    return sprintf(buf, "%d\n", cali);
}

static ssize_t ap3426_store_calibration_state(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
    struct input_dev *input = to_input_dev(dev);
    struct ap3426_data *data = input_get_drvdata(input);
    int stdls, lux; 
    char tmp[10];

    LDBG("DEBUG ap3426_store_calibration_state..\n");

    /* No LUX data if not operational */
    if (ap3426_get_mode(data->client) == AP3426_SYS_DEV_DOWN)
    {
	printk("Please power up first!");
	return -EINVAL;
    }

    cali = 100;
    sscanf(buf, "%d %s", &stdls, tmp);

    if (!strncmp(tmp, "-setcv", 6))
    {
	cali = stdls;
	return -EBUSY;
    }

    if (stdls < 0)
    {
	printk("Std light source: [%d] < 0 !!!\nCheck again, please.\n\
		Set calibration factor to 100.\n", stdls);
	return -EBUSY;
    }

    lux = ap3426_get_adc_value(data->client);
    cali = stdls * 100 / lux;

    return -EBUSY;
}

static DEVICE_ATTR(calibration, S_IWUSR | S_IRUGO,
	ap3426_show_calibration_state, ap3426_store_calibration_state);

#ifdef LSC_DBG
/* engineer mode */
static ssize_t ap3426_em_read(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct ap3426_data *data = i2c_get_clientdata(client);
    int i;
    u8 tmp;

    LDBG("DEBUG ap3426_em_read..\n");

    for (i = 0; i < reg_num; i++)
    {
	tmp = i2c_smbus_read_byte_data(data->client, reg_array[i]);

	LDBG("Reg[0x%x] Val[0x%x]\n", reg_array[i], tmp);
    }

    return 0;
}

static ssize_t ap3426_em_write(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct ap3426_data *data = i2c_get_clientdata(client);
    u32 addr,val,idx=0;
    int ret = 0;

    LDBG("DEBUG ap3426_em_write..\n");

    sscanf(buf, "%x%x", &addr, &val);

    printk("Write [%x] to Reg[%x]...\n",val,addr);

    ret = i2c_smbus_write_byte_data(data->client, addr, val);
    ADD_TO_IDX(addr,idx)
	if (!ret)
	    data->reg_cache[idx] = val;

    return count;
}
static DEVICE_ATTR(em, S_IWUSR |S_IRUGO,
	ap3426_em_read, ap3426_em_write);
#endif

static struct attribute *ap3426_attributes[] = {
    &dev_attr_range.attr,
    &dev_attr_mode.attr,
    &dev_attr_lux.attr,
    &dev_attr_object.attr,
    &dev_attr_pxvalue.attr,
    &dev_attr_althres.attr,
    &dev_attr_ahthres.attr,
    &dev_attr_plthres.attr,
    &dev_attr_phthres.attr,
    &dev_attr_calibration.attr,
#ifdef LSC_DBG
    &dev_attr_em.attr,
#endif
    NULL
};

static const struct attribute_group ap3426_attr_group = {
    .attrs = ap3426_attributes,
};

static int ap3426_init_client(struct i2c_client *client)
{
    struct ap3426_data *data = i2c_get_clientdata(client);
    int i;

    LDBG("DEBUG ap3426_init_client..\n");


    /* read all the registers once to fill the cache.
     * if one of the reads fails, we consider the init failed */
    for (i = 0; i < reg_num; i++) {
	int v = i2c_smbus_read_byte_data(client, reg_array[i]);
	if (v < 0)
	    return -ENODEV;

	data->reg_cache[i] = v;
    }

    /* set defaults */
    ap3426_set_range(client, AP3426_ALS_RANGE_1);
    ap3426_set_mode(client, AP3426_SYS_DEV_DOWN);

	ap3426_set_plthres(client,276);  //5m    //<asus-annach20150331>
	ap3426_set_phthres(client,627);//3             
	// ap3426_set_althres(client, 5000);
            //ap3426_set_ahthres(client, 13000);

	__ap3426_write_reg(client, AP3426_REG_PS_INTEGR,
	    0, 0, 0x0f);

	__ap3426_write_reg(client, AP3426_REG_SYS_INTCTRL,
	    0x88, 0, 0x00);


    return 0;
}

#ifndef INTERRUPT_MODE
void pl_timer_callback(unsigned long pl_data)
{
    struct ap3426_data *data;
    int ret =0;

    data = private_pl_data;
    queue_work(data->plsensor_wq, &data->plsensor_work);

    ret = mod_timer(&private_pl_data->pl_timer, jiffies + msecs_to_jiffies(PL_TIMER_DELAY));

    if(ret) 
	LDBG("Timer Error\n");

}
    #ifdef HR_TIMER 
        static enum hrtimer_restart ap3426_hrtimer_callback( struct hrtimer *timer )
        {
            struct ap3426_data *data;
            int ret =0;
            ktime_t currtime , interval;
            currtime  = ktime_get();

            data = private_pl_data;
            queue_work(data->plsensor_wq, &data->plsensor_work);

            //LDBG("%s\n", __func__);
            interval = ktime_set(0, MS_TO_NS(PL_TIMER_DELAY)); 
            hrtimer_forward(timer, currtime , interval);

            if(ret) 
            LDBG("Timer Error\n");
            return HRTIMER_RESTART;
        }
    #endif
#endif
static void plsensor_work_handler(struct work_struct *w)
{

    struct ap3426_data *data =private_pl_data;
	//container_of(w, struct ap3426_data, plsensor_work);
    u8 int_stat;
    int pxvalue;
    int distance;
    //int obj;
    int ret;
   
    unsigned char INT_Reg;
	
	//pr_err( "[PS]anna irq\n");
    int_stat = ap3426_get_intstat(data->client);
    if (int_stat & AP3426_REG_SYS_INT_AL_MASK) 
        INT_Reg &= 0xFE;
    if (int_stat & AP3426_REG_SYS_INT_PS_MASK)
        INT_Reg &= 0xFD;
    if (INT_Reg != int_stat){
        ret = __ap3426_write_reg(data->client, AP3426_REG_SYS_INTSTATUS,
            0x03, 0x00, INT_Reg);
    }
    // ALS int
    if (misc_ls_opened == 1){
        #ifdef INTERRUPT_MODE
            if (int_stat & AP3426_REG_SYS_INT_AMASK)
            {
            	
            	/*value = ap3426_get_adc_value(data->client);
            	input_report_abs(data->lsensor_input_dev, ABS_MISC, value);				
            	input_sync(data->lsensor_input_dev);;
		ap3426_set_ahthres(data->client,Highth);  */
            	ap3426_change_ls_threshold(data->client);
            }
        #else    
	   int value;
            value = ap3426_get_adc_value(data->client);
          //  LDBG("ALS ADC Data: %d\n", value);
            input_report_abs(data->lsensor_input_dev, ABS_MISC, value);
            input_sync(data->lsensor_input_dev);
        #endif
    }

  /*  if(data->hsensor_enable) {
        if (int_stat & AP3426_REG_SYS_INT_PMASK)
        {    
         // pr_err( "[PS]anna irq 33 \n");
            pxvalue = ap3426_get_px_value(data->client);
            input_report_rel(data->hsensor_input_dev, ABS_WHEEL, pxvalue);
            input_sync(data->hsensor_input_dev);
           // LDBG("hsensor pxvalue = %d\n", pxvalue);
        }
    }else{ */
        if (int_stat & AP3426_REG_SYS_INT_PMASK)
        {    
    distance = ap3426_get_object(data->client);
		//	if (data->once_ps_opened == 0) {
		if(distance == 1){
			distance = 0;
		}else{	
			distance = 1;}
		//	}else{
		 //		data->once_ps_opened--;
		//	distance = 1;
	   	//}

//			LDBG("ZXTEST plsensor_work_handler distance = %x\n", distance);
    input_report_abs(data->psensor_input_dev, ABS_DISTANCE, distance);
    input_sync(data->psensor_input_dev);
		proximity_state = distance;
    pxvalue = ap3426_get_px_value(data->client); 
//	LDBG("ZXTEST plsensor_work_handler pxvalue = %x\n", pxvalue);
    input_report_abs(data->hsensor_input_dev, ABS_WHEEL, pxvalue);
    input_sync(data->hsensor_input_dev);
           	// LDBG("px-value = %d -> %s\n", pxvalue, obj ? "obj near":"obj far");
        }
   // }

#ifdef INTERRUPT_MODE
    enable_irq(data->client->irq);
#endif
}
/*
 * I2C layer
 */

static irqreturn_t ap3426_irq(int irq, void *data_)
{
    struct ap3426_data *data = data_;
    if (proximity_state == 0) {
		wake_lock_timeout(&(data->ps_wake_lock), 1 * HZ);
		//pr_err("ap3426:  ap3426_irq \n");
	}
    disable_irq_nosync(data->client->irq);
    queue_work(data->plsensor_wq, &ap3426_irq_work);

    return IRQ_HANDLED;
}

static int ap3426_setup(struct i2c_client *client)
{

	int ret = 0;
	int gpio;

	
	gpio = get_gpio_by_name("ALS_INT#");

	ret = gpio_request(gpio, "gpio_ap3426_intr");

	if (ret < 0) {
		pr_err("[PS][ap3426 error]%s: gpio %d request failed (%d)\n", __func__, gpio, ret);
		return ret;
	}

	ret = gpio_direction_input(gpio);
	if (ret < 0) {
   		   pr_err( "[PS][ap3426 error]%s: fail to set gpio %d as input (%d)\n", __func__, gpio, ret);
		 goto fail_free_intr_pin;
	}

    	client->irq =  gpio_to_irq(gpio); 
	LDBG("ZXTEST ap3426_probe client->irq = %x\n", client->irq);


	ret = enable_irq_wake(client->irq);

	if (ret < 0) {
           pr_err( "[PS][ap3426 error]%s: req_irq(%d) fail for gpio %d (%d)\n", __func__, client->irq, gpio, ret);
           goto fail_free_intr_pin;
	}


	return ret;

fail_free_intr_pin:
	gpio_free(gpio);

	return ret;
}

static int __devinit ap3426_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
    struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
    struct ap3426_data *data;
    int err = 0;
 int status =0;
 
	pr_err("[PS][ap3426] %s\n", __func__);

    if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)){
		pr_err("[PS_ERR][ap3426 error]%s: ap3426 not present!\n", __func__);
	err = -EIO;
	goto exit_free_gpio;
    }
	pr_err("[PS_ERR][ap3426 error]%s: ap3426 present test!\n", __func__);
    status = i2c_smbus_read_byte_data(client, AP3426_REG_SYS_CONF);
	if (status < 0) {
	pr_err("[PS_ERR][ap3426 error]%s: ap3426 is not present !\n", __func__);
	err = -ENODEV;
	goto exit_free_gpio;
    }

    reg_array = ap3426_reg;
    range = ap3426_range;
    reg_num = AP3426_NUM_CACHABLE_REGS;

    data = kzalloc(sizeof(struct ap3426_data), GFP_KERNEL);
    if (!data){
	err = -ENOMEM;
	goto exit_free_gpio;
    }

    // Instead of set in intel-mid.c, irq value is set here.   

	err =ap3426_setup(client);

	if (err) {
		pr_err("[PS_ERR][ap3426 error]%s: ap3426_setup error!\n", __func__);
		goto exit_kfree;
	}

    data->client = client;
    i2c_set_clientdata(client, data);
    data->irq = client->irq;
    data->status_calling = 0;

	LDBG("annatest ap3426_probe client->irq = %x\n", client->irq);
	
    /* initialize the AP3426 chip */
           err = ap3426_init_client(client);
	err =  ap3426_als_init(client);
	err =  ap3426_ps_init(client);
	//pr_err("[PS_ERR][ap3426 error]%s: ap3426 present test!4444\n", __func__);


    if (err)
	goto exit_kfree;

    err = ap3426_register_lsensor_device(client,data);
    if (err){
	dev_err(&client->dev, "failed to register_lsensor_device\n");
	goto exit_kfree;
    }

    err = ap3426_register_psensor_device(client, data);
    if (err) {
	dev_err(&client->dev, "failed to register_psensor_device\n");
	goto exit_free_ls_device;
    }

    err = ap3426_register_heartbeat_sensor_device(client, data);
    if (err) {
	dev_err(&client->dev, "failed to register_heartbeatsensor_device\n");
	goto exit_free_heartbeats_device;
    }
     	wake_lock_init(&(data->ps_wake_lock), WAKE_LOCK_SUSPEND, "proximity");
#ifdef INTERRUPT_MODE

    data->plsensor_wq = create_singlethread_workqueue("plsensor_wq");
    if (!data->plsensor_wq) {
	LDBG("%s: create workqueue failed\n", __func__);
	err = -ENOMEM;
	goto err_create_wq_failed;
    }

  err = request_irq (client->irq,
                       ap3426_irq,
                       IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
             			"ap3426",
            			data);
  if (err) {
    	dev_err(&client->dev, "ret: %d, could not get IRQ %d\n",err,client->irq);
    	goto exit_free_ps_device;
    }

#else    
    #ifdef HR_TIMER 
        LDBG("HR Timer module installing\n");
        ktime = ktime_set( 0, MS_TO_NS(PL_TIMER_DELAY) );
        hrtimer_init( &hr_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
        hr_timer.function = &ap3426_hrtimer_callback;
        LDBG( "Starting timer to fire in %dms (%ld)\n", PL_TIMER_DELAY, jiffies );
        hrtimer_start( &hr_timer, ktime, HRTIMER_MODE_REL );   
    #else
        LDBG("Timer module installing\n");
        setup_timer(&data->pl_timer, pl_timer_callback, 0);
    #endif
#endif


#if 1
    /* register sysfs hooks */
    err = sysfs_create_group(&data->client->dev.kobj, &ap3426_attr_group);
    if (err)
	goto exit_free_ps_device;
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
    ap3426_early_suspend.suspend = ap3426_suspend_early;
    ap3426_early_suspend.resume  = ap3426_resume_early;
    ap3426_early_suspend.level   = 0x02;
    register_early_suspend(&ap3426_early_suspend);
#endif


  //  INIT_WORK(&data->plsensor_work, plsensor_work_handler);


    private_pl_data = data;
    dev_info(&client->dev, "Driver version %s enabled\n", DRIVER_VERSION);
	pr_err("[PS][ap3426] %s end  anna test\n", __func__);
    return 0;
err_create_wq_failed:
#ifndef INTERRUPT_MODE
    #ifdef HR_TIMER
    if(&data->pl_timer != NULL)
        hrtimer_cancel( &hr_timer );
    #else
        if(&data->pl_timer != NULL)
	del_timer(&data->pl_timer);
    #endif
#endif
    if (data->plsensor_wq)
	destroy_workqueue(data->plsensor_wq);
exit_free_ps_device:
    ap3426_unregister_psensor_device(client,data);
    wake_lock_destroy(&(data->ps_wake_lock));
exit_free_heartbeats_device:
    ap3426_unregister_heartbeat_device(client,data);
exit_free_ls_device:
    ap3426_unregister_lsensor_device(client,data);

exit_kfree:
    kfree(data);

exit_free_gpio:
     
    return err;
}

static int __devexit ap3426_remove(struct i2c_client *client)
{
    struct ap3426_data *data = i2c_get_clientdata(client);
    free_irq(data->irq, data);

    sysfs_remove_group(&data->client->dev.kobj, &ap3426_attr_group);
    ap3426_unregister_psensor_device(client,data);
    ap3426_unregister_lsensor_device(client,data);
    ap3426_unregister_heartbeat_device(client,data);
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&ap3426_early_suspend);
#endif

    ap3426_set_mode(client, 0);
    kfree(i2c_get_clientdata(client));

    if (data->plsensor_wq)
	destroy_workqueue(data->plsensor_wq);
#ifndef INTERRUPT_MODE
    #ifdef HR_TIMER
        if(&data->pl_timer != NULL)
        hrtimer_cancel( &hr_timer );
    #else
        if(&data->pl_timer != NULL)
	del_timer(&data->pl_timer);
    #endif
#endif
    return 0;
}

static const struct i2c_device_id ap3426_id[] = {
    { AP3426_DRV_NAME, 0 },
    {}
};
MODULE_DEVICE_TABLE(i2c, ap3426_id);

static struct i2c_driver ap3426_driver = {
    .driver = {
	.name	= AP3426_DRV_NAME,
	.owner	= THIS_MODULE,
    },
    .probe	= ap3426_probe,
    .remove	= __devexit_p(ap3426_remove),
    .suspend = ap3426_suspend_normal,
    .resume = ap3426_resume_normal,
    .id_table = ap3426_id,
};

static struct i2c_board_info AP3426_board_info[] = {
	{
    		I2C_BOARD_INFO(AP3426_DRV_NAME, 0x1e),
	},
};


static int __init ap3426_init(void)
{
    int ret;

    LDBG("ap3426_init\n");

	i2c_register_board_info(0x05, AP3426_board_info, ARRAY_SIZE(AP3426_board_info));

    ret = i2c_add_driver(&ap3426_driver);
    return ret;	

}

static void __exit ap3426_exit(void)
{
    i2c_del_driver(&ap3426_driver);
}

MODULE_AUTHOR("Templeton Tsai Dyna-Image Corporation.");
MODULE_DESCRIPTION("AP3426 driver.");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

module_init(ap3426_init);
module_exit(ap3426_exit);



