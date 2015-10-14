#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/gpio.h>
#include <linux/kthread.h>
#include <linux/delay.h>

extern int Read_LCD_ID(void);
extern int Read_PCB_ID(void);
ssize_t pcb_id_read (struct file *filp, char __user *userbuf, size_t size, loff_t *loff_p)
{
	unsigned char len;
	char kernelbuf[50];
	len = sprintf(kernelbuf,"%d\n",Read_LCD_ID());//because pcb id still not ready,just use lcb id as a pcb id
	return simple_read_from_buffer(userbuf, size,loff_p,kernelbuf,len);
}
struct file_operations pcb_id_fops = {
	.read=pcb_id_read,
};

extern int Read_MiniOS_ID(void);
ssize_t MiniOS_id_read (struct file *filp, char __user *userbuf, size_t size, loff_t *loff_p)
{
	unsigned char len;
	char kernelbuf[50];
	len = sprintf(kernelbuf,"%d\n",Read_MiniOS_ID());
	return simple_read_from_buffer(userbuf, size,loff_p,kernelbuf,len);
}
struct file_operations MiniOS_id_fops = {
	.read=MiniOS_id_read,
};

extern int fac_check_headsetstatus(void);
ssize_t headset_status_read (struct file *filp, char __user *userbuf, size_t size, loff_t *loff_p)
{
	unsigned char len;
	char kernelbuf[50];
	len = sprintf(kernelbuf,"%d\n",fac_check_headsetstatus());
	return simple_read_from_buffer(userbuf, size,loff_p,kernelbuf,len);
}
struct file_operations headset_status_fops = {
	.read=headset_status_read,
};

extern int get_voltage_now(void);
ssize_t voltage_now_read (struct file *filp, char __user *userbuf, size_t size, loff_t *loff_p)
{
	unsigned char len;
	char kernelbuf[50];
	len = sprintf(kernelbuf,"%d\n",get_voltage_now());
	return simple_read_from_buffer(userbuf, size,loff_p,kernelbuf,len);
}
struct file_operations voltage_now_fops = {
	.read=voltage_now_read,
};

extern int get_current_now(void);
ssize_t current_now_read (struct file *filp, char __user *userbuf, size_t size, loff_t *loff_p)
{
	unsigned char len;
	char kernelbuf[50];
	len = sprintf(kernelbuf,"%d\n",get_current_now());
	return simple_read_from_buffer(userbuf, size,loff_p,kernelbuf,len);
}
struct file_operations current_now_fops = {
	.read=current_now_read,
};

#define RT8515_FLASH_ENF 		161
#define RT8515_FLASH_ENT  		159

static struct task_struct *camera_flash_tsk;
static int thread_function(void *data)
{
	gpio_direction_output(RT8515_FLASH_ENF, 0);
	gpio_direction_output(RT8515_FLASH_ENT, 0);
	mdelay(4);
	while(kthread_should_stop() == 0)
	{
		gpio_set_value(RT8515_FLASH_ENT, 0);
		mdelay(4);
		gpio_set_value(RT8515_FLASH_ENT, 1);
		mdelay(4);
	}
	gpio_set_value(RT8515_FLASH_ENT, 0);
	return 0;
}
static unsigned char fac_camera_flash_state = 0;
ssize_t camera_flash_write (struct file *filp, const char __user *userbuf, size_t size, loff_t *loff_p)
{
	printk(KERN_ERR"[jevian log]fac interface camera flash arg:%c\n",userbuf[0]);
	if(userbuf[0]=='1')
	{
		if (fac_camera_flash_state == 0)
		{
			camera_flash_tsk = kthread_run(thread_function, NULL, "camera_flash_thread");
			fac_camera_flash_state = 1;
			printk(KERN_ERR"[jevian log]camera flash on\n");
		}
		else
			printk(KERN_ERR"[jevian log]camera is on already\n");
	}
	else
	{
		if (fac_camera_flash_state == 1)
		{
			kthread_stop(camera_flash_tsk);
			fac_camera_flash_state = 0;
			printk(KERN_ERR"[jevian log]camera flash off\n");
		}
		else
			printk(KERN_ERR"[jevian log]camera is off already\n");
	}
	return size;
}
struct file_operations camera_flash_fops = {
	.write=camera_flash_write,
};

#include "asus_battery.h"
unsigned char fac_charge_disable = 0;
ssize_t stop_charge_write (struct file *filp, const char __user *userbuf, size_t size, loff_t *loff_p)
{
	printk(KERN_ERR"[jevian log]fac interface stop charge arg:%c\n",userbuf[0]);
	if(userbuf[0]=='1')
	{
		fac_charge_disable = 1;
		asus_queue_update_all();
		printk(KERN_ERR"[jevian log]stop charge,status\n");
	}
	else
	{
		fac_charge_disable = 0;
		asus_queue_update_all();
		printk(KERN_ERR"[jevian log]start charge\n");
	}
	return size;
}
struct file_operations stop_charge_fops = {
	.write=stop_charge_write,
};

unsigned char disable_power_key = 0;
unsigned char enablewakelock = 1;
extern unsigned char power_key_value;
extern void set_power_key_value(unsigned value);
extern void kernel_power_off(void);
extern void release_wakeup_source(void);
extern void alarm_irq_disable(void);
ssize_t power_key_write (struct file *filp, const char __user *userbuf, size_t size, loff_t *loff_p)
{
	printk(KERN_ERR"[jevian log]fac interface power key arg:%c\n",userbuf[0]);
	if(userbuf[0] < '2')
	{
		disable_power_key = userbuf[0] - '0';
		printk(KERN_ERR"[jevian log]disable power key:%d\n",disable_power_key);
	}
	else if(userbuf[0] == '2')
	{
		unsigned char value;
		value = userbuf[0] - '2';
		set_power_key_value(value);
		printk(KERN_ERR"[jevian log]power key value:%d\n",value);
	}
	else if(userbuf[0] == '3')
		kernel_power_off();
	else if(userbuf[0] == '4')
	{
		enablewakelock = 0;
		release_wakeup_source();
		alarm_irq_disable();
	}
	else if(userbuf[0] == '5')
	{
		enablewakelock = 1;
	}
	return size;
}
ssize_t power_key_read (struct file *filp, char __user *userbuf, size_t size, loff_t *loff_p)
{
	unsigned char len;
	char kernelbuf[5];
	len = sprintf(kernelbuf,"%d\n",power_key_value);
	return simple_read_from_buffer(userbuf, size,loff_p,kernelbuf,len);
}
struct file_operations power_key_fops = {
	.read=power_key_read,
	.write=power_key_write,
};

unsigned char enter_keypad_test_mode = 0;
extern unsigned char vol_key_value;
extern void set_vol_key_value(unsigned value);
ssize_t keypad_test_mode_write (struct file *filp, const char __user *userbuf, size_t size, loff_t *loff_p)
{
	printk(KERN_ERR"[jevian log]fac interface keypad test mode arg:%c\n",userbuf[0]);
	if(userbuf[0] < '2')
	{
		enter_keypad_test_mode = userbuf[0] - '0';
		printk(KERN_ERR"[jevian log]enter keypad test mode:%d\n",disable_power_key);
	}
	else
	{
		unsigned char value;
		value = userbuf[0] - '2';
		set_vol_key_value(value);
		printk(KERN_ERR"vol key value:%d\n",value);
	}
	return size;
}
ssize_t keypad_test_mode_read (struct file *filp, char __user *userbuf, size_t size, loff_t *loff_p)
{
	unsigned char len;
	char kernelbuf[5];
	len = sprintf(kernelbuf,"%d\n",vol_key_value);
	return simple_read_from_buffer(userbuf, size,loff_p,kernelbuf,len);
}
struct file_operations keypad_test_mode_fops = {
	.read=keypad_test_mode_read,
	.write=keypad_test_mode_write,
};

ssize_t printklog_write (struct file *filp, const char __user *userbuf, size_t size, loff_t *loff_p)
{
	char str[128];
	if(size < 128)
	{
		strncpy(str,userbuf,size);
		str[size]='\0';
	}
	else
	{
		strncpy(str,userbuf,127);
		str[127]='\0';
	}
	printk(KERN_ERR"[jevian log]:%s",str);
	return size;
}
struct file_operations printklog_fops = {
	.write=printklog_write,
};

static int __init fac_interface_init(void)
{
	printk(KERN_ERR"[jevian log]this is fac img\n");
	if(proc_create("zc451cg_PCB_ID", 0777, NULL, &pcb_id_fops)==NULL)
	{
		printk(KERN_ERR"create zc451cg_PCB_ID inode is error\n");
	}
	if(proc_create("zc451cg_MiniOS_ID", 0777, NULL, &MiniOS_id_fops)==NULL)
	{
		printk(KERN_ERR"create zc451cg_MiniOS_ID inode is error\n");
	}
	if(proc_create("zc451cg_headset_status", 0777, NULL, &headset_status_fops)==NULL)
	{
		printk(KERN_ERR"create zc451cg_headset_status inode is error\n");
	}
	if(proc_create("zc451cg_voltage_now", 0777, NULL, &voltage_now_fops)==NULL)
	{
		printk(KERN_ERR"create zc451cg_voltage_now inode is error\n");
	}
	if(proc_create("zc451cg_current_now", 0777, NULL, &current_now_fops)==NULL)
	{
		printk(KERN_ERR"create zc451cg_current_now inode is error\n");
	}
	if(proc_create("zc451cg_camera_flash", 0777, NULL, &camera_flash_fops)==NULL)
	{
		printk(KERN_ERR"create zc451cg_camera_flash inode is error\n");
	}
	if(proc_create("zc451cg_stop_charge", 0777, NULL, &stop_charge_fops)==NULL)
	{
		printk(KERN_ERR"create zc451cg_stop_charge inode is error\n");
	}
	if(proc_create("zc451cg_power_key", 0777, NULL, &power_key_fops)==NULL)
	{
		printk(KERN_ERR"create zc451cg_power_key inode is error\n");
	}
	if(proc_create("zc451cg_Keypad_Test_Mode", 0777, NULL, &keypad_test_mode_fops)==NULL)
	{
		printk(KERN_ERR"create zc451cg_Keypad_Test_Mode inode is error\n");
	}
	if(proc_create("zc451cg_printklog", 0777, NULL, &printklog_fops)==NULL)
	{
		printk(KERN_ERR"create zc451cg_printklog inode is error\n");
	}
	return 0;
}

static void __exit fac_interface_exit(void)
{
	;
}

late_initcall(fac_interface_init);
module_exit(fac_interface_exit);
