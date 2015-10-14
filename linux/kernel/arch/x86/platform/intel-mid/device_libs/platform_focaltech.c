#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/lnw_gpio.h>
#include <linux/gpio.h>
//#include <linux/power/smb347-charger.h>
#include <asm/intel-mid.h>
#include <asm/intel_crystalcove_pwrsrc.h>

int focaltech_info[2] = {
	62,
	162
};


static struct i2c_board_info __initdata focaltech_i2c_device[] = {
#if defined (CONFIG_TOUCHSCREEN_FTXXXX)	//add by red_zhang@asus.com
        {
         	.type          = "ft5x0x_ts",
		    .addr          = 0x38,
		    .flags         = 0,
		    .irq           = 62,
		    .platform_data = focaltech_info,
        },
#endif	//add by red_zhang@asus.com
};

#ifdef CONFIG_TOUCHSCREEN_FTXXXX
static int __init focaltech_i2c_init(void)
{

	return i2c_register_board_info(0, focaltech_i2c_device, ARRAY_SIZE(focaltech_i2c_device));
}
module_init(focaltech_i2c_init);
#endif
