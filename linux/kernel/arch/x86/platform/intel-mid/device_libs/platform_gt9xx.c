#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/lnw_gpio.h>
#include <linux/gpio.h>
#include <linux/power/smb347-charger.h>
#include <asm/intel-mid.h>
#include <asm/intel_crystalcove_pwrsrc.h>

int goodix_info[2] = {
	62,
	162
};


static struct i2c_board_info __initdata gt9xx_i2c_device[] = {
#if defined (CONFIG_TOUCHSCREEN_GT9XX)	//add by red_zhang@asus.com
        {
         	.type          = "Goodix-TS",
		    .addr          = 0x14,
		    .flags         = 0,
		    .irq           = 62,
		    .platform_data = goodix_info,
        },
#endif	//add by red_zhang@asus.com
};

#ifdef CONFIG_TOUCHSCREEN_GT9XX
static int __init gt9xx_i2c_init(void)
{

	return i2c_register_board_info(0, gt9xx_i2c_device, ARRAY_SIZE(gt9xx_i2c_device));
}
module_init(gt9xx_i2c_init);
#endif
