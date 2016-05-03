#include <linux/gpio.h>
#include <asm/intel-mid.h>
#include <linux/lnw_gpio.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <asm/dc_xpwr_pwrsrc.h>

static struct dc_xpwr_pwrsrc_pdata pdata;

static void *get_platform_data(void)
{
#if defined(CONFIG_MRD8) || defined(CONFIG_MRD7P05)
	/*
	 * set en_chrg_det to true if the
	 * D+/D- lines are connected to
	 * PMIC itself.
	 */
	pdata.en_chrg_det = true;
#else
	pdata.en_chrg_det = false;
#endif
	return &pdata;
}

void *dc_xpwr_pwrsrc_pdata(void *info)
{

#if defined(CONFIG_MRD8) || defined(CONFIG_MRD7P05)
	int ret;

	pdata.mux_gpio = 131;   /* GPIO_S5[1] */
	ret = gpio_request(pdata.mux_gpio, "otg_gpio");
	if (ret) {
		pr_err("unable to request GPIO pin\n");
		pdata.mux_gpio = -1;
	} else {
		lnw_gpio_set_alt(pdata.mux_gpio, 0);
	}
#endif
	return get_platform_data();
}
