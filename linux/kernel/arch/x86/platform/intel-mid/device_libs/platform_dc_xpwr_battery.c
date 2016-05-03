
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/power/dc_xpwr_battery.h>
#include <asm/intel_em_config.h>

#ifdef CONFIG_BATTERY_TYPE
#if defined(CONFIG_BATTERY_BT_B0BFQ)
#define THERM_CURVE_MAX_SAMPLES 18
#elif defined(CONFIG_BATTERY_BT_E002H)
#define THERM_CURVE_MAX_SAMPLES 18
#endif
#else
#define THERM_CURVE_MAX_SAMPLES 18
#endif
#define THERM_CURVE_MAX_VALUES	4

static struct dollarcove_fg_pdata pdata;

int bat_curve[] = {
	0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1, 0x2,
	0x2, 0x4, 0x6, 0xc, 0x14, 0x20, 0x2a, 0x2d,
	0x31, 0x34, 0x38, 0x3d, 0x42, 0x45, 0x49, 0x4c,
	0x50, 0x53, 0x55, 0x56, 0x5a, 0x5d, 0x61, 0x64,
};

/*
 * This array represents the Battery Pack thermistor
 * temperature and corresponding ADC value limits
 */
static int const therm_curve_data[THERM_CURVE_MAX_SAMPLES]
	[THERM_CURVE_MAX_VALUES] = {
#ifdef CONFIG_BATTERY_TYPE
#if defined(CONFIG_BATTERY_BT_B0BFQ)
	/* {temp_max, temp_min, adc_max, adc_min} */
	{-15, -20, 508 , 401 },
	{-10, -15, 401 , 318 },
	{-5, -10, 318 , 254 },
	{0, -5, 254 , 205 },
	{5, 0, 205 , 166 },
	{10, 5, 166 , 135 },
	{15, 10, 135 , 110 },
	{20, 15, 110 , 91 },
	{25, 20, 91 , 75 },
	{30, 25, 75 , 62 },
	{35, 30, 62 , 52 },
	{40, 35, 52 , 44 },
	{45, 40, 44 , 37 },
	{50, 45, 37 , 31 },
	{55, 50, 31 , 27 },
	{60, 55, 27 , 23 },
	{65, 60, 23 , 19 },
	{70, 65, 19 , 17 },
#elif defined(CONFIG_BATTERY_BT_E002H)
	/* {temp_max, temp_min, adc_max, adc_min} */
	{-15, -20, 508 , 401 },
	{-10, -15, 401 , 318 },
	{-5, -10, 318 , 254 },
	{0, -5, 254 , 205 },
	{5, 0, 205 , 166 },
	{10, 5, 166 , 135 },
	{15, 10, 135 , 110 },
	{20, 15, 110 , 91 },
	{25, 20, 91 , 75 },
	{30, 25, 75 , 62 },
	{35, 30, 62 , 52 },
	{40, 35, 52 , 44 },
	{45, 40, 44 , 37 },
	{50, 45, 37 , 31 },
	{55, 50, 31 , 27 },
	{60, 55, 27 , 23 },
	{65, 60, 23 , 19 },
	{70, 65, 19 , 17 },
#endif
#else
	/* {temp_max, temp_min, adc_max, adc_min} */
	{-15, -20, 682, 536},
	{-10, -15, 536, 425},
	{-5, -10, 425, 338},
	{0, -5, 338, 272},
	{5, 0, 272, 220},
	{10, 5, 220, 179},
	{15, 10, 179, 146},
	{20, 15, 146, 120},
	{25, 20, 120, 100},
	{30, 25, 100, 83},
	{35, 30, 83, 69},
	{40, 35, 69, 58},
	{45, 40, 58, 49},
	{50, 45, 49, 41},
	{55, 50, 41, 35},
	{60, 55, 35, 30},
	{65, 60, 30, 25},
	{70, 65, 25, 22},
#endif
};

static int conv_adc_temp(int adc_val, int adc_max, int adc_diff, int temp_diff)
{
	int ret;

	ret = (adc_max - adc_val) * temp_diff;
	return ret / adc_diff;
}

static bool is_valid_temp_adc_range(int val, int min, int max)
{
	if (val > min && val <= max)
		return true;
	else
		return false;
}

static int dc_xpwr_get_batt_temp(int adc_val, int *temp)
{
	int i;

	for (i = 0; i < THERM_CURVE_MAX_SAMPLES; i++) {
		/* linear approximation for battery pack temperature */
		if (is_valid_temp_adc_range(adc_val, therm_curve_data[i][3],
					    therm_curve_data[i][2])) {

			*temp = conv_adc_temp(adc_val, therm_curve_data[i][2],
					     therm_curve_data[i][2] -
					     therm_curve_data[i][3],
					     therm_curve_data[i][0] -
					     therm_curve_data[i][1]);

			*temp += therm_curve_data[i][1];
			break;
		}
	}

	if (i >= THERM_CURVE_MAX_SAMPLES)
		return -ERANGE;

	return 0;

}
static bool dollarcove_is_valid_batid(void)
{
	struct em_config_oem0_data data;
	bool ret = true;

	if (!em_config_get_oem0_data(&data))
		ret = false;

	return ret;
}

static void *get_platform_data(void)
{
	int i;

	if (dollarcove_is_valid_batid()) {
		snprintf(pdata.battid, (BATTID_LEN + 1),
				"%s", "INTN0001");
		pdata.technology = POWER_SUPPLY_TECHNOLOGY_LION;
	} else {
		snprintf(pdata.battid, (BATTID_LEN + 1),
				"%s", "UNKNOWNB");
		pdata.technology = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
	}

	pdata.batt_adc_to_temp = dc_xpwr_get_batt_temp;
#ifdef CONFIG_BATTERY_TYPE
#if defined(CONFIG_BATTERY_BT_B0BFQ)
	pdata.design_cap = 7135;
	pdata.design_min_volt = 3400;
	pdata.design_max_volt = 4200;
	pdata.max_temp = 50;
	pdata.min_temp = -5;
#elif defined(CONFIG_BATTERY_BT_E002H)
	pdata.design_cap = 4500;
	pdata.design_min_volt = 3400;
	pdata.design_max_volt = 4350;
	pdata.max_temp = 46;
	pdata.min_temp = 0;
#endif
#else
	pdata.design_cap = 3724;
	pdata.design_min_volt = 3400;
	pdata.design_max_volt = 4350;
	pdata.max_temp = 55;
	pdata.min_temp = 0;
#endif
	return &pdata;
}

void *dollarcove_fg_pdata(void *info)
{
	return get_platform_data();
}
