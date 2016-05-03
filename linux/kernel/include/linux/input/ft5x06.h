#ifndef _EDT_FT5X06_H
#define _EDT_FT5X06_H

/*
 * Copyright (c) 2012 Simon Budig, <simon.budig@kernelconcepts.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

struct ft5x06_platform_data {
	int irq_pin;
	int reset_pin;

	/* startup defaults for operational parameters */
	bool use_parameters;
	u8 gain;
	u8 threshold;
	u8 offset;
	u8 report_rate;
	u16 x_max;
	u16 y_max;
};

#define FT5X06_RESET_PIN (102+26) //GPIO_S0_NC[26]
#define FT5X06_INT_PIN   (130+3) //GPIO_S5[3]
#ifdef CONFIG_MRD8
#define X_MAX	800
#define Y_MAX	1280
#else
#define X_MAX	1280
#define Y_MAX	800
#endif
#endif /* _EDT_FT5X06_H */
