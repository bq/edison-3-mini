/* include/linux/cm3232.h
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

#ifndef __LINUX_CM3232_H
#define __LINUX_CM3232_H

#define CM3232_I2C_NAME	"cm3232"

#define	CM3232_ADDR		0x10

/*cm3232*/
#define CM3232_ALS_CMD	0
#define CM3232_ALS_DATA	0x50

/*for ALS command*/
#define CM3232_ALS_RESET 	    (1 << 6)
#define CM3232_ALS_IT_100MS 	(0 << 2)
#define CM3232_ALS_IT_200MS 	(1 << 2)
#define CM3232_ALS_IT_400MS 	(2 << 2)
#define CM3232_ALS_IT_800MS 	(3 << 2)
#define CM3232_ALS_IT_1600MS 	(4 << 2)
#define CM3232_ALS_IT_3200MS 	(5 << 2)
#define CM3232_ALS_HS_HIGH		(1 << 1)
#define CM3232_ALS_SD			(1 << 0)

#define LS_PWR_ON				(1 << 0)

struct cm3232_platform_data {
	int (*power)(int, uint8_t); /* power to the chip */
};

#endif
