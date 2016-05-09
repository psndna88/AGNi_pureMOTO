/* drivers/input/misc/ak0991x_input.h - AK0991X compass driver
 *
 * Copyright (C) 2012 ASAHI KASEI MICRODEVICES CORPORATION.
 *
 * Authors: Rikita Yamada <yamada.rj (at) om.asahi-kasei.co.jp>
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
#ifndef _AK0991X_INPUT_H_
#define _AK0991X_INPUT_H_

struct device;
struct ak0991x_data;

struct ak0991x_bus_ops {
	u16 bustype;
	int (*rxdata)(struct device *dev, unsigned char *rxdata, int length);
	int (*txdata)(struct device *dev, unsigned char *txdata, int length);
};

int ak0991x_suspend(struct ak0991x_data *akm);
int ak0991x_resume(struct ak0991x_data *akm);
struct ak0991x_data *ak0991x_probe(struct device *dev, int irq,
		const struct ak0991x_bus_ops *bops);
int ak0991x_remove(struct ak0991x_data *akm);

#endif
