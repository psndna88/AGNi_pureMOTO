/* drivers/input/misc/ak0991x.c - AK0991X compass driver
 *
 * Copyright (C) 2014 ASAHI KASEI MICRODEVICES CORPORATION.
 * Copyright (C) 2012 Sony Mobile Communications AB.
 *
 * Authors: Rikita Yamada <yamada.rj (at) om.asahi-kasei.co.jp>
 *          Takashi Shiina <takashi.shiina (at) sonymobile.com>
 *          Masashi Shimizu <Masashi.X.Shimizu (at) sonymobile.com>
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

/*#define DEBUG*/
/*#define VERBOSE_DEBUG*/

#include <linux/ak0991x.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include "ak0991x_input.h"

/* Read/Write buffer size.*/
#define RWBUF_SIZE		16
#define AK0991X_NUM_AXIS	3

/* Setting for CNTL1 */
#define AK0991X_CNTL_NSF	0x40
/* Setting for CNTL2 */
#define AK0991X_CNTL_PDN	0x00
#define AK0991X_CNTL_SNG	0x01
#define AK0991X_CNTL_CNT(CNT_FREQ)	(((CNT_FREQ) << 1) & 0x0E)
#define AK0991X_CNTL_TEST	0x10
#define AK0991X_CNTL_FUSE	0x1F
/* Setting for CNTL3 */
#define AK0991X_CNTL_RESET	0x01

/* Other parameters */
#define AK0991X_COMPANY_ID	0x48
#define AK09911_DEVICE_ID	0x05
#define AK09912_DEVICE_ID	0x04
#define AK09913_DEVICE_ID	0x08
#define AK09915_DEVICE_ID	0x10
#define AK09916_DEVICE_ID	0x09
#define AK0991X_DATA_READY	0x01
#define AK0991X_CNT_FREQ_10HZ	1
#define AK0991X_CNT_FREQ_20HZ	2
#define AK0991X_CNT_FREQ_50HZ	3
#define AK0991X_CNT_FREQ_100HZ	4

/* Register Address and parameters */
#define AK0991X_ADDR_WIA	0x00
#define AK0991X_WIA_SIZE	4
#define AK0991X_ADDR_ST1	0x10
#define AK0991X_DATA_SIZE	9
#define AK0991X_ADDR_FUSE	0x60
#define AK0991X_FUSE_SIZE	(AK0991X_NUM_AXIS)
#define AK0991X_FUSE_DUMMY	(128)
#define AK0991X_FUSE_060Q16	(39322) /*0.6  in Q16 format */
#define AK0991X_FUSE_015Q16	(9830)  /*0.15  in Q16 format */
#define AK0991X_ADDR_CNTL1	0x30
#define AK0991X_ADDR_CNTL2	0x31
#define AK0991X_ADDR_CNTL3	0x32

#define AK0991X_NSF(NSF)	(((NSF) & 0x03) << 5)
#define AK0991X_DEFAULT_NSF	2

/* Driver definition */
#define UCHAR_MIN	0
#define UCHAR_MAX	255
#define AKM_MAG_MAXVAL  32768
#define AKM_MAG_MINVAL  -32768

#define AK0991X_WAIT_TIME_MS	10
#define AK0991X_CNTL_DELAY_US	100
#define AK0991X_POR_DELAY_US	100

#define AK0991X_INFO_SIZE ( \
		AK0991X_WIA_SIZE + AK0991X_FUSE_SIZE + \
		AK0991X_NUM_AXIS + AK0991X_NUM_AXIS)

#define IS_PDN(mode)    (!(mode))
#define IS_SNG(mode)    ((mode) == AK0991X_CNTL_SNG)
#define IS_CNT(mode)    (((mode) & (0x0E)) && !((mode) & (0xF1)))
#define IS_TST(mode)    ((mode) == AK0991X_CNTL_TEST)
#define IS_FUS(mode)    ((mode) == AK0991X_CNTL_FUSE)

/* #define USE_DELAY_WORK */

struct ak0991x_data {
	struct device			*dev;
	struct input_dev		*input;
	struct delayed_work		dwork;
#ifndef USE_DELAY_WORK
	struct workqueue_struct	*work_queue;
	struct hrtimer			poll_timer;
#endif

	/* This value represents current operation mode.
	 * 0x00: power down mode
	 * 0x01: single measurement mode
	 * 0x02: continuous mode @ 10Hz (interval 100ms)
	 * 0x04: continuous mode @ 20Hz (interval  50ms)
	 * 0x06: continuous mode @ 50Hz (interval  20ms)
	 * 0x08: continuous mode @ 100Hz (interval 10ms)
	 * 0x10: self-test mode
	 */
	atomic_t	mode;
	atomic_t	mode_rsv;
	/* Software continuous measurement interval in millisecond */
	/* negative: not set,  0 or positive: set */
	atomic_t	interval;
	atomic_t	interval_rsv;
	/* Self-test result flag.
	 *  0: not done yet.
	 *  1: success.
	 * -1: error.
	 */
	atomic_t	selftest;
	/* NSF flag.
	 *  0 or +: valid.
	 * -1: This device does not have NSF.
	 */
	atomic_t	cntl1;
	/* IRQ number.  0:not set,  <0:set. */
	int		irq;
	/* A buffer to save FUSE ROM value */
	unsigned char	fuse[AK0991X_FUSE_SIZE];
	/* scale factor for raw data */
	s32		raw_to_micro_q16[AK0991X_NUM_AXIS];
	/* Axis conversion */
	unsigned char	axis_order[AK0991X_NUM_AXIS];
	unsigned char	axis_sign[AK0991X_NUM_AXIS];
	/* A reserved device id */
	unsigned char	company_id;
	unsigned char	device_id;

	const struct ak0991x_bus_ops *bops;

	struct ak0991x_platform_data *pdata;
};

/*
 * This driver has 3 measurement mode.
 * 1. single measurement (or One Shot measurement).
 *    Measure only once. When measuremnt is done, status changes to
 *    powerdown mode automatically.
 * 2. tm_continuous measurement.
 *    Continuous measurement is impremented using timer function.
 *    Resolution of interval depends on timer resolution of system.
 * 3. continuous measurement.
 *    Continuous measuremnt using hardware (device own) function.
 *    Interval value is discrete value which depends on the device.
 * 4. self test mode.
 *    Same as single measurement.
 *
 * Each condition is as follows.
 *   Single        : ((mode == CNTL_SNG)  && (interval < 0)).
 *   tm_continuous : ((mode == CNTL_SNG)  && (interval >= 0)).
 *   Continuous    : ((mode == CNTL_CNT)  && (interval >= 0)).
 *   Self-test     : ((mode == CNTL_TEST) && (interval < 0)).
 */
static int akecs_busy_check(struct ak0991x_data *akm)
{
	if (AK0991X_CNTL_PDN != atomic_read(&akm->mode))
		return 1;
	if (0 <= atomic_read(&akm->interval))
		return 1;
	return 0;
}

static int akecs_setmode_measure(struct ak0991x_data *akm,
		unsigned char mode, int interval)
{
	unsigned char buffer[3];
	int cntl1;
	int len;
	int request_timer;
	int err = 0;

	dev_dbg(akm->dev, "%s called: mode=%d, interval=%d",
			__func__, mode, interval);

	/* Check setting value */
	if (IS_SNG(mode)) {
		if ((!akm->irq) || (0 <= interval))
			/* 'tm_continuous' or 'single wo drdy' */
			request_timer = 1;
		else
			/* 'single with drdy' */
			request_timer = 0;
	} else if (IS_CNT(mode)) {
		/* interval should be positive value. */
		if (0 > interval)
			return -EINVAL;
		if (!akm->irq)
			/* continuous wo drdy */
			request_timer = 1;
		else
			/* continuous with drdy */
			request_timer = 0;
	} else if (IS_TST(mode)) {
		/* interval should be negative value. */
		if (0 <= interval)
			return -EINVAL;
		if (!akm->irq)
			/* self-test wo drdy */
			request_timer = 1;
		else
			/* continuous with drdy */
			request_timer = 0;
	} else
		return -EINVAL;

	if (0 <= atomic_read(&akm->interval))
		return -EBUSY;

	/* set mode */
	if (atomic_cmpxchg(&akm->mode,
				AK0991X_CNTL_PDN, mode)
			!= AK0991X_CNTL_PDN) {
		dev_err(akm->dev, "%s: device is busy", __func__);
		return -EBUSY;
	}
	/* set interval */
	atomic_set(&akm->interval, interval);

	if (akm->input->users) {
		cntl1 = atomic_read(&akm->cntl1);
		if (cntl1 >= 0) {
			buffer[0] = AK0991X_ADDR_CNTL1;
			buffer[1] = (unsigned char)cntl1;
			buffer[2] = mode;
			len = 3;
		} else {
			buffer[0] = AK0991X_ADDR_CNTL2;
			buffer[1] = mode;
			len = 2;
		}

		err = akm->bops->txdata(akm->dev, buffer, len);
		if (err) {
			/* Error recovery */
			atomic_set(&akm->mode, AK0991X_CNTL_PDN);
			atomic_set(&akm->interval, -1);
			return err;
		}

		/* set timer for wo DRDY mode */
		if (request_timer)
#ifndef USE_DELAY_WORK
			hrtimer_start(&akm->poll_timer,
					ns_to_ktime(interval * 1000000),
					HRTIMER_MODE_REL);
#else
			schedule_delayed_work(
				&akm->dwork,
				msecs_to_jiffies(AK0991X_WAIT_TIME_MS));
#endif
	} else {
		/* reserve current mode & interval for when driver is activated */
		atomic_set(&akm->mode_rsv, atomic_read(&akm->mode));
		atomic_set(&akm->interval_rsv, atomic_read(&akm->interval));
		atomic_set(&akm->mode, AK0991X_CNTL_PDN);
		atomic_set(&akm->interval, -1);
	}

	return err;
}

/* If power down mode is set while DRDY is HIGH,
  (i.e. before work que function read out the measurement data)
  DRDY bit is reset to 0, then work que function will fail.*/
static int akecs_setmode_powerdown(struct ak0991x_data *akm)
{
	unsigned char buffer[2];
	int err;

	dev_dbg(akm->dev, "%s called", __func__);

	if (0 <= atomic_read(&akm->interval)) {
#ifndef USE_DELAY_WORK
		hrtimer_cancel(&akm->poll_timer);
		cancel_work_sync(&akm->dwork.work);
#else
		cancel_delayed_work_sync(&akm->dwork);
#endif
		atomic_set(&akm->interval, -1);
	}

	buffer[0] = AK0991X_ADDR_CNTL2;
	buffer[1] = AK0991X_CNTL_PDN;

	err = akm->bops->txdata(akm->dev, buffer, sizeof(buffer));
	if (err)
		dev_err(akm->dev, "%s: Can not set to powerdown.",
			__func__);

	udelay(AK0991X_CNTL_DELAY_US);

	atomic_set(&akm->mode, AK0991X_CNTL_PDN);
	return err;
}

/* Reset function is also same behavior as powerdown mode */
static int akecs_setmode_reset(struct ak0991x_data *akm)
{
	unsigned char buffer[2];
	int err;

	dev_dbg(akm->dev, "%s called", __func__);

	if (0 <= atomic_read(&akm->interval)) {
#ifndef USE_DELAY_WORK
		hrtimer_cancel(&akm->poll_timer);
		cancel_work_sync(&akm->dwork.work);
#else
		cancel_delayed_work_sync(&akm->dwork);
#endif
		atomic_set(&akm->interval, -1);
	}

	buffer[0] = AK0991X_ADDR_CNTL3;
	buffer[1] = AK0991X_CNTL_RESET;

	err = akm->bops->txdata(akm->dev, buffer, sizeof(buffer));
	if (err)
		dev_err(akm->dev, "%s: Can not reset the device.",
			__func__);

	/* Wait for bootup the device */
	udelay(AK0991X_POR_DELAY_US);

	atomic_set(&akm->mode, AK0991X_CNTL_PDN);
	atomic_set(&akm->mode_rsv, AK0991X_CNTL_PDN);
	atomic_set(&akm->interval, -1);
	atomic_set(&akm->interval_rsv, -1);
	return err;
}

/* No busy check */
/* This function is called only once in initialization. */
static int akecs_checkdevice(struct ak0991x_data *akm)
{
	unsigned char buffer[AK0991X_WIA_SIZE];
	int err;

	dev_dbg(akm->dev, "%s called", __func__);

	buffer[0] = AK0991X_ADDR_WIA;
	err = akm->bops->rxdata(akm->dev, buffer, AK0991X_WIA_SIZE);
	if (err) {
		dev_err(akm->dev, "%s: Can not read WIA.", __func__);
		goto CHECKDEVICE_ERR;
	}

	/* Reserve device id */
	akm->company_id = buffer[0];
	akm->device_id = buffer[1];

	if (akm->company_id != AK0991X_COMPANY_ID) {
		dev_err(akm->dev, "%s: The device is not AKM.", __func__);
		err = -ENXIO;
		goto CHECKDEVICE_ERR;
	}

	switch (akm->device_id) {
	case AK09912_DEVICE_ID:
	case AK09915_DEVICE_ID:
		atomic_set(&akm->cntl1, AK0991X_NSF(AK0991X_DEFAULT_NSF));
		break;
	case AK09911_DEVICE_ID:
	case AK09913_DEVICE_ID:
	case AK09916_DEVICE_ID:
		/* AK0991x: nop */
		break;
	default:
		/* Other: error */
		dev_err(akm->dev, "%s: This device is not supported.\n",
				__func__);
		err = -ENXIO;
		goto CHECKDEVICE_ERR;
	}
	return 0;

CHECKDEVICE_ERR:
	/* Reset read IDs */
	akm->company_id = 0;
	akm->device_id = 0;
	return err;
}

/* No busy check */
/* This function is called only once in initialization. */
static int akecs_read_fuse(struct ak0991x_data *akm)
{
	unsigned char buffer[2];
	int err;
	int i;

	dev_dbg(akm->dev, "%s called", __func__);

	/* Some device does not have FUSEROM. */
	if ((AK09913_DEVICE_ID == akm->device_id) ||
		(AK09915_DEVICE_ID == akm->device_id) ||
		(AK09916_DEVICE_ID == akm->device_id))  {
		for (i = 0; i < AK0991X_NUM_AXIS; i++) {
			akm->raw_to_micro_q16[i] = AK0991X_FUSE_015Q16;
			akm->fuse[i] = AK0991X_FUSE_DUMMY;
		}
		return 0;
	}

	buffer[0] = AK0991X_ADDR_CNTL2;
	buffer[1] = AK0991X_CNTL_FUSE;
	err = akm->bops->txdata(akm->dev, buffer, sizeof(buffer));
	if (err) {
		dev_err(akm->dev, "%s: Can not set to fuse access mode.",
			__func__);
		return err;
	}

	akm->fuse[0] = AK0991X_ADDR_FUSE;
	err = akm->bops->rxdata(akm->dev, akm->fuse, AK0991X_FUSE_SIZE);
	if (err) {
		dev_err(akm->dev, "%s: Can not read the FUSEROM.", __func__);
		return err;
	}

	if (AK09911_DEVICE_ID == akm->device_id) {
		for (i = 0; i < AK0991X_NUM_AXIS; i++)
			akm->raw_to_micro_q16[i] = ((akm->fuse[i] + 128)
					* AK0991X_FUSE_060Q16) >> 7;
	} else if (AK09912_DEVICE_ID == akm->device_id)  {
		for (i = 0; i < AK0991X_NUM_AXIS; i++)
			akm->raw_to_micro_q16[i] = ((akm->fuse[i] + 128)
					* AK0991X_FUSE_015Q16) >> 8;
	}

	dev_dbg(akm->dev, "%s: FUSE = 0x%02x, 0x%02x, 0x%02x", __func__,
		akm->fuse[0],
		akm->fuse[1],
		akm->fuse[2]);
	dev_dbg(akm->dev, "%s: R2M  = %d, %d, %d", __func__,
		akm->raw_to_micro_q16[0],
		akm->raw_to_micro_q16[1],
		akm->raw_to_micro_q16[2]);

	return akecs_setmode_powerdown(akm);
}

static s64 get_time_ns(void)
{
	struct timespec ts;
	/*ts = ktime_to_timespec(alarm_get_elapsed_realtime());*/
	get_monotonic_boottime(&ts);
	return timespec_to_ns(&ts);
}

static int read_and_event(struct ak0991x_data *akm)
{
	unsigned char buffer[AK0991X_DATA_SIZE];
	int err;
	int mode;
	int i;
	s16 val;
	s16 val_st2;
	s32 tmp32[3];
	s32 val32[3];
	s32 val_tsl;
	s32 val_tsh;
	s64 timestamp;

	/* Get timestamp at first */
	timestamp = get_time_ns();
	val_tsl = (s32)((u64)timestamp & 0xffffffff);
	val_tsh = (s32)((u64)timestamp >> 32);

	dev_dbg(akm->dev, "%s called", __func__);
	buffer[0] = AK0991X_ADDR_ST1;
	err = akm->bops->rxdata(akm->dev, buffer, AK0991X_DATA_SIZE);
	if (err)
		return err;

	/* Check ST bit */
	if ((buffer[0] & AK0991X_DATA_READY) != AK0991X_DATA_READY) {
		dev_err(akm->dev, "%s: DRDY is not set", __func__);
	//	return -EAGAIN;
	}

	mode = atomic_read(&akm->mode);

	/* If operation is already canceled, don't report values */
	if (mode == AK0991X_CNTL_PDN) {
		dev_info(akm->dev, "%s: Operation canceled.", __func__);
		return 0;
	}

	/* If one shot operation, set to powerdown state */
	if (IS_SNG(mode) || IS_TST(mode))
		/* Device automatically goes to PDN. */
		/* No need to set device. */
		atomic_set(&akm->mode, AK0991X_CNTL_PDN);

	/* report axis data: HXL & HXH / HYL & HYH / HZL & HZH */
	for (i = 0; i < AK0991X_NUM_AXIS; i++) {
		val = (s16)(((u16)buffer[i * 2 + 2] << 8)
				| (u16)buffer[i * 2 + 1]);
		tmp32[i] = (s32)val * akm->raw_to_micro_q16[i];
	}
	val_st2 = (s16)buffer[8];

	dev_vdbg(akm->dev, "hval = %d,%d,%d: ST2 = 0x%02X TS = %lld",
		tmp32[0], tmp32[1], tmp32[2], val_st2, timestamp);

	/* If operation is self-test mode, don't report input */
	if (mode == AK0991X_CNTL_TEST) {
		/* TODO: selft test judge */
		return 0;
	}

	/* Axis conversion */
	for (i = 0; i < AK0991X_NUM_AXIS; i++) {
		val32[i] = tmp32[akm->axis_order[i]];
		if (akm->axis_sign[i])
			val32[i] *= -1;
	}
	input_event(akm->input, EV_MSC, MSC_RX,  val32[0]);
	input_event(akm->input, EV_MSC, MSC_RY,  val32[1]);
	input_event(akm->input, EV_MSC, MSC_RZ,  val32[2]);
	input_event(akm->input, EV_MSC, MSC_ST2, val_st2);
	input_event(akm->input, EV_MSC, MSC_TSL, val_tsl);
	input_event(akm->input, EV_MSC, MSC_TSH, val_tsh);
	input_sync(akm->input);

	return 0;
}

static irqreturn_t ak0991x_irq(int irq, void *handle)
{
	struct ak0991x_data *akm = handle;

	dev_dbg(akm->dev, "%s called", __func__);

	read_and_event(akm);
	return IRQ_HANDLED;
}

static void ak0991x_tm_continuous(struct work_struct *work)
{
	struct ak0991x_data *akm =
		container_of(work, struct ak0991x_data, dwork.work);
	unsigned char buffer[2];
	int interval;

	dev_dbg(akm->dev, "%s called", __func__);

	/* Timer Event is used when driver status is as follows:
	 * 1. Single/Test without DRDY (SNG/TEST, irq<0, interval<0)
	 *    a. notify measurement data
	 * 2. tm_continuous with DRDY (SNG, irq>0, interval>0)
	 *    b. start timer with 'interval' value
	 *    c. start next measurement,
	 * 3. tm_continuous without DRDY (SNG, irq<0, interval>0)
	 *    a. notify measurement data
	 *    b. start timer with 'interval' value
	 *    c. start next measurement,
	 * 4. Continuous without DRDY (CNT, irq<0, interval>0)
	 *    a. notify measurement
	 *    b. start timer with 'interval' value
	 *
	 * Note that 'Single with DRDY' and 'Continuous with DRDY' mode
	 * does not use timer. Required operation is done in 'ak0991x_irq'
	 * function.
	 */
	if (!akm->irq)
		/* a. notify measurement data */
		if (read_and_event(akm))
			goto TRY_AGAIN;

	interval = atomic_read(&akm->interval);
	if (0 <= interval) {
#ifndef USE_DELAY_WORK
#else
		/* b. start timer with 'interval' value */
		schedule_delayed_work(&akm->work,
				msecs_to_jiffies(interval));
#endif
		/* tm_continuous mode sets mode to powerdown every measurement.
		 * Therefore, it should be power-down mode
		 * before start new measurement.
		 */

		if (atomic_cmpxchg(&akm->mode, AK0991X_CNTL_PDN,
				AK0991X_CNTL_SNG) == AK0991X_CNTL_PDN) {
			/* c. start next measurement, */
			buffer[0] = AK0991X_ADDR_CNTL2;
			buffer[1] = AK0991X_CNTL_SNG;
			akm->bops->txdata(akm->dev, buffer, sizeof(buffer));
		}
	}
	return;


TRY_AGAIN:
#ifndef USE_DELAY_WORK
#else
	/* To repair quickly, use shorter interval */
	schedule_delayed_work(&akm->work,
			msecs_to_jiffies(AK0991X_WAIT_TIME_MS));
#endif
	return;
}

#ifndef USE_DELAY_WORK
static enum hrtimer_restart akm_timer_func(struct hrtimer *timer)
{
	struct ak0991x_data *akm =
		container_of(timer, struct ak0991x_data, poll_timer);
	int interval;

	interval = atomic_read(&akm->interval);
	queue_work(akm->work_queue, &akm->dwork.work);
	hrtimer_forward_now(&akm->poll_timer,
			ns_to_ktime(interval * 1000000));

	return HRTIMER_RESTART;
}
#endif

static int ak0991x_device_power_on(struct ak0991x_data *akm)
{
	int ret = 0;

	dev_dbg(akm->dev, "%s called", __func__);

	/* TODO: platform data will not be able to use with device tree */
	if (akm->pdata)
		if (akm->pdata->power_on) {
			ret = akm->pdata->power_on();
			udelay(AK0991X_POR_DELAY_US);
		}

	return ret;
}

static void ak0991x_device_power_off(struct ak0991x_data *akm)
{
	dev_dbg(akm->dev, "%s called", __func__);

	/* TODO: platform data will not be able to use with device tree */
	if (akm->pdata)
		if (akm->pdata->power_off)
			akm->pdata->power_off();
}

static int ak0991x_open(struct ak0991x_data *akm)
{
	int mode;
	int interval;
	int ret;

	dev_dbg(akm->dev, "%s called", __func__);

	ret = ak0991x_device_power_on(akm);
	if (ret)
		return ret;

	if (akm->irq)
		enable_irq(akm->irq);

	/* restore reserved parameter */
	mode = atomic_read(&akm->mode_rsv);
	interval = atomic_read(&akm->interval_rsv);
	if ((0 <= interval) && (IS_SNG(mode) || IS_CNT(mode)))
		ret = akecs_setmode_measure(akm, mode, interval);

	return ret;
}

static int ak0991x_close(struct ak0991x_data *akm)
{
	int ret;

	dev_dbg(akm->dev, "%s called", __func__);

	if (akm->irq)
		disable_irq(akm->irq);

	/* reserve current parameter */
	atomic_set(&akm->mode_rsv, atomic_read(&akm->mode));
	atomic_set(&akm->interval_rsv, atomic_read(&akm->interval));

	/* stop measurement */
	ret = akecs_setmode_powerdown(akm);
	if (ret)
		return ret;

	ak0991x_device_power_off(akm);

	return 0;
}

int ak0991x_resume(struct ak0991x_data *akm)
{
	dev_dbg(akm->dev, "%s called", __func__);

	if (akm->input->users)
		ak0991x_open(akm);

	return 0;
}

int ak0991x_suspend(struct ak0991x_data *akm)
{
	dev_dbg(akm->dev, "%s called", __func__);

	if (akm->input->users)
		ak0991x_close(akm);

	return 0;
}

/*****************************************************************************
 *
 * SysFS attribute functions
 *
 * files :
 *  - interval   [rw] [t] : store measurement interval (millisecond)
 *  - nsf        [rw] [t] : set digital filter (from 0 to 3)
 *  - selftest   [rw] [t] : device's self test mode
 *  - single     [w]  [t] : single-shot trigger
 *  - continuous [w]  [t] : continuous measurement (interval ms)
 *  - reset      [w]  [t] : soft reset
 *  - info       [r]  [b] : get device part no. and ASA value.
 *
 * debug mode only:
 *  - debug      [r]  [t] : get driver status.
 *
 * [b] = binary format
 * [t] = text format
 *
 */

/*********** interval (TEXT) ***********/
static ssize_t attr_interval_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct ak0991x_data *akm = dev_get_drvdata(dev);
	dev_dbg(dev, "%s called\n", __func__);
	return scnprintf(buf, PAGE_SIZE, "%d", atomic_read(&akm->interval));
}

static ssize_t attr_interval_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct ak0991x_data *akm = dev_get_drvdata(dev);
	int err;
	long interval = 0;

	dev_dbg(dev, "%s called: '%s'(%d)", __func__, buf, count);

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	if (kstrtol(buf, 10, &interval))
		return -EINVAL;

	/* normalize interval value */
	if ((0 <= interval) && (interval < AK0991X_WAIT_TIME_MS))
		interval = AK0991X_WAIT_TIME_MS;
	if (0 > interval)
		interval = -1;

	/* Stop current measurement at first */
	if (akecs_busy_check(akm)) {
		err = akecs_setmode_powerdown(akm);
		if (err < 0)
			return err;
	}

	if (0 <= interval) {
		err = akecs_setmode_measure(akm, AK0991X_CNTL_SNG, interval);
		if (err)
			return err;
	} else
		err = akecs_setmode_powerdown(akm);

	return count;
}

/*********** nsf (TEXT) ***********/
static ssize_t attr_nsf_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct ak0991x_data *akm = dev_get_drvdata(dev);
	uint8_t read_cntl;

	dev_dbg(dev, "%s called", __func__);

	if ((akm->device_id != AK09912_DEVICE_ID) ||
		(akm->device_id != AK09915_DEVICE_ID))
		return -ENXIO;

	read_cntl = (uint8_t)atomic_read(&akm->cntl1);

	read_cntl = ((read_cntl >> 5) & 0x03);
	return sprintf(buf, "%d", read_cntl);
}

static ssize_t attr_nsf_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct ak0991x_data *akm = dev_get_drvdata(dev);
	long nsf;

	dev_dbg(dev, "%s called: '%s'(%d)", __func__, buf, count);

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	if ((akm->device_id != AK09912_DEVICE_ID) ||
		(akm->device_id != AK09915_DEVICE_ID))
		return -ENXIO;

	if (kstrtol(buf, 10, &nsf))
		return -EINVAL;

	if ((nsf < 0) || (3 < nsf))
		return -EINVAL;

	atomic_set(&akm->cntl1, AK0991X_NSF(nsf));

	return count;
}

/*********** selftest (TEXT) ***********/
static ssize_t attr_selftest_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct ak0991x_data *akm = dev_get_drvdata(dev);
	dev_dbg(dev, "%s called", __func__);
	return scnprintf(buf, PAGE_SIZE, "%d", atomic_read(&akm->selftest));
}

static ssize_t attr_selftest_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct ak0991x_data *akm = dev_get_drvdata(dev);
	int err;

	dev_dbg(dev, "%s called: '%s'(%d)", __func__, buf, count);

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	/* Just start self-test operation */
	/* Self-test will not raise event */
	err = akecs_setmode_measure(akm, AK0991X_CNTL_TEST, -1);
	if (err)
		return err;

	return count;
}

/*********** single (TEXT) ***********/
static ssize_t attr_single_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct ak0991x_data *akm = dev_get_drvdata(dev);
	int err;

	dev_dbg(dev, "%s called: '%s'(%d)", __func__, buf, count);

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	err = akecs_setmode_measure(akm, AK0991X_CNTL_SNG, -1);
	if (err)
		return err;

	return count;
}

/*********** continuous (TEXT) ***********/
static ssize_t attr_continuous_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct ak0991x_data *akm = dev_get_drvdata(dev);
	long interval;
	int next_mode;
	int next_interval;
	int err;

	dev_dbg(dev, "%s called: '%s'(%d)", __func__, buf, count);

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	if (kstrtol(buf, 10, &interval))
		return -EINVAL;

	if (0 > interval) {
		next_mode = AK0991X_CNTL_PDN;
		next_interval = -1;
	} else if (20 > interval) {
		/* the fastest */
		next_mode = AK0991X_CNTL_CNT(AK0991X_CNT_FREQ_100HZ);
		next_interval = 10;
	} else if (50 > interval) {
		next_mode = AK0991X_CNTL_CNT(AK0991X_CNT_FREQ_50HZ);
		next_interval = 20;
	} else if (100 > interval) {
		next_mode = AK0991X_CNTL_CNT(AK0991X_CNT_FREQ_20HZ);
		next_interval = 50;
	} else {
		/* the slowest */
		next_mode = AK0991X_CNTL_CNT(AK0991X_CNT_FREQ_10HZ);
		next_interval = 100;
	}

	/* Stop current measurement at first */
	if (akecs_busy_check(akm)) {
		err = akecs_setmode_powerdown(akm);
		if (err < 0)
			return err;
	}

	if (AK0991X_CNTL_PDN == next_mode)
		err = akecs_setmode_powerdown(akm);
	else
		err = akecs_setmode_measure(akm, next_mode, next_interval);

	if (err < 0)
		return err;

	return count;
}

/*********** reset (TEXT) ***********/
static ssize_t attr_reset_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct ak0991x_data *akm = dev_get_drvdata(dev);
	int err;

	/* TODO: hard reset will be implemented? */

	dev_dbg(dev, "%s called: '%s'(%d)", __func__, buf, count);

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	/* soft reset */
	err = akecs_setmode_reset(akm);

	if (err)
		return err;

	return count;
}

#ifdef DEBUG
/*********** debug (TEXT) ***********/
static ssize_t attr_axismap_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct ak0991x_data *akm = dev_get_drvdata(dev);
	dev_dbg(dev, "%s called", __func__);

	return scnprintf(buf, PAGE_SIZE, "%d,%d,%d,%d,%d,%d",
			akm->axis_order[0], akm->axis_order[1],
			akm->axis_order[2],
			akm->axis_sign[0], akm->axis_sign[1],
			akm->axis_sign[2]);
}

/* Format: order[0],order[1],order[2],sign[0],sign[1],sign[2] */
/* 'order' should be one of 0,1,2. sign should be 0 or 1. */
static ssize_t attr_axismap_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct ak0991x_data *akm = dev_get_drvdata(dev);
	char cp_buffer[2];
	unsigned char tmp_order[3];
	unsigned char tmp_sign[3];
	long val;
	int i;

	dev_dbg(dev, "%s called: '%s'(%d)", __func__, buf, count);

	if (NULL == buf)
		return -EINVAL;

	if (0 == count)
		return 0;

	/* this function access buf[10] */
	if (11 > count)
		return -EINVAL;

	/* initialize buffer */
	cp_buffer[0] = cp_buffer[1] = '\0';

	for (i = 0; i < AK0991X_NUM_AXIS; i++) {
		/* read axis order */
		cp_buffer[0] = buf[i * 2];
		if (kstrtol(cp_buffer, 10, &val))
			return -EINVAL;
		if (0 > val || 2 < val)
			return -EINVAL;
		tmp_order[i] = val;

		/* read axis sign */
		cp_buffer[0] = buf[i * 2 + 6];
		if (kstrtol(cp_buffer, 10, &val))
			return -EINVAL;
		if (0 > val || 1 < val)
			return -EINVAL;
		tmp_sign[i] = val;
	}
	/* Copy */
	for (i = 0; i < AK0991X_NUM_AXIS; i++) {
		akm->axis_order[i] = tmp_order[i];
		akm->axis_sign[i] = tmp_sign[i];
	}

	return count;
}

static ssize_t attr_debug_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct ak0991x_data *akm = dev_get_drvdata(dev);
	char *curbuf;
	int curlen;
	int wrlen;
	dev_dbg(dev, "%s called", __func__);

	curbuf = buf;
	curlen = PAGE_SIZE;
	wrlen = scnprintf(curbuf, curlen, "irq: %d\n",
			akm->irq);
	if (wrlen < 0)
		return wrlen;

	curbuf += wrlen;
	curlen -= wrlen;
	wrlen = scnprintf(curbuf, curlen, "device id: %d\n",
			akm->device_id);
	if (wrlen < 0)
		return wrlen;

	curbuf += wrlen;
	curlen -= wrlen;
	wrlen = scnprintf(curbuf, curlen, "current mode: %d\n",
			atomic_read(&akm->mode));
	if (wrlen < 0)
		return wrlen;

	curbuf += wrlen;
	curlen -= wrlen;
	wrlen = scnprintf(curbuf, curlen, "reserved mode: %d\n",
			atomic_read(&akm->mode_rsv));
	if (wrlen < 0)
		return wrlen;

	curbuf += wrlen;
	curlen -= wrlen;
	wrlen = scnprintf(curbuf, curlen, "interval : %d\n",
			atomic_read(&akm->interval));
	if (wrlen < 0)
		return wrlen;

	return (ssize_t)(curbuf + wrlen - buf);
}
#endif

/*********** registers (BINARY) ***********/
static ssize_t bin_attr_info_read(
		struct file *file,
		struct kobject *kobj,
		struct bin_attribute *attr,
		char *buf, loff_t pos, size_t size)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct ak0991x_data *akm = dev_get_drvdata(dev);
	unsigned char info[AK0991X_INFO_SIZE];
	unsigned char n;

	dev_dbg(dev, "%s called", __func__);

	/* copy company/device id from cache */
	info[0] = akm->company_id;
	info[1] = akm->device_id;

	/* copy ASA value from cache */
	n = AK0991X_WIA_SIZE;
	memcpy(&info[n], akm->fuse, AK0991X_FUSE_SIZE);
	/* copy info */
	n += AK0991X_FUSE_SIZE;
	memcpy(&info[n], akm->axis_order, AK0991X_NUM_AXIS);
	n += AK0991X_NUM_AXIS;
	memcpy(&info[n], akm->axis_sign, AK0991X_NUM_AXIS);

	if (AK0991X_INFO_SIZE > size)
		memcpy(buf, info, size);
	else
		memcpy(buf, info, AK0991X_INFO_SIZE);

	return (size < AK0991X_INFO_SIZE) ? size : AK0991X_INFO_SIZE;
}

#ifndef __BIN_ATTR
#define __BIN_ATTR(_name, _mode, _read, _write, _size) {   \
	.attr = { .name = __stringify(_name), .mode = _mode }, \
	.read   = _read,  \
	.write  = _write, \
	.size   = _size,  \
}
#endif

static struct device_attribute ak0991x_attributes[] = {
	__ATTR(interval,   0660, attr_interval_show, attr_interval_store),
	__ATTR(nsf,        0660, attr_nsf_show,      attr_nsf_store),
	__ATTR(selftest,   0660, attr_selftest_show, attr_selftest_store),
	__ATTR(single,     0220, NULL,               attr_single_store),
	__ATTR(continuous, 0220, NULL,               attr_continuous_store),
	__ATTR(reset,      0220, NULL,               attr_reset_store),
#ifdef DEBUG
	__ATTR(axismap,    0660, attr_axismap_show,  attr_axismap_store),
	__ATTR(debug,      0400, attr_debug_show,    NULL),
#endif
};

static struct bin_attribute ak0991x_bin_attributes[] = {
	__BIN_ATTR(info, 0440, bin_attr_info_read, NULL, AK0991X_INFO_SIZE),
};

static int create_sysfs_interfaces(struct ak0991x_data *akm)
{
	int i, j;
	struct device *dev = &akm->input->dev;

	for (i = 0; i < ARRAY_SIZE(ak0991x_attributes); i++)
		if (device_create_file(dev, ak0991x_attributes + i))
			goto device_create_file_failed;

	for (j = 0; j < ARRAY_SIZE(ak0991x_bin_attributes); j++)
		if (sysfs_create_bin_file(&dev->kobj,
			ak0991x_bin_attributes + j))
			goto device_create_bin_file_failed;

	return 0;

device_create_bin_file_failed:
	for (--j; j >= 0; j--)
		sysfs_remove_bin_file(&dev->kobj, ak0991x_bin_attributes + j);
	dev_err(dev, "%s: failed to create binary sysfs interface.", __func__);
device_create_file_failed:
	for (--i; i >= 0; i--)
		device_remove_file(dev, ak0991x_attributes + i);
	dev_err(dev, "%s: failed to create sysfs interface.", __func__);
	return -ENODEV;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ak0991x_attributes); i++)
		device_remove_file(dev, ak0991x_attributes + i);

	for (i = 0; i < ARRAY_SIZE(ak0991x_bin_attributes); i++)
		sysfs_remove_bin_file(&dev->kobj, ak0991x_bin_attributes + i);

}

static int ak0991x_input_open(struct input_dev *dev)
{
	struct ak0991x_data *akm = input_get_drvdata(dev);

	dev_dbg(&dev->dev, "%s called", __func__);

	return ak0991x_open(akm);
}

static void ak0991x_input_close(struct input_dev *dev)
{
	struct ak0991x_data *akm = input_get_drvdata(dev);

	dev_dbg(&dev->dev, "%s called", __func__);

	ak0991x_close(akm);
}

static void ak0991x_set_axis(struct ak0991x_data *akm)
{
	struct device_node *node = akm->dev->of_node;
	int i;

	if (node) {
		/* Get from device node */
		/* parameters are declared as 'unsigned char' */
		if (of_property_read_u8(node, "axis_order_x",
					&akm->axis_order[0]) != 0)
			goto SET_DEFAULT_AXIS;
		if (of_property_read_u8(node, "axis_order_y",
					&akm->axis_order[1]) != 0)
			goto SET_DEFAULT_AXIS;
		if (of_property_read_u8(node, "axis_order_z",
					&akm->axis_order[2]) != 0)
			goto SET_DEFAULT_AXIS;
		if (of_property_read_u8(node, "axis_sign_x",
					&akm->axis_sign[0]) != 0)
			goto SET_DEFAULT_AXIS;
		if (of_property_read_u8(node, "axis_sign_y",
					&akm->axis_sign[1]) != 0)
			goto SET_DEFAULT_AXIS;
		if (of_property_read_u8(node, "axis_sign_z",
					&akm->axis_sign[2]) != 0)
			goto SET_DEFAULT_AXIS;
	} else {
		/* platform data is NULL, use default value. */
		if (!akm->pdata)
			goto SET_DEFAULT_AXIS;

		/* Get from pdata */
		for (i = 0; i < 3; i++) {
			akm->axis_order[i] = akm->pdata->axis_order[i];
			akm->axis_sign[i] = akm->pdata->axis_sign[i];
		}
	}

	dev_dbg(akm->dev, "%s : axis=[%d,%d,%d] sign=[%d,%d,%d]", __func__,
		akm->axis_order[0], akm->axis_order[1], akm->axis_order[2],
		akm->axis_sign[0], akm->axis_sign[1], akm->axis_sign[2]);
	return;

SET_DEFAULT_AXIS:
	dev_err(akm->dev, "%s: Axis info read failed. Use default value.",
			__func__);
	/* set default axis value */
	for (i = 0; i < 3; i++) {
		akm->axis_order[i] = i;
		akm->axis_sign[i] = 0;
	}
}

struct ak0991x_data *ak0991x_probe(struct device *dev, int irq,
		const struct ak0991x_bus_ops *bops)
{
	struct ak0991x_data *akm;
	int err = 0;
	/* platform data will not be able to use with device tree */

	dev_dbg(dev, "%s called", __func__);

	akm = kzalloc(sizeof(struct ak0991x_data), GFP_KERNEL);
	if (!akm) {
		dev_err(dev, "%s: memory allocation failed.", __func__);
		err = -ENOMEM;
		goto err_kzalloc;
	}
	/* Copy pointer */
	akm->dev = dev;
	akm->bops = bops;
	/* Initialize static variables. */
	atomic_set(&akm->mode, AK0991X_CNTL_PDN);
	atomic_set(&akm->mode_rsv, AK0991X_CNTL_PDN);
	atomic_set(&akm->interval, -1);
	atomic_set(&akm->interval_rsv, -1);
	atomic_set(&akm->selftest, 0);
	atomic_set(&akm->cntl1, -1);
	akm->irq = irq;
	akm->pdata = dev->platform_data;
	/* Set axis info */
	ak0991x_set_axis(akm);

	err = ak0991x_device_power_on(akm);
	if (err) {
		dev_err(dev, "%s: device power on failed.", __func__);
		goto err_power_on;
	}

	err = akecs_checkdevice(akm);
	if (err < 0) {
		ak0991x_device_power_off(akm);
		goto err_check_device;
	}

	err = akecs_read_fuse(akm);
	if (err < 0) {
		ak0991x_device_power_off(akm);
		goto err_read_fuse;
	}

	ak0991x_device_power_off(akm);

	akm->input = input_allocate_device();
	if (!akm->input) {
		err = -ENOMEM;
		dev_err(dev, "%s: Failed to allocate input device.", __func__);
		goto err_input_allocate_device;
	}

	input_set_drvdata(akm->input, akm);

	set_bit(EV_MSC, akm->input->evbit);
	akm->input->id.product = akm->device_id;
	akm->input->id.bustype = bops->bustype;
	akm->input->open = ak0991x_input_open;
	akm->input->close = ak0991x_input_close;
	akm->input->name = AKM_DRIVER_NAME;
	input_set_capability(akm->input, EV_MSC, MSC_RX);
	input_set_capability(akm->input, EV_MSC, MSC_RY);
	input_set_capability(akm->input, EV_MSC, MSC_RZ);
	input_set_capability(akm->input, EV_MSC, MSC_ST2);
	input_set_capability(akm->input, EV_MSC, MSC_TSL);
	input_set_capability(akm->input, EV_MSC, MSC_TSH);

	err = input_register_device(akm->input);
	if (err) {
		input_free_device(akm->input);
		dev_err(dev, "%s: Unable to register input device.", __func__);
		goto err_input_register_device;
	}

#ifndef USE_DELAY_WORK
	hrtimer_init(&akm->poll_timer, CLOCK_BOOTTIME,
			HRTIMER_MODE_REL);
	akm->poll_timer.function = akm_timer_func;
	akm->work_queue = alloc_workqueue("akm_poll_work",
			WQ_UNBOUND | WQ_MEM_RECLAIM | WQ_HIGHPRI, 1);
	INIT_WORK(&akm->dwork.work, ak0991x_tm_continuous);
#else
	INIT_DELAYED_WORK(&akm->dwork, ak0991x_tm_continuous);
#endif

	if (akm->irq) {
		err = request_threaded_irq(akm->irq, NULL, ak0991x_irq,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				dev_name(akm->dev), akm);
		if (err) {
			dev_err(dev, "%s: request irq failed.", __func__);
			goto err_request_th_irq;
		}
		disable_irq(akm->irq);
	}

	err = create_sysfs_interfaces(akm);
	if (err) {
		dev_err(dev, "%s: create sysfs failed.", __func__);
		goto err_create_sysfs_interfaces;
	}

	dev_info(dev, "successfully probed (irq=%d, id=%d).",
			akm->irq, akm->device_id);
	return 0;

err_create_sysfs_interfaces:
	if (akm->irq)
		free_irq(akm->irq, akm);
err_request_th_irq:
	input_unregister_device(akm->input);
err_input_register_device:
err_input_allocate_device:
err_read_fuse:
err_check_device:
err_power_on:
	kfree(akm);
err_kzalloc:
	return ERR_PTR(err);
}
EXPORT_SYMBOL_GPL(ak0991x_probe);

int ak0991x_remove(struct ak0991x_data *akm)
{
	dev_dbg(akm->dev, "%s called", __func__);
	remove_sysfs_interfaces(&akm->input->dev);
	if (akm->irq)
		free_irq(akm->irq, akm);
	input_unregister_device(akm->input);
	ak0991x_device_power_off(akm);
	dev_info(akm->dev, "successfully removed.");
	kfree(akm);
	return 0;
}
EXPORT_SYMBOL_GPL(ak0991x_remove);

MODULE_AUTHOR("AKM & SOMC");
MODULE_DESCRIPTION("AK0991x compass driver");
MODULE_LICENSE("GPL");
