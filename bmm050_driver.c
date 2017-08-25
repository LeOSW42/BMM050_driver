/*!
 * @section LICENSE
 * (C) Copyright 2013 Bosch Sensortec GmbH All Rights Reserved
 *
 * This software program is licensed subject to the GNU General
 * Public License (GPL).Version 2,June 1991,
 * available at http://www.fsf.org/copyleft/gpl.html
 *
 * @filename    bmm050_driver.c
 * @date        2014/03/11
 * @id          "7bf4b97"
 * @version     v2.8
 *
 * @brief       BMM050 Linux Driver
 */

#include <linux/version.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/delay.h>

#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/unistd.h>
#include <linux/types.h>
#include <linux/string.h>
#else
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#endif


#include "bmm050.h"
#include "bs_log.h"

/* sensor specific */
#define SENSOR_NAME "bmm050"

#define SENSOR_CHIP_ID_BMM (0x32)
#define CHECK_CHIP_ID_TIME_MAX   5

#define BMM_REG_NAME(name) BMM050_##name
#define BMM_VAL_NAME(name) BMM050_##name
#define BMM_CALL_API(name) bmm050_##name

#define BMM_SPI_WRITE_DELAY_TIME 5

#define BMM_DEFAULT_REPETITION_XY BMM_VAL_NAME(REGULAR_REPXY)
#define BMM_DEFAULT_REPETITION_Z BMM_VAL_NAME(REGULAR_REPZ)
#define BMM_DEFAULT_ODR BMM_VAL_NAME(REGULAR_DR)
/* generic */
#define BMM_MAX_RETRY_SPI_XFER (100)
#define BMM_MAX_RETRY_WAKEUP (5)
#define BMM_MAX_RETRY_WAIT_DRDY (100)

#define BMM_DELAY_MIN (1)
#define BMM_DELAY_DEFAULT (200)

#define MAG_VALUE_MAX (32767)
#define MAG_VALUE_MIN (-32768)

#define BYTES_PER_LINE (16)

#define BMM_SELF_TEST 1
#define BMM_ADV_TEST 2

#define BMM_OP_MODE_UNKNOWN (-1)

/*! Bosch sensor unknown place*/
#define BOSCH_SENSOR_PLACE_UNKNOWN (-1)
/*! Bosch sensor remapping table size P0~P7*/
#define MAX_AXIS_REMAP_TAB_SZ 8

struct bosch_sensor_specific {
	char *name;
	/* 0 to 7 */
	unsigned int place:3;
	int irq;
	int (*irq_gpio_cfg)(void);
};

/*!
 * we use a typedef to hide the detail,
 * because this type might be changed
 */
struct bosch_sensor_axis_remap {
	/* src means which source will be mapped to target x, y, z axis */
	/* if an target OS axis is remapped from (-)x,
	 * src is 0, sign_* is (-)1 */
	/* if an target OS axis is remapped from (-)y,
	 * src is 1, sign_* is (-)1 */
	/* if an target OS axis is remapped from (-)z,
	 * src is 2, sign_* is (-)1 */
	int src_x:3;
	int src_y:3;
	int src_z:3;

	int sign_x:2;
	int sign_y:2;
	int sign_z:2;
};

struct bosch_sensor_data {
	union {
		int16_t v[3];
		struct {
			int16_t x;
			int16_t y;
			int16_t z;
		};
	};
};

static const u8 odr_map[] = {10, 2, 6, 8, 15, 20, 25, 30};
static const long op_mode_maps[] = {
	BMM_VAL_NAME(NORMAL_MODE),
	BMM_VAL_NAME(FORCED_MODE),
	BMM_VAL_NAME(SUSPEND_MODE),
	BMM_VAL_NAME(SLEEP_MODE)
};


struct bmm_device_data {
	struct bmm050 device;
	struct spi_device *spi;
	struct input_dev *input;
	struct delayed_work work;

	atomic_t delay;
	/* whether the system in suspend state */
	atomic_t in_suspend;

	struct bmm050_mdata_s32 value;
	u8 enable:1;
	s8 op_mode:4;
	u8 odr;
	u8 rept_xy;
	u8 rept_z;

	s16 result_test;

	struct mutex mutex_power_mode;

	/* controls not only reg, but also workqueue */
	struct mutex mutex_op_mode;
	struct mutex mutex_enable;
	struct mutex mutex_odr;
	struct mutex mutex_rept_xy;
	struct mutex mutex_rept_z;

	struct mutex mutex_value;
};

static struct spi_device *bmm_device;
/* spi operation for API */
static void bmm_delay(u32 msec);
static int bmm_spi_read(struct spi_device *spi, u8 reg_addr,
		u8 *data, u8 len);
static int bmm_spi_write(struct spi_device *spi, u8 reg_addr,
		u8 *data, u8 len);

static void bmm_dump_reg(struct spi_device *spi);
static int bmm_wakeup(struct spi_device *spi);
static int bmm_check_chip_id(struct spi_device *spi);

static int bmm_pre_suspend(struct spi_device *spi);
static int bmm_post_resume(struct spi_device *spi);

static int bmm_restore_hw_cfg(struct spi_device *spi);

static const struct bosch_sensor_axis_remap
bst_axis_remap_tab_dft[MAX_AXIS_REMAP_TAB_SZ] = {
	/* src_x src_y src_z  sign_x  sign_y  sign_z */
	{  0,    1,    2,     1,      1,      1 }, /* P0 */
	{  1,    0,    2,     1,     -1,      1 }, /* P1 */
	{  0,    1,    2,    -1,     -1,      1 }, /* P2 */
	{  1,    0,    2,    -1,      1,      1 }, /* P3 */

	{  0,    1,    2,    -1,      1,     -1 }, /* P4 */
	{  1,    0,    2,    -1,     -1,     -1 }, /* P5 */
	{  0,    1,    2,     1,     -1,     -1 }, /* P6 */
	{  1,    0,    2,     1,      1,     -1 }, /* P7 */
};

static void bst_remap_sensor_data(struct bosch_sensor_data *data,
		const struct bosch_sensor_axis_remap *remap)
{
	struct bosch_sensor_data tmp;

	tmp.x = data->v[remap->src_x] * remap->sign_x;
	tmp.y = data->v[remap->src_y] * remap->sign_y;
	tmp.z = data->v[remap->src_z] * remap->sign_z;

	memcpy(data, &tmp, sizeof(*data));
}

static void bmm_remap_sensor_data(struct bmm050_mdata_s32 *val,
		struct bmm_device_data *device_data)
{
	(void)val;
	(void)device_data;
}

static int bmm_check_chip_id(struct spi_device *spi)
{
	int err = -1;
	u8 chip_id = 0;
	u8 read_count = 0;

	while (read_count++ < CHECK_CHIP_ID_TIME_MAX) {
		bmm_spi_read(spi, BMM_REG_NAME(CHIP_ID), &chip_id, 1);
		PINFO("read chip id result: %#x", chip_id);

		if ((chip_id & 0xff) != SENSOR_CHIP_ID_BMM) {
			mdelay(1);
		} else {
			err = 0;
			break;
		}
	}

	return err;
}

static void bmm_delay(u32 msec)
{
	mdelay(msec);
}

static inline int bmm_get_forced_drdy_time(int rept_xy, int rept_z)
{
	return  (145 * rept_xy + 500 * rept_z + 980 + (1000 - 1)) / 1000;
}


static void bmm_dump_reg(struct spi_device *spi)
{
#ifdef DEBUG
	int i;
	u8 dbg_buf[64];
	u8 dbg_buf_str[64 * 3 + 1] = "";

	for (i = 0; i < BYTES_PER_LINE; i++) {
		dbg_buf[i] = i;
		sprintf(dbg_buf_str + i * 3, "%02x%c",
				dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	printk(KERN_DEBUG "%s\n", dbg_buf_str);

	bmm_spi_read(spi, BMM_REG_NAME(CHIP_ID), dbg_buf, 64);
	for (i = 0; i < 64; i++) {
		sprintf(dbg_buf_str + i * 3, "%02x%c",
				dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	printk(KERN_DEBUG "%s\n", dbg_buf_str);
#endif
}

static int bmm_wakeup(struct spi_device *spi)
{
	int err = 0;
	int try_times = BMM_MAX_RETRY_WAKEUP;
	const u8 value = 0x01;
	u8 dummy;

	PINFO("waking up the chip...");

	mdelay(BMM_SPI_WRITE_DELAY_TIME);
	while (try_times) {
		err = bmm_spi_write(spi,
				BMM_REG_NAME(POWER_CNTL), (u8 *)&value, 1);
		mdelay(BMM_SPI_WRITE_DELAY_TIME);
		dummy = 0;
		err = bmm_spi_read(spi, BMM_REG_NAME(POWER_CNTL), &dummy, 1);
		if (value == dummy)
			break;

		try_times--;
	}

	PINFO("wake up result: %s, tried times: %d",
			(try_times > 0) ? "succeed" : "fail",
			BMM_MAX_RETRY_WAKEUP - try_times + 1);

	err = (try_times > 0) ? 0 : -1;

	return err;
}

/*spi read routine for API*/
static int bmm_spi_read(struct spi_device *spi, u8 reg_addr,
		u8 *data, u8 len)
{
	s32 dummy;
	u8 frame = 0;

	frame = 0x80 | reg_addr;

	dummy = spi_write_then_read(spi, &frame, sizeof(frame), data, sizeof(*data) * len);
	if (dummy < 0)
		return -1;
	udelay(2);
	return 0;
}

/*spi write routine for */
static int bmm_spi_write(struct spi_device *spi, u8 reg_addr,
		u8 *data, u8 len)
{
	s32 dummy;
	u16 frame = 0;
	u8 i;

	for(i=0 ; i<len ; i++)
	{
		frame = 0x7F & (((u16)reg_addr << 8) | (u16)*data);
		reg_addr++;

		dummy = spi_write(spi, &frame, sizeof(frame));
		if (dummy < 0)
			return -1;
		udelay(2);
	}

	return 0;
}

static int bmm_spi_read_wrapper(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	int err = 0;
	err = bmm_spi_read(bmm_device, reg_addr, data, len);
	return err;
}

static int bmm_spi_write_wrapper(u8 dev_addr, u8 reg_addr, u8 *data, u8 len)
{
	int err = 0;
	err = bmm_spi_write(bmm_device, reg_addr, data, len);
	return err;
}

/* this function exists for optimization of speed,
 * because it is frequently called */
static inline int bmm_set_forced_mode(struct spi_device *spi)
{
	int err = 0;

	/* FORCED_MODE */
	const u8 value = 0x02;
	err = bmm_spi_write(spi, BMM_REG_NAME(CONTROL), (u8 *)&value, 1);

	return err;
}

static void bmm_work_func(struct work_struct *work)
{
	struct bmm_device_data *device_data =
		container_of((struct delayed_work *)work,
			struct bmm_device_data, work);
	struct spi_device *spi = device_data->spi;
	unsigned long delay =
		msecs_to_jiffies(atomic_read(&device_data->delay));

	mutex_lock(&device_data->mutex_value);

	mutex_lock(&device_data->mutex_op_mode);
	if (BMM_VAL_NAME(NORMAL_MODE) != device_data->op_mode)
		bmm_set_forced_mode(spi);

	mutex_unlock(&device_data->mutex_op_mode);

	BMM_CALL_API(read_mdataXYZ_s32)(&device_data->value);
	bmm_remap_sensor_data(&device_data->value, device_data);

	input_report_abs(device_data->input, ABS_X, device_data->value.datax);
	input_report_abs(device_data->input, ABS_Y, device_data->value.datay);
	input_report_abs(device_data->input, ABS_Z, device_data->value.dataz);
	mutex_unlock(&device_data->mutex_value);

	input_sync(device_data->input);

	schedule_delayed_work(&device_data->work, delay);
}


static int bmm_set_odr(struct spi_device *spi, u8 odr)
{
	int err = 0;

	err = BMM_CALL_API(set_datarate)(odr);

	return err;
}

static int bmm_get_odr(struct spi_device *spi, u8 *podr)
{
	int err = 0;
	u8 value;

	err = BMM_CALL_API(get_datarate)(&value);
	if (!err)
		*podr = value;

	return err;
}

static ssize_t bmm_show_chip_id(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", SENSOR_CHIP_ID_BMM);
}

static ssize_t bmm_show_op_mode(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_device_data *device_data = input_get_drvdata(input);
	u8 op_mode = 0xff;
	u8 power_mode;

	mutex_lock(&device_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		mutex_lock(&device_data->mutex_op_mode);
		BMM_CALL_API(get_functional_state)(&op_mode);
		mutex_unlock(&device_data->mutex_op_mode);
	} else {
		op_mode = BMM_VAL_NAME(SUSPEND_MODE);
	}

	mutex_unlock(&device_data->mutex_power_mode);

	PDEBUG("op_mode: %d", op_mode);

	ret = sprintf(buf, "%d\n", op_mode);

	return ret;
}


static inline int bmm_get_op_mode_idx(u8 op_mode)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(op_mode_maps); i++) {
		if (op_mode_maps[i] == op_mode)
			break;
	}

	if (i < ARRAY_SIZE(op_mode_maps))
		return i;
	else
		return -1;
}


static int bmm_set_op_mode(struct bmm_device_data *device_data, int op_mode)
{
	int err = 0;

	err = BMM_CALL_API(set_functional_state)(
			op_mode);

	if (BMM_VAL_NAME(SUSPEND_MODE) == op_mode)
		atomic_set(&device_data->in_suspend, 1);
	else
		atomic_set(&device_data->in_suspend, 0);

	return err;
}

static ssize_t bmm_store_op_mode(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int err = 0;
	int i;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_device_data *device_data = input_get_drvdata(input);
	struct spi_device *spi = device_data->spi;
	long op_mode;

	err = kstrtoul(buf, 10, &op_mode);
	if (err)
		return err;

	mutex_lock(&device_data->mutex_power_mode);

	i = bmm_get_op_mode_idx(op_mode);

	if (i != -1) {
		mutex_lock(&device_data->mutex_op_mode);
		if (op_mode != device_data->op_mode) {
			if (BMM_VAL_NAME(FORCED_MODE) == op_mode) {
				/* special treat of forced mode
				 * for optimization */
				err = bmm_set_forced_mode(spi);
			} else {
				err = bmm_set_op_mode(device_data, op_mode);
			}

			if (!err) {
				if (BMM_VAL_NAME(FORCED_MODE) == op_mode)
					device_data->op_mode =
						BMM_OP_MODE_UNKNOWN;
				else
					device_data->op_mode = op_mode;
			}
		}
		mutex_unlock(&device_data->mutex_op_mode);
	} else {
		err = -EINVAL;
	}

	mutex_unlock(&device_data->mutex_power_mode);

	if (err)
		return err;
	else
		return count;
}

static ssize_t bmm_show_odr(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_device_data *device_data = input_get_drvdata(input);
	struct spi_device *spi = device_data->spi;
	int err;
	u8 power_mode;

	mutex_lock(&device_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		mutex_lock(&device_data->mutex_odr);
		err = bmm_get_odr(spi, &data);
		mutex_unlock(&device_data->mutex_odr);
	} else {
		err = -EIO;
	}
	mutex_unlock(&device_data->mutex_power_mode);

	if (!err) {
		if (data < ARRAY_SIZE(odr_map))
			err = sprintf(buf, "%d\n", odr_map[data]);
		else
			err = -EINVAL;
	}

	return err;
}

static ssize_t bmm_store_odr(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp;
	unsigned char data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_device_data *device_data = input_get_drvdata(input);
	struct spi_device *spi = device_data->spi;
	u8 power_mode;
	int i;

	err = kstrtoul(buf, 10, &tmp);
	if (err)
		return err;

	if (tmp > 255)
		return -EINVAL;

	data = (unsigned char)tmp;

	mutex_lock(&device_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		for (i = 0; i < ARRAY_SIZE(odr_map); i++) {
			if (odr_map[i] == data)
				break;
		}

		if (i < ARRAY_SIZE(odr_map)) {
			mutex_lock(&device_data->mutex_odr);
			err = bmm_set_odr(spi, i);
			if (!err)
				device_data->odr = i;

			mutex_unlock(&device_data->mutex_odr);
		} else {
			err = -EINVAL;
		}
	} else {
		err = -EIO;
	}

	mutex_unlock(&device_data->mutex_power_mode);
	if (err)
		return err;

	return count;
}

static ssize_t bmm_show_rept_xy(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_device_data *device_data = input_get_drvdata(input);
	int err;
	u8 power_mode;

	mutex_lock(&device_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		mutex_lock(&device_data->mutex_rept_xy);
		err = BMM_CALL_API(get_repetitions_XY)(&data);
		mutex_unlock(&device_data->mutex_rept_xy);
	} else {
		err = -EIO;
	}

	mutex_unlock(&device_data->mutex_power_mode);

	if (err)
		return err;

	return sprintf(buf, "%d\n", data);
}

static ssize_t bmm_store_rept_xy(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_device_data *device_data = input_get_drvdata(input);
	int err;
	u8 data;
	u8 power_mode;

	err = kstrtoul(buf, 10, &tmp);
	if (err)
		return err;

	if (tmp > 255)
		return -EINVAL;

	data = (unsigned char)tmp;

	mutex_lock(&device_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		mutex_lock(&device_data->mutex_rept_xy);
		err = BMM_CALL_API(set_repetitions_XY)(data);
		if (!err) {
			mdelay(BMM_SPI_WRITE_DELAY_TIME);
			device_data->rept_xy = data;
		}
		mutex_unlock(&device_data->mutex_rept_xy);
	} else {
		err = -EIO;
	}
	mutex_unlock(&device_data->mutex_power_mode);

	if (err)
		return err;

	return count;
}

static ssize_t bmm_show_rept_z(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	unsigned char data = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_device_data *device_data = input_get_drvdata(input);
	int err;
	u8 power_mode;

	mutex_lock(&device_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		mutex_lock(&device_data->mutex_rept_z);
		err = BMM_CALL_API(get_repetitions_Z)(&data);
		mutex_unlock(&device_data->mutex_rept_z);
	} else {
		err = -EIO;
	}

	mutex_unlock(&device_data->mutex_power_mode);

	if (err)
		return err;

	return sprintf(buf, "%d\n", data);
}

static ssize_t bmm_store_rept_z(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long tmp = 0;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_device_data *device_data = input_get_drvdata(input);
	int err;
	u8 data;
	u8 power_mode;

	err = kstrtoul(buf, 10, &tmp);
	if (err)
		return err;

	if (tmp > 255)
		return -EINVAL;

	data = (unsigned char)tmp;

	mutex_lock(&device_data->mutex_power_mode);
	BMM_CALL_API(get_powermode)(&power_mode);
	if (power_mode) {
		mutex_lock(&device_data->mutex_rept_z);
		err = BMM_CALL_API(set_repetitions_Z)(data);
		if (!err) {
			mdelay(BMM_SPI_WRITE_DELAY_TIME);
			device_data->rept_z = data;
		}
		mutex_unlock(&device_data->mutex_rept_z);
	} else {
		err = -EIO;
	}
	mutex_unlock(&device_data->mutex_power_mode);

	if (err)
		return err;

	return count;
}


static ssize_t bmm_show_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmm_device_data *device_data = input_get_drvdata(input);
	int count;
	struct bmm050_mdata_s32 value = {0, 0, 0, 0, 0};

	BMM_CALL_API(read_mdataXYZ_s32)(&value);
	if (value.drdy) {
		bmm_remap_sensor_data(&value, device_data);
		device_data->value = value;
	} else
		PERR("data not ready");

	count = sprintf(buf, "%d %d %d\n",
			device_data->value.datax,
			device_data->value.datay,
			device_data->value.dataz);
	PDEBUG("%d %d %d",
			device_data->value.datax,
			device_data->value.datay,
			device_data->value.dataz);

	return count;
}


static ssize_t bmm_show_value_raw(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct bmm050_mdata value;
	int count;

	BMM_CALL_API(get_raw_xyz)(&value);

	count = sprintf(buf, "%hd %hd %hd\n",
			value.datax,
			value.datay,
			value.dataz);

	return count;
}


static ssize_t bmm_show_enable(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmm_device_data *device_data = input_get_drvdata(input);
	int err;

	mutex_lock(&device_data->mutex_enable);
	err = sprintf(buf, "%d\n", device_data->enable);
	mutex_unlock(&device_data->mutex_enable);
	return err;
}

static ssize_t bmm_store_enable(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_device_data *device_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;

	data = data ? 1 : 0;
	mutex_lock(&device_data->mutex_enable);
	if (data != device_data->enable) {
		if (data) {
			schedule_delayed_work(
					&device_data->work,
					msecs_to_jiffies(atomic_read(
							&device_data->delay)));
		} else {
			cancel_delayed_work_sync(&device_data->work);
		}

		device_data->enable = data;
	}
	mutex_unlock(&device_data->mutex_enable);

	return count;
}

static ssize_t bmm_show_delay(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmm_device_data *device_data = input_get_drvdata(input);

	return sprintf(buf, "%d\n", atomic_read(&device_data->delay));

}

static ssize_t bmm_store_delay(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_device_data *device_data = input_get_drvdata(input);

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;

	if (data == 0) {
		err = -EINVAL;
		return err;
	}

	if (data < BMM_DELAY_MIN)
		data = BMM_DELAY_MIN;

	atomic_set(&device_data->delay, data);

	return count;
}

static ssize_t bmm_show_test(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct input_dev *input = to_input_dev(dev);
	struct bmm_device_data *device_data = input_get_drvdata(input);
	int err;

	err = sprintf(buf, "%d\n", device_data->result_test);
	return err;
}

static ssize_t bmm_store_test(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int err;
	struct input_dev *input = to_input_dev(dev);
	struct bmm_device_data *device_data = input_get_drvdata(input);
	struct spi_device *spi = device_data->spi;
	u8 dummy;

	err = kstrtoul(buf, 10, &data);
	if (err)
		return err;

	/* the following code assumes the work thread is not running */
	if (BMM_SELF_TEST == data) {
		/* self test */
		err = bmm_set_op_mode(device_data, BMM_VAL_NAME(SLEEP_MODE));
		mdelay(3);
		err = BMM_CALL_API(set_selftest)(1);
		mdelay(3);
		err = BMM_CALL_API(get_self_test_XYZ)(&dummy);
		device_data->result_test = dummy;
	} else if (BMM_ADV_TEST == data) {
		/* advanced self test */
		err = BMM_CALL_API(perform_advanced_selftest)(
				&device_data->result_test);
	} else {
		err = -EINVAL;
	}

	if (!err) {
		BMM_CALL_API(soft_reset)();
		mdelay(BMM_SPI_WRITE_DELAY_TIME);
		bmm_restore_hw_cfg(spi);
	}

	if (err)
		count = -1;

	return count;
}


static ssize_t bmm_show_reg(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int err = 0;
	int i;
	u8 dbg_buf[64];
	u8 dbg_buf_str[64 * 3 + 1] = "";
	struct input_dev *input = to_input_dev(dev);
	struct bmm_device_data *device_data = input_get_drvdata(input);
	struct spi_device *spi = device_data->spi;

	for (i = 0; i < BYTES_PER_LINE; i++) {
		dbg_buf[i] = i;
		sprintf(dbg_buf_str + i * 3, "%02x%c",
				dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	memcpy(buf, dbg_buf_str, BYTES_PER_LINE * 3);

	for (i = 0; i < BYTES_PER_LINE * 3 - 1; i++)
		dbg_buf_str[i] = '-';

	dbg_buf_str[i] = '\n';
	memcpy(buf + BYTES_PER_LINE * 3, dbg_buf_str, BYTES_PER_LINE * 3);


	bmm_spi_read(spi, BMM_REG_NAME(CHIP_ID), dbg_buf, 64);
	for (i = 0; i < 64; i++) {
		sprintf(dbg_buf_str + i * 3, "%02x%c",
				dbg_buf[i],
				(((i + 1) % BYTES_PER_LINE == 0) ? '\n' : ' '));
	}
	memcpy(buf + BYTES_PER_LINE * 3 + BYTES_PER_LINE * 3,
			dbg_buf_str, 64 * 3);

	err = BYTES_PER_LINE * 3 + BYTES_PER_LINE * 3 + 64 * 3;
	return err;
}


static ssize_t bmm_show_place(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int place = BOSCH_SENSOR_PLACE_UNKNOWN;

	return sprintf(buf, "%d\n", place);
}


static DEVICE_ATTR(chip_id, S_IRUGO,
		bmm_show_chip_id, NULL);
static DEVICE_ATTR(op_mode, S_IRUGO|S_IWUSR,
		bmm_show_op_mode, bmm_store_op_mode);
static DEVICE_ATTR(odr, S_IRUGO|S_IWUSR,
		bmm_show_odr, bmm_store_odr);
static DEVICE_ATTR(rept_xy, S_IRUGO|S_IWUSR,
		bmm_show_rept_xy, bmm_store_rept_xy);
static DEVICE_ATTR(rept_z, S_IRUGO|S_IWUSR,
		bmm_show_rept_z, bmm_store_rept_z);
static DEVICE_ATTR(value, S_IRUGO,
		bmm_show_value, NULL);
static DEVICE_ATTR(value_raw, S_IRUGO,
		bmm_show_value_raw, NULL);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR,
		bmm_show_enable, bmm_store_enable);
static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR,
		bmm_show_delay, bmm_store_delay);
static DEVICE_ATTR(test, S_IRUGO|S_IWUSR,
		bmm_show_test, bmm_store_test);
static DEVICE_ATTR(reg, S_IRUGO,
		bmm_show_reg, NULL);
static DEVICE_ATTR(place, S_IRUGO,
		bmm_show_place, NULL);

static struct attribute *bmm_attributes[] = {
	&dev_attr_chip_id.attr,
	&dev_attr_op_mode.attr,
	&dev_attr_odr.attr,
	&dev_attr_rept_xy.attr,
	&dev_attr_rept_z.attr,
	&dev_attr_value.attr,
	&dev_attr_value_raw.attr,
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_test.attr,
	&dev_attr_reg.attr,
	&dev_attr_place.attr,
	NULL
};


static struct attribute_group bmm_attribute_group = {
	.attrs = bmm_attributes
};


static int bmm_input_init(struct bmm_device_data *device_data)
{
	struct input_dev *dev;
	int err = 0;

	dev = input_allocate_device();
	if (NULL == dev)
		return -ENOMEM;

	dev->name = SENSOR_NAME;
	dev->id.bustype = BUS_SPI;

	input_set_capability(dev, EV_ABS, ABS_MISC);
	input_set_abs_params(dev, ABS_X, MAG_VALUE_MIN, MAG_VALUE_MAX, 0, 0);
	input_set_abs_params(dev, ABS_Y, MAG_VALUE_MIN, MAG_VALUE_MAX, 0, 0);
	input_set_abs_params(dev, ABS_Z, MAG_VALUE_MIN, MAG_VALUE_MAX, 0, 0);
	input_set_drvdata(dev, device_data);

	err = input_register_device(dev);
	if (err < 0) {
		input_free_device(dev);
		return err;
	}
	device_data->input = dev;

	return 0;
}

static void bmm_input_destroy(struct bmm_device_data *device_data)
{
	struct input_dev *dev = device_data->input;

	input_unregister_device(dev);
	input_free_device(dev);
}

static int bmm_restore_hw_cfg(struct spi_device *spi)
{
	int err = 0;
	u8 value;
	struct bmm_device_data *device_data =
		(struct bmm_device_data *)spi_get_drvdata(spi);
	int op_mode;

	mutex_lock(&device_data->mutex_op_mode);
	err = bmm_set_op_mode(device_data, BMM_VAL_NAME(SLEEP_MODE));

	if (bmm_get_op_mode_idx(device_data->op_mode) != -1)
		err = bmm_set_op_mode(device_data, device_data->op_mode);

	op_mode = device_data->op_mode;
	mutex_unlock(&device_data->mutex_op_mode);

	if (BMM_VAL_NAME(SUSPEND_MODE) == op_mode)
		return err;

	PINFO("app did not close this sensor before suspend");

	mutex_lock(&device_data->mutex_odr);
	BMM_CALL_API(set_datarate)(device_data->odr);
	mdelay(BMM_SPI_WRITE_DELAY_TIME);
	mutex_unlock(&device_data->mutex_odr);

	mutex_lock(&device_data->mutex_rept_xy);
	err = bmm_spi_write(spi, BMM_REG_NAME(NO_REPETITIONS_XY),
			&device_data->rept_xy, 1);
	mdelay(BMM_SPI_WRITE_DELAY_TIME);
	err = bmm_spi_read(spi, BMM_REG_NAME(NO_REPETITIONS_XY), &value, 1);
	PINFO("BMM_NO_REPETITIONS_XY: %02x", value);
	mutex_unlock(&device_data->mutex_rept_xy);

	mutex_lock(&device_data->mutex_rept_z);
	err = bmm_spi_write(spi, BMM_REG_NAME(NO_REPETITIONS_Z),
			&device_data->rept_z, 1);
	mdelay(BMM_SPI_WRITE_DELAY_TIME);
	err = bmm_spi_read(spi, BMM_REG_NAME(NO_REPETITIONS_Z), &value, 1);
	PINFO("BMM_NO_REPETITIONS_Z: %02x", value);
	mutex_unlock(&device_data->mutex_rept_z);

	mutex_lock(&device_data->mutex_op_mode);
	if (BMM_OP_MODE_UNKNOWN == device_data->op_mode) {
		bmm_set_forced_mode(spi);
		PINFO("set forced mode after hw_restore");
		mdelay(bmm_get_forced_drdy_time(device_data->rept_xy,
					device_data->rept_z));
	}
	mutex_unlock(&device_data->mutex_op_mode);


	PINFO("register dump after init");
	bmm_dump_reg(spi);

	return err;
}

static int bmm_probe(struct spi_device *spi)
{
	int err = 0;
	struct bmm_device_data *device_data = NULL;
	int dummy;

	PINFO("function entrance");

	/* wake up the chip */
	dummy = bmm_wakeup(spi);
	if (dummy < 0) {
		PERR("Cannot wake up %s, SPI xfer error", SENSOR_NAME);
		err = -EIO;
		goto exit_err_clean;
	}

	PINFO("register dump after waking up");
	bmm_dump_reg(spi);
	/* check chip id */
	err = bmm_check_chip_id(spi);
	if (!err) {
		PNOTICE("Bosch Sensortec Device %s detected", SENSOR_NAME);
	} else {
		PERR("Bosch Sensortec Device not found, chip id mismatch");
		err = -1;
		goto exit_err_clean;
	}

	device_data = kzalloc(sizeof(struct bmm_device_data), GFP_KERNEL);
	if (NULL == device_data) {
		PERR("no memory available");
		err = -ENOMEM;
		goto exit_err_clean;
	}

	spi_set_drvdata(spi, device_data);
	device_data->spi = spi;

	mutex_init(&device_data->mutex_power_mode);
	mutex_init(&device_data->mutex_op_mode);
	mutex_init(&device_data->mutex_enable);
	mutex_init(&device_data->mutex_odr);
	mutex_init(&device_data->mutex_rept_xy);
	mutex_init(&device_data->mutex_rept_z);
	mutex_init(&device_data->mutex_value);

	/* input device init */
	err = bmm_input_init(device_data);
	if (err < 0)
		goto exit_err_clean;

	/* sysfs node creation */
	err = sysfs_create_group(&device_data->input->dev.kobj,
			&bmm_attribute_group);

	if (err < 0)
		goto exit_err_sysfs;


	/* workqueue init */
	INIT_DELAYED_WORK(&device_data->work, bmm_work_func);
	atomic_set(&device_data->delay, BMM_DELAY_DEFAULT);

	/* h/w init */
	device_data->device.bus_read = bmm_spi_read_wrapper;
	device_data->device.bus_write = bmm_spi_write_wrapper;
	device_data->device.delay_msec = bmm_delay;
	BMM_CALL_API(init)(&device_data->device);

	bmm_dump_reg(spi);

	PDEBUG("trimming_reg x1: %d y1: %d x2: %d y2: %d xy1: %d xy2: %d",
			device_data->device.dig_x1,
			device_data->device.dig_y1,
			device_data->device.dig_x2,
			device_data->device.dig_y2,
			device_data->device.dig_xy1,
			device_data->device.dig_xy2);

	PDEBUG("trimming_reg z1: %d z2: %d z3: %d z4: %d xyz1: %d",
			device_data->device.dig_z1,
			device_data->device.dig_z2,
			device_data->device.dig_z3,
			device_data->device.dig_z4,
			device_data->device.dig_xyz1);

	device_data->enable = 0;
	/* now it's power on which is considered as resuming from suspend */
	device_data->op_mode = BMM_VAL_NAME(SUSPEND_MODE);
	device_data->odr = BMM_DEFAULT_ODR;
	device_data->rept_xy = BMM_DEFAULT_REPETITION_XY;
	device_data->rept_z = BMM_DEFAULT_REPETITION_Z;


	err = bmm_set_op_mode(device_data, BMM_VAL_NAME(SUSPEND_MODE));
	if (err) {
		PERR("fail to init h/w of %s", SENSOR_NAME);
		err = -EIO;
		goto exit_err_sysfs;
	}

	PNOTICE("sensor %s probed successfully", SENSOR_NAME);

	PDEBUG("spi_device: %p device_data: %p spi_device: %p input: %p",
			spi, device_data, &spi->dev, device_data->input);

	return 0;

exit_err_sysfs:
	if (err)
		bmm_input_destroy(device_data);

exit_err_clean:
	if (err) {
		if (device_data != NULL) {
			kfree(device_data);
			device_data = NULL;
		}

		bmm_device = NULL;
	}

	return err;
}

void bmm_shutdown(struct spi_device *spi)
{
	struct bmm_device_data *device_data =
		(struct bmm_device_data *)spi_get_drvdata(spi);

	mutex_lock(&device_data->mutex_power_mode);
	bmm_set_op_mode(device_data, BMM_VAL_NAME(SUSPEND_MODE));
	mutex_unlock(&device_data->mutex_power_mode);
}

static int bmm_remove(struct spi_device *spi)
{
	int err = 0;
	struct bmm_device_data *device_data =
		(struct bmm_device_data *)spi_get_drvdata(spi);

	if (NULL != device_data) {
		mutex_lock(&device_data->mutex_op_mode);
		if (BMM_VAL_NAME(NORMAL_MODE) == device_data->op_mode) {
			cancel_delayed_work_sync(&device_data->work);
			PDEBUG("cancel work");
		}
		mutex_unlock(&device_data->mutex_op_mode);

		err = bmm_set_op_mode(device_data, BMM_VAL_NAME(SUSPEND_MODE));
		mdelay(BMM_SPI_WRITE_DELAY_TIME);

		sysfs_remove_group(&device_data->input->dev.kobj,
				&bmm_attribute_group);
		bmm_input_destroy(device_data);

		kfree(device_data);

		bmm_device = NULL;
	}

	return err;
}

static const struct spi_device_id bmm_id[] = {
	{SENSOR_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(spi, bmm_id);

static struct spi_driver bmm_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = SENSOR_NAME,
	},
	.id_table = bmm_id,
	.probe = bmm_probe,
	.remove = bmm_remove,
	.shutdown = bmm_shutdown,
};

static int __init BMM_init(void)
{
	return spi_register_driver(&bmm_driver);
}

static void __exit BMM_exit(void)
{
	spi_unregister_driver(&bmm_driver);
}

MODULE_AUTHOR("contact@bosch.sensortec.com");
MODULE_DESCRIPTION("BMM MAGNETIC SENSOR DRIVER");
MODULE_LICENSE("GPL v2");

module_init(BMM_init);
module_exit(BMM_exit);
