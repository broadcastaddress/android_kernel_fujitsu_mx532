/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/isl29023.h>
#include <linux/fsl_devices.h>

#define ISL29023_DRV_NAME	"isl29023"
#define DRIVER_VERSION		"1.0"

#define ISL29023_COMMAND1		0x00
#define ISL29023_MODE_SHIFT		(5)
#define ISL29023_MODE_MASK		(0x7 << ISL29023_MODE_SHIFT)
#define ISL29023_INT_FLAG_SHIFT		(2)
#define ISL29023_INT_FLAG_MASK		(0x1 << ISL29023_INT_FLAG_SHIFT)
#define ISL29023_INT_PERSISTS_SHIFT	(0)
#define ISL29023_INT_PERSISTS_MASK	(0x3 << ISL29023_INT_PERSISTS_SHIFT)

#define ISL29023_COMMAND2		0x01
#define ISL29023_RES_SHIFT		(2)
#define ISL29023_RES_MASK		(0x3 << ISL29023_RES_SHIFT)
#define ISL29023_RANGE_SHIFT		(0)
#define ISL29023_RANGE_MASK		(0x3 << ISL29023_RANGE_SHIFT)

#define ISL29023_REG_LSB_SENSOR		0x02
#define ISL29023_REG_MSB_SENSOR		0x03
#define ISL29023_REG_IRQ_TH_LO_LSB	0x04
#define ISL29023_REG_IRQ_TH_LO_MSB	0x05
#define ISL29023_REG_IRQ_TH_HI_LSB	0x06
#define ISL29023_REG_IRQ_TH_HI_MSB	0x07

#define ISL29023_NUM_CACHABLE_REGS	8

#define ISL29023_DEFAULT_MODE  ISL29023_ALS_ONCE_MODE	// Default mode on startup

struct isl29023_data {
	struct i2c_client *client;
	struct mutex lock;
	struct input_dev *input;
	struct work_struct work;
	struct delayed_work polled_work;
	struct workqueue_struct *workqueue;
	char phys[32];
	u8 reg_cache[ISL29023_NUM_CACHABLE_REGS];
	u8 mode_before_suspend;
	u8 mode_before_interrupt;
	u8 mode_before_powerdown;
	u16 rext;
	u8 polled;
	u16 poll_interval;
};

static int gain_range[] = {
	1000, 4000, 16000, 64000
};

static int adc_resolution[] = {
	16, 12, 8, 4
};

/*
 * register access helpers
 */
static int __isl29023_read_reg(struct i2c_client *client,
			       u32 reg, u8 mask, u8 shift)
{
	struct isl29023_data *data = i2c_get_clientdata(client);
	return (data->reg_cache[reg] & mask) >> shift;
}

static int __isl29023_write_reg(struct i2c_client *client,
				u32 reg, u8 mask, u8 shift, u8 val)
{
	struct isl29023_data *data = i2c_get_clientdata(client);
	int ret = 0;
	u8 tmp;

	if (reg >= ISL29023_NUM_CACHABLE_REGS)
		return -EINVAL;

	mutex_lock(&data->lock);

	tmp = data->reg_cache[reg];
	tmp &= ~mask;
	tmp |= val << shift;

	ret = i2c_smbus_write_byte_data(client, reg, tmp);
	if (!ret)
		data->reg_cache[reg] = tmp;

	mutex_unlock(&data->lock);
	return ret;
}

/*
 * internally used functions
 */

/* interrupt persists */
static int isl29023_get_int_persists(struct i2c_client *client)
{
	return __isl29023_read_reg(client, ISL29023_COMMAND1,
		ISL29023_INT_PERSISTS_MASK, ISL29023_INT_PERSISTS_SHIFT);
}

static int isl29023_set_int_persists(struct i2c_client *client,
				int int_persists)
{
	return __isl29023_write_reg(client, ISL29023_COMMAND1,
		ISL29023_INT_PERSISTS_MASK, ISL29023_INT_PERSISTS_SHIFT,
		int_persists);
}

/* interrupt flag */
static int isl29023_get_int_flag(struct i2c_client *client)
{
	return __isl29023_read_reg(client, ISL29023_COMMAND1,
		ISL29023_INT_FLAG_MASK, ISL29023_INT_FLAG_SHIFT);
}

static int isl29023_set_int_flag(struct i2c_client *client, int flag)
{
	return __isl29023_write_reg(client, ISL29023_COMMAND1,
		ISL29023_INT_FLAG_MASK, ISL29023_INT_FLAG_SHIFT, flag);
}

/* interrupt lt */
static int isl29023_get_int_lt(struct i2c_client *client)
{
	struct isl29023_data *data = i2c_get_clientdata(client);
	int lsb, msb, lt;

	mutex_lock(&data->lock);
	lsb = i2c_smbus_read_byte_data(client, ISL29023_REG_IRQ_TH_LO_LSB);

	if (lsb < 0) {
		mutex_unlock(&data->lock);
		return lsb;
	}

	msb = i2c_smbus_read_byte_data(client, ISL29023_REG_IRQ_TH_LO_MSB);
	mutex_unlock(&data->lock);

	if (msb < 0)
		return msb;

	lt = ((msb << 8) | lsb);

	return lt;
}

static int isl29023_set_int_lt(struct i2c_client *client, int lt)
{
	int ret = 0;
	struct isl29023_data *data = i2c_get_clientdata(client);

	mutex_lock(&data->lock);
	ret = i2c_smbus_write_byte_data(client, ISL29023_REG_IRQ_TH_LO_LSB,
					lt & 0xff);
	if (ret < 0) {
		mutex_unlock(&data->lock);
		return ret;
	}

	ret = i2c_smbus_write_byte_data(client, ISL29023_REG_IRQ_TH_LO_MSB,
					(lt >> 8) & 0xff);
	if (ret < 0) {
		mutex_unlock(&data->lock);
		return ret;
	}

	data->reg_cache[ISL29023_REG_IRQ_TH_LO_MSB] = (lt >> 8) & 0xff;
	data->reg_cache[ISL29023_REG_IRQ_TH_LO_LSB] = lt & 0xff;
	mutex_unlock(&data->lock);

	return ret;
}

/* interrupt ht */
static int isl29023_get_int_ht(struct i2c_client *client)
{
	struct isl29023_data *data = i2c_get_clientdata(client);
	int lsb, msb, ht;

	mutex_lock(&data->lock);
	lsb = i2c_smbus_read_byte_data(client, ISL29023_REG_IRQ_TH_HI_LSB);

	if (lsb < 0) {
		mutex_unlock(&data->lock);
		return lsb;
	}

	msb = i2c_smbus_read_byte_data(client, ISL29023_REG_IRQ_TH_HI_MSB);
	mutex_unlock(&data->lock);

	if (msb < 0)
		return msb;

	ht = ((msb << 8) | lsb);

	return ht;
}

static int isl29023_set_int_ht(struct i2c_client *client, int ht)
{
	int ret = 0;
	struct isl29023_data *data = i2c_get_clientdata(client);

	mutex_lock(&data->lock);
	ret = i2c_smbus_write_byte_data(client, ISL29023_REG_IRQ_TH_HI_LSB,
					ht & 0xff);
	if (ret < 0) {
		mutex_unlock(&data->lock);
		return ret;
	}

	ret = i2c_smbus_write_byte_data(client, ISL29023_REG_IRQ_TH_HI_MSB,
					(ht >> 8) & 0xff);
	if (ret < 0) {
		mutex_unlock(&data->lock);
		return ret;
	}

	data->reg_cache[ISL29023_REG_IRQ_TH_HI_MSB] = (ht >> 8) & 0xff;
	data->reg_cache[ISL29023_REG_IRQ_TH_HI_LSB] = ht & 0xff;
	mutex_unlock(&data->lock);

	return ret;
}

/* range */
static int isl29023_get_range(struct i2c_client *client)
{
	return __isl29023_read_reg(client, ISL29023_COMMAND2,
		ISL29023_RANGE_MASK, ISL29023_RANGE_SHIFT);
}

static int isl29023_set_range(struct i2c_client *client, int range)
{
	return __isl29023_write_reg(client, ISL29023_COMMAND2,
		ISL29023_RANGE_MASK, ISL29023_RANGE_SHIFT, range);
}

/* resolution */
static int isl29023_get_resolution(struct i2c_client *client)
{
	return __isl29023_read_reg(client, ISL29023_COMMAND2,
		ISL29023_RES_MASK, ISL29023_RES_SHIFT);
}

static int isl29023_set_resolution(struct i2c_client *client, int res)
{
	return __isl29023_write_reg(client, ISL29023_COMMAND2,
		ISL29023_RES_MASK, ISL29023_RES_SHIFT, res);
}

/* mode */
static int isl29023_get_mode(struct i2c_client *client)
{
	return __isl29023_read_reg(client, ISL29023_COMMAND1,
		ISL29023_MODE_MASK, ISL29023_MODE_SHIFT);
}

static int isl29023_set_mode(struct i2c_client *client, int mode)
{
	return __isl29023_write_reg(client, ISL29023_COMMAND1,
		ISL29023_MODE_MASK, ISL29023_MODE_SHIFT, mode);
}

/* power_state */
static int isl29023_set_power_state(struct i2c_client *client, int state)
{
	struct isl29023_data *data = i2c_get_clientdata(client);

	int rc;
	
	// Save mode if we go into power down mode and return to it 
	if( !state ) 
	{
		// Power down and save current mode
		data->mode_before_powerdown = isl29023_get_mode( client );
		rc = isl29023_set_mode( client, ISL29023_PD_MODE );
	}
	else if( isl29023_get_mode( client ) == ISL29023_PD_MODE )
	{
			// Power up, restore previous mode if available
			if( data->mode_before_powerdown != ISL29023_PD_MODE )
				rc = isl29023_set_mode( client, data->mode_before_powerdown );
			else
				rc = isl29023_set_mode( client, ISL29023_DEFAULT_MODE );
	}

	if (data->polled) {
		if (state)
			schedule_delayed_work(&data->polled_work,
				msecs_to_jiffies(data->poll_interval));
		else
			cancel_delayed_work_sync(&data->polled_work);
		}
	return rc;
}

static int isl29023_get_power_state(struct i2c_client *client)
{
	struct isl29023_data *data = i2c_get_clientdata(client);
	u8 cmdreg = data->reg_cache[ISL29023_COMMAND1];

	if (cmdreg & ISL29023_MODE_MASK)
		return 1;
	else
		return 0;
}

//-----------------------------------------------------------------------------

static int isl29023_convert_lux( struct i2c_client *client, int adcval )
{
	int bitdepth, range, luxval;
	struct isl29023_data *data = i2c_get_clientdata(client);
	
	if( adcval < 0 ) return adcval;
	
	// Bit resolution
	bitdepth = (4 - isl29023_get_resolution(client)) * 4;
	
	// Range
	range    = isl29023_get_range(client);
	
	// Convert to lux unit
	// 499 is the nominal external resistor value in kOhms (see datasheet)	
	luxval = ( adcval * ( (gain_range[range] * 499)/(data->rext) ) ) >> bitdepth;
	
	return luxval;
}

//-----------------------------------------------------------------------------

static int isl29023_get_adc_value(struct i2c_client *client)
{
	struct isl29023_data *data = i2c_get_clientdata(client);
	int lsb, msb, adcval, mode;

	mutex_lock(&data->lock);
	
	lsb = i2c_smbus_read_byte_data(client, ISL29023_REG_LSB_SENSOR);

	if (lsb < 0) {
		mutex_unlock(&data->lock);
		return lsb;
	}

	msb = i2c_smbus_read_byte_data(client, ISL29023_REG_MSB_SENSOR);
	mutex_unlock(&data->lock);

	if (msb < 0)
		return msb;
	
	adcval = ((msb << 8 ) | lsb);
		
	return adcval;
}

//-----------------------------------------------------------------------------

static int isl29023_get_lux_value(struct i2c_client *client)
{
	int adcval, luxval;
	int mode;
	
	// Check if we are in the correct mode
	mode = isl29023_get_mode( client );
	if( mode != ISL29023_ALS_CONT_MODE )
	{
		if( isl29023_set_mode( client, ISL29023_ALS_ONCE_MODE ) < 0 )
			return -EBUSY;
			
		// Wait for measurement
		msleep(100);
	}
	
	// Read ADC
	adcval = isl29023_get_adc_value( client );
	
	// Convert to lux unit
	luxval = isl29023_convert_lux( client, adcval );
	
	// Return to previous mode
	if( mode == ISL29023_IR_CONT_MODE || mode == ISL29023_PD_MODE )
		isl29023_set_mode( client, mode );
	
	return luxval;
}

//-----------------------------------------------------------------------------

static int isl29023_get_ir_value(struct i2c_client *client)
{
	int adcval, luxval;
	int mode;
	
	// Check if we are in the correct mode
	mode = isl29023_get_mode( client );
	if( mode != ISL29023_IR_CONT_MODE )
	{
		if( isl29023_set_mode( client, ISL29023_IR_ONCE_MODE ) < 0 )
			return -EBUSY;
			
		// Wait for measurement
		msleep(100);
	}
	
	// Read ADC
	adcval = isl29023_get_adc_value( client );
	
	// Convert to lux unit
	luxval = isl29023_convert_lux( client, adcval );
	
	// Return to previous mode
	if( mode == ISL29023_ALS_CONT_MODE || mode == ISL29023_PD_MODE )
		isl29023_set_mode( client, mode );
	
	return luxval;
}

//-----------------------------------------------------------------------------

static int isl29023_get_int_lt_value(struct i2c_client *client)
{
	struct isl29023_data *data = i2c_get_clientdata(client);
	int lsb, msb, adcval;

	mutex_lock(&data->lock);
	lsb = i2c_smbus_read_byte_data(client, ISL29023_REG_IRQ_TH_LO_LSB);

	if (lsb < 0) {
		mutex_unlock(&data->lock);
		return lsb;
	}

	msb = i2c_smbus_read_byte_data(client, ISL29023_REG_IRQ_TH_LO_MSB);
	mutex_unlock(&data->lock);

	if (msb < 0)
		return msb;
		
	adcval = ((msb << 8 ) | lsb);
		
	return isl29023_convert_lux( client, adcval );
}

static int isl29023_get_int_ht_value(struct i2c_client *client)
{
	struct isl29023_data *data = i2c_get_clientdata(client);
	int lsb, msb, adcval;

	mutex_lock(&data->lock);
	lsb = i2c_smbus_read_byte_data(client, ISL29023_REG_IRQ_TH_HI_LSB);

	if (lsb < 0) {
		mutex_unlock(&data->lock);
		return lsb;
	}

	msb = i2c_smbus_read_byte_data(client, ISL29023_REG_IRQ_TH_HI_MSB);
	mutex_unlock(&data->lock);

	if (msb < 0)
		return msb;

  adcval = ((msb << 8 ) | lsb);

	return isl29023_convert_lux( client, adcval );
}

/*
 * sysfs layer
 */

/* interrupt persists */
static ssize_t isl29023_show_int_persists(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	return sprintf(buf, "%i\n", isl29023_get_int_persists(client));
}

static ssize_t isl29023_store_int_persists(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val;
	int ret;

	if ((strict_strtoul(buf, 10, &val) < 0) ||
	    (val > ISL29023_INT_PERSISTS_16))
		return -EINVAL;

	ret = isl29023_set_int_persists(client, val);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(int_persists, S_IWUSR | S_IRUGO,
		   isl29023_show_int_persists, isl29023_store_int_persists);

/* interrupt flag */
static ssize_t isl29023_show_int_flag(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	return sprintf(buf, "%i\n", isl29023_get_int_flag(client));
}

static ssize_t isl29023_store_int_flag(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val;
	int ret;

	if ((strict_strtoul(buf, 10, &val) < 0) || (val > 1))
		return -EINVAL;

	ret = isl29023_set_int_flag(client, val);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(int_flag, S_IWUSR | S_IRUGO,
		   isl29023_show_int_flag, isl29023_store_int_flag);

/* interrupt lt */
static ssize_t isl29023_show_int_lt(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	return sprintf(buf, "%i\n", isl29023_get_int_lt(client));
}

static ssize_t isl29023_store_int_lt(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val;
	int ret;

	if ((strict_strtoul(buf, 16, &val) < 0) || (val > 0xffff))
		return -EINVAL;

	ret = isl29023_set_int_lt(client, val);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(int_lt, S_IWUSR | S_IRUGO,
		   isl29023_show_int_lt, isl29023_store_int_lt);

/* interrupt ht */
static ssize_t isl29023_show_int_ht(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	return sprintf(buf, "%i\n", isl29023_get_int_ht(client));
}

static ssize_t isl29023_store_int_ht(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val;
	int ret;

	if ((strict_strtoul(buf, 16, &val) < 0) || (val > 0xffff))
		return -EINVAL;

	ret = isl29023_set_int_ht(client, val);
	if (ret < 0)
		return ret;

	return count;
}

static DEVICE_ATTR(int_ht, S_IWUSR | S_IRUGO,
		   isl29023_show_int_ht, isl29023_store_int_ht);

/* range */
static ssize_t isl29023_show_range(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int k_gain = isl29023_get_range(client);
	
	if( k_gain < 0 || k_gain > ARRAY_SIZE(gain_range)-1 ) return -EAGAIN;
	
	return sprintf(buf, "%d\n", gain_range[k_gain]);
}

static ssize_t isl29023_store_range(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val;
	int ret, i, k = -1;

	if( strict_strtoul(buf, 10, &val) < 0 )
		return -EINVAL;
		
	// Check if given value is allowed
	for( i = 0; i < ARRAY_SIZE( gain_range ); i++ ) {
		if( gain_range[i] == val ) {
				k = i;	
				break;
		}
	}
	
	if( k < 0 )
	{
		dev_err( dev, "The range is not supported.\n" );
		return -EINVAL;
	}

	// Apply new range
	ret = isl29023_set_range(client, k);
	if (ret < 0)
	{
		dev_err( dev, "Error in setting range.\n" );
		return ret;
	}

	return count;
}

//-----------------------------------------------------------------------------

static ssize_t isl29023_show_range_available(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return sprintf(buf, "1000 4000 16000 64000\n");
}

//-----------------------------------------------------------------------------

static DEVICE_ATTR(range, S_IWUSR | S_IRUGO,
		   isl29023_show_range, isl29023_store_range);
static DEVICE_ATTR(range_available, S_IRUGO,
       isl29023_show_range_available, NULL);


/* resolution */
static ssize_t isl29023_show_resolution(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	int k_res = isl29023_get_resolution(client);
	
	if( k_res < 0 || k_res > ARRAY_SIZE(adc_resolution)-1 ) return -EAGAIN;
	
	return sprintf(buf, "%d\n", adc_resolution[k_res]);
}

static ssize_t isl29023_store_resolution(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val;
	int ret;
	
	int i, k = -1;

	if( strict_strtoul(buf, 10, &val) < 0 )
		return -EINVAL;
		
	// Check if given value is allowed
	for( i = 0; i < ARRAY_SIZE( adc_resolution ) - 1; i++ ) {
		if( adc_resolution[i] == val ) {
				k = i;	
				break;
		}
	}
	
	if( k < 0 )
	{
		dev_err( dev, "The ADC resolution is not supported.\n" );
		return -EINVAL;
	}

	// Apply new ADC resolution
	ret = isl29023_set_resolution(client, k);
	if (ret < 0)
	{
		dev_err( dev, "Error in setting ADC resolution.\n" );
		return ret;
	}

	return count;
}

//-----------------------------------------------------------------------------

static ssize_t isl29023_show_resolution_available(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return sprintf(buf, "4 8 12 16\n");
}

//-----------------------------------------------------------------------------

static DEVICE_ATTR(resolution, S_IWUSR | S_IRUGO,
		   isl29023_show_resolution, isl29023_store_resolution);
static DEVICE_ATTR(resolution_available, S_IRUGO,
       isl29023_show_resolution_available, NULL);

/* mode */
static ssize_t isl29023_show_mode(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	return sprintf(buf, "%d\n", isl29023_get_mode(client));
}

static ssize_t isl29023_store_mode(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val;
	int ret;

	if ((strict_strtoul(buf, 10, &val) < 0) ||
	    (val > ISL29023_IR_CONT_MODE))
		return -EINVAL;

	ret = isl29023_set_mode(client, val);
	if (ret < 0)
		return ret;

	return count;
}

//-----------------------------------------------------------------------------

static ssize_t isl29023_show_mode_available(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return sprintf(buf, __stringify(ISL29023_PD_MODE)       " - power down\n"
	                    __stringify(ISL29023_ALS_ONCE_MODE) " - measure ambient light once (human spectral response)\n"
	                    __stringify(ISL29023_IR_ONCE_MODE)  " - measure light once (IR-heavy response)\n"
	                    __stringify(ISL29023_ALS_CONT_MODE) " - measure ambient light continuously (human spectral response)\n"
	                    __stringify(ISL29023_IR_CONT_MODE)  " - measure light continuously (IR-heavy response)\n");
}

//-----------------------------------------------------------------------------

static DEVICE_ATTR(mode, 0666,
		   isl29023_show_mode, isl29023_store_mode);
static DEVICE_ATTR(mode_available, S_IRUGO,
		   isl29023_show_mode_available, NULL);

/* power state */
static ssize_t isl29023_show_power_state(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	return sprintf(buf, "%d\n", isl29023_get_power_state(client));
}

static ssize_t isl29023_store_power_state(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	unsigned long val;
	int ret;

	if ((strict_strtoul(buf, 10, &val) < 0) || (val > 1))
		return -EINVAL;

	ret = isl29023_set_power_state(client, val);

	return ret ? ret : count;
}

static DEVICE_ATTR(power_state, 0666,
		   isl29023_show_power_state, isl29023_store_power_state);

/* lux */
static ssize_t isl29023_show_lux(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	/* No LUX data if not operational */
	if (!isl29023_get_power_state(client))
	{
		dev_err( dev, "Device is powered down.\n" );
		return -EBUSY;
	}

	return sprintf(buf, "%d\n", isl29023_get_lux_value(client));
}

static DEVICE_ATTR(lux, S_IRUGO, isl29023_show_lux, NULL);

/* ir */
static ssize_t isl29023_show_ir(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	/* No IR data if not operational */
	if (!isl29023_get_power_state(client))
	{
		dev_err( dev, "Device is powered down.\n" );
		return -EBUSY;
	}

	return sprintf(buf, "%d\n", isl29023_get_ir_value(client));
}

static DEVICE_ATTR(ir, S_IRUGO, isl29023_show_ir, NULL);

/* lux interrupt low threshold */
static ssize_t isl29023_show_int_lt_lux(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	/* No LUX data if not operational */
	if (isl29023_get_mode(client) != ISL29023_ALS_ONCE_MODE &&
	    isl29023_get_mode(client) != ISL29023_ALS_CONT_MODE)
		return -EIO;

	return sprintf(buf, "%d\n", isl29023_get_int_lt_value(client));
}

static ssize_t isl29023_store_int_lt_lux(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct isl29023_data *data = i2c_get_clientdata(client);
	unsigned long val, lux_data;
	int range, bitdepth, ret;
	u8 lsb, msb;

	if ((strict_strtoul(buf, 10, &val) < 0))
		return -EINVAL;

	/* No LUX data if not operational */
	if (isl29023_get_mode(client) != ISL29023_ALS_ONCE_MODE &&
	    isl29023_get_mode(client) != ISL29023_ALS_CONT_MODE)
		return -EIO;

	if (val > (gain_range[isl29023_get_range(client)]*499/data->rext))
		return -EINVAL;

	range = isl29023_get_range(client);
	bitdepth = (4 - isl29023_get_resolution(client)) * 4;
	lux_data = ((unsigned long)(val << bitdepth)) /
		((gain_range[range] * 499) / data->rext);
	lux_data &= 0xffff;

	msb = lux_data >> 8;
	lsb = lux_data & 0xff;

	mutex_lock(&data->lock);
	ret = i2c_smbus_write_byte_data(client, ISL29023_REG_IRQ_TH_LO_LSB,
					lsb);
	if (ret < 0) {
		mutex_unlock(&data->lock);
		return ret;
	}

	ret = i2c_smbus_write_byte_data(client, ISL29023_REG_IRQ_TH_LO_MSB,
					msb);
	if (ret < 0) {
		mutex_unlock(&data->lock);
		return ret;
	}

	data->reg_cache[ISL29023_REG_IRQ_TH_LO_MSB] = msb;
	data->reg_cache[ISL29023_REG_IRQ_TH_LO_LSB] = lsb;
	mutex_unlock(&data->lock);

	return count;
}

static DEVICE_ATTR(int_lt_lux, S_IWUSR | S_IRUGO,
		isl29023_show_int_lt_lux, isl29023_store_int_lt_lux);

/* lux interrupt high threshold */
static ssize_t isl29023_show_int_ht_lux(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	/* No LUX data if not operational */
	if (isl29023_get_mode(client) != ISL29023_ALS_ONCE_MODE &&
	    isl29023_get_mode(client) != ISL29023_ALS_CONT_MODE)
		return -EIO;

	return sprintf(buf, "%d\n", isl29023_get_int_ht_value(client));
}

static ssize_t isl29023_store_int_ht_lux(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct isl29023_data *data = i2c_get_clientdata(client);
	unsigned long val, lux_data;
	int range, bitdepth, ret;
	u8 lsb, msb;

	if ((strict_strtoul(buf, 10, &val) < 0))
		return -EINVAL;

	/* No LUX data if not operational */
	if (isl29023_get_mode(client) != ISL29023_ALS_ONCE_MODE &&
	    isl29023_get_mode(client) != ISL29023_ALS_CONT_MODE)
		return -EIO;

	if (val > (gain_range[isl29023_get_range(client)]*499/data->rext))
		return -EINVAL;

	range = isl29023_get_range(client);
	bitdepth = (4 - isl29023_get_resolution(client)) * 4;
	lux_data = ((unsigned long)(val << bitdepth)) /
		((gain_range[range] * 499) / data->rext);
	lux_data &= 0xffff;

	msb = lux_data >> 8;
	lsb = lux_data & 0xff;

	mutex_lock(&data->lock);
	ret = i2c_smbus_write_byte_data(client, ISL29023_REG_IRQ_TH_HI_LSB,
					lsb);
	if (ret < 0) {
		mutex_unlock(&data->lock);
		return ret;
	}

	ret = i2c_smbus_write_byte_data(client, ISL29023_REG_IRQ_TH_HI_MSB,
					msb);
	if (ret < 0) {
		mutex_unlock(&data->lock);
		return ret;
	}

	data->reg_cache[ISL29023_REG_IRQ_TH_HI_MSB] = msb;
	data->reg_cache[ISL29023_REG_IRQ_TH_HI_LSB] = lsb;
	mutex_unlock(&data->lock);

	return count;
}

static DEVICE_ATTR(int_ht_lux, S_IWUSR | S_IRUGO,
		isl29023_show_int_ht_lux, isl29023_store_int_ht_lux);

static struct attribute *isl29023_attributes[] = {
	&dev_attr_int_persists.attr,
	&dev_attr_range.attr,
	&dev_attr_range_available.attr,
	&dev_attr_resolution.attr,
	&dev_attr_resolution_available.attr,
	&dev_attr_mode.attr,
	&dev_attr_mode_available.attr,
	&dev_attr_power_state.attr,
	&dev_attr_ir.attr,
	&dev_attr_lux.attr,
	&dev_attr_int_lt_lux.attr,
	&dev_attr_int_ht_lux.attr,
	&dev_attr_int_lt.attr,
	&dev_attr_int_ht.attr,
	&dev_attr_int_flag.attr,
	NULL
};

static const struct attribute_group isl29023_attr_group = {
	.attrs = isl29023_attributes,
};

static int isl29023_init_client(struct i2c_client *client)
{
	struct isl29023_data *data = i2c_get_clientdata(client);
	int i;

	/* read all the registers once to fill the cache.
	 * if one of the reads fails, we consider the init failed */
	for (i = 0; i < ARRAY_SIZE(data->reg_cache); i++) {
		int v = i2c_smbus_read_byte_data(client, i);
		if (v < 0)
			return -ENODEV;

		data->reg_cache[i] = v;
	}

	/* set defaults */
	isl29023_set_int_persists(client, ISL29023_INT_PERSISTS_8);
	isl29023_set_int_ht(client, 0xffff);
	isl29023_set_int_lt(client, 0x0);
	isl29023_set_range(client, ISL29023_RANGE_4K);
	isl29023_set_resolution(client, ISL29023_RES_16);
	isl29023_set_mode(client, ISL29023_DEFAULT_MODE);
	isl29023_set_int_flag(client, 0);
	//isl29023_set_power_state(client, 1); // Not needed as power state equals a mode setting

	return 0;
}

static void isl29023_work(struct work_struct *work)
{
	struct i2c_client *client;
	int lux;
	struct isl29023_data *data =
		container_of(work, struct isl29023_data, work);

	struct isl29023_data *data_polled =
		container_of((struct delayed_work *)work, struct isl29023_data,
				polled_work);

	if (data_polled->polled)
		data = data_polled;

	client = data->client;
	
	if (!data->polled) {
		/* Clear interrupt flag */
		isl29023_set_int_flag(client, 0);
		data->mode_before_interrupt = isl29023_get_mode(client);
	}

	lux = isl29023_get_lux_value(client);

	if (!data->polled) {
	/* To clear the interrpt status */
		isl29023_set_power_state(client, ISL29023_PD_MODE);
		isl29023_set_mode(client, data->mode_before_interrupt);
	}

	msleep(100);

	input_report_abs(data->input, ABS_MISC, lux);
	input_sync(data->input);

	if(data->polled) {
		schedule_delayed_work(&data->polled_work,
				msecs_to_jiffies(data->poll_interval));
	}
}

static irqreturn_t isl29023_irq_handler(int irq, void *handle)
{
	struct isl29023_data *data = handle;
	queue_work(data->workqueue, &data->work);
	return IRQ_HANDLED;
}

/*
 * I2C layer
 */

static int __devinit isl29023_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct isl29023_data *data;
	struct isl29023_platform_data *ls_data;
	struct input_dev *input_dev;
	int err = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	data = kzalloc(sizeof(struct isl29023_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	ls_data = (struct isl29023_platform_data *)
	    (client->dev).platform_data;

	data->client = client;
	data->rext = ls_data->rext;
	data->polled = ls_data->polled;
	data->poll_interval = ls_data->poll_interval;
	data->mode_before_powerdown = ISL29023_PD_MODE;
	snprintf(data->phys, sizeof(data->phys),
		 "%s", dev_name(&client->dev));
	i2c_set_clientdata(client, data);
	mutex_init(&data->lock);

	/* initialize the ISL29023 chip */
	err = isl29023_init_client(client);
	if (err)
		goto exit_kfree;

	/* register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &isl29023_attr_group);
	if (err)
		goto exit_kfree;

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		goto exit_kfree;
	}

	data->input = input_dev;
	input_dev->name = "isl29023 light sensor";
	input_dev->id.bustype = BUS_I2C;
	input_dev->phys = data->phys;

	__set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_MISC, 0,
			gain_range[ARRAY_SIZE(gain_range)-1]*499/data->rext, 0, 0);

	err = input_register_device(input_dev);
	if (err)
		goto exit_free_input;

	if (!data->polled) {
		/* set irq type to edge falling */
		irq_set_irq_type(client->irq, IRQF_TRIGGER_FALLING);
		err = request_irq(client->irq, isl29023_irq_handler, 0,
				client->dev.driver->name, data);
		if (err < 0) {
			dev_err(&client->dev, "failed to register irq %d!\n",
				client->irq);
			goto exit_free_input;
		}
	} else {
		INIT_DELAYED_WORK(&data->polled_work, isl29023_work);
	}

	data->workqueue = create_singlethread_workqueue("isl29023");
	INIT_WORK(&data->work, isl29023_work);
	if (data->workqueue == NULL) {
		dev_err(&client->dev, "couldn't create workqueue\n");
		err = -ENOMEM;
		goto exit_free_interrupt;
	}

	dev_info(&client->dev, "driver version %s enabled\n", DRIVER_VERSION);
	return 0;

exit_free_interrupt:
	free_irq(client->irq, data);
exit_free_input:
	input_free_device(input_dev);
exit_kfree:
	kfree(data);
	return err;
}

static int __devexit isl29023_remove(struct i2c_client *client)
{
	struct isl29023_data *data = i2c_get_clientdata(client);

	if (data->polled)
		cancel_delayed_work_sync(&data->polled_work);
	cancel_work_sync(&data->work);
	destroy_workqueue(data->workqueue);
	free_irq(client->irq, data);
	input_unregister_device(data->input);
	input_free_device(data->input);
	sysfs_remove_group(&client->dev.kobj, &isl29023_attr_group);
	isl29023_set_power_state(client, 0);
	kfree(i2c_get_clientdata(client));

	return 0;
}

#ifdef CONFIG_PM
static int isl29023_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct isl29023_data *data = i2c_get_clientdata(client);

	data->mode_before_suspend = isl29023_get_mode(client);
	return isl29023_set_power_state(client, ISL29023_PD_MODE);
}

static int isl29023_resume(struct i2c_client *client)
{
	int i;
	struct isl29023_data *data = i2c_get_clientdata(client);

	/* restore registers from cache */
	for (i = 0; i < ARRAY_SIZE(data->reg_cache); i++)
		if (i2c_smbus_write_byte_data(client, i, data->reg_cache[i]))
			return -EIO;

	return isl29023_set_mode(client, data->mode_before_suspend);
}

#else
#define isl29023_suspend	NULL
#define isl29023_resume		NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id isl29023_id[] = {
	{ ISL29023_DRV_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, isl29023_id);

static struct i2c_driver isl29023_driver = {
	.driver = {
		.name	= ISL29023_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend = isl29023_suspend,
	.resume	= isl29023_resume,
	.probe	= isl29023_probe,
	.remove	= __devexit_p(isl29023_remove),
	.id_table = isl29023_id,
};

static int __init isl29023_init(void)
{
	return i2c_add_driver(&isl29023_driver);
}

static void __exit isl29023_exit(void)
{
	i2c_del_driver(&isl29023_driver);
}

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("ISL29023 ambient light sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(isl29023_init);
module_exit(isl29023_exit);