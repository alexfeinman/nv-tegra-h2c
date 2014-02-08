/*
 * OmniVision TC358743 sensor driver
 *
 * Copyright (c) 2013, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/videodev2.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/log2.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/mutex.h>

#include <linux/platform_device.h>
#include <linux/miscdevice.h>

#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-chip-ident.h>
#include <media/soc_camera.h>

#include <media/tc358743.h>
#include <asm/uaccess.h>
#include "tc358743-regs.h"

#define SIZEOF_I2C_TRANSBUF 32
#define DRIVER_NAME "tc358743"

struct tc358743_priv {
	struct v4l2_subdev				subdev;
	struct v4l2_mbus_framefmt		mf;
	struct miscdevice				misc_dev;
	struct work_struct				work;
	struct mutex					mutex;
	int								irq;
	struct mutex					tx_mutex;
	wait_queue_head_t				rx_waitq;
	wait_queue_head_t				tx_waitq;
	int								rx_wake;
	int								tx_isr;

	int 							ident;
	u16 							chip_id;
	u8 								revision;
	int								lanes;

	int 							last_requested_mode;
	int 							last_set_mode;
	u32								last_frame_status;

	struct i2c_client* 				client;
	u8 								i2c_trans_buf[SIZEOF_I2C_TRANSBUF];
	unsigned int 					i2c_trans_buf_filled;
	int								streaming;
	u8								video_res;
    bool							video_is_present;
	bool							video_update_flag;
};

static void tc358743_enable_hpd_interrupt(struct i2c_client* client);
static void tc358743_setup_interrupts(struct i2c_client *client);
static long tc358743_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg);

static struct tc358743_priv *to_tc358743(const struct v4l2_subdev *sd)
{
	return container_of(sd, struct tc358743_priv, subdev);
}

__used static struct h2c_32_reg stop_streaming[] = {
};


static int h2c_read_reg(struct i2c_client *client, u16 addr, u8 *val, u8 len)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[6];

	if (!client->adapter)
		return -ENODEV;

	if ( len > 4 )
		return EINVAL;

	memset(val, 0, len);

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 3;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

	if ( len > 2 ) {
		val[2] = data[4];
		val[3] = data[5];
	}

	if ( len > 1 )
		val[1] = data[3];

	val[0] = data[2];

	return 0;
}

static int h2c_write_reg(struct i2c_client *client, u32 addr, u32 val, u32 len)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[6];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	if ( len == 0 )
		len = 1;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
	switch(len) {
		case 4:
			data[4] = (u8) ((val >> 16) & 0xff);
			data[5] = (u8) ((val >> 24) & 0xff);
		case 2:
			data[3] = (u8) ((val >> 8) & 0xff);
		case 1:
			data[2] = (u8) (val & 0xff);
			break;
	}

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 2 + len;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		dev_err(&client->dev, "i2c transfer failed, retrying %x %x %d\n",
		       addr, val, err);

		msleep(3);
	} while (retry <= H2C_MAX_RETRIES);

	return err;
}

static int h2c_write_reg_multiple(struct i2c_client *client, u32 addr, u8 *val, u8 len)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[6];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	if ( len > 4 )
		return EINVAL;

	if ( len == 0 )
		len = 1;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);

	memcpy(&data[2], val, len);
	/*
	switch(len) {
	case 4:
		data[4] = (u8) ((val >> 16) & 0xff);
		data[5] = (u8) ((val >> 24) & 0xff);
	case 2:
		data[3] = (u8) ((val >> 8) & 0xff);
	case 1:
		data[2] = (u8) (val & 0xff);
		break;
	}*/

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 2 + len;
	msg.buf = data;
 
	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		dev_err(&client->dev, "i2c transfer failed, retrying %x %x %d\n",
			addr, val[0], err);

		msleep(3);
	} while (retry <= H2C_MAX_RETRIES);

	return err;
}



static int h2c_read_reg_multiple(struct i2c_client *client, u16 addr, u8 *val, u8 len)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[6];

	if (!client->adapter)
		return -ENODEV;
 
	if ( len > 4 )
		return EINVAL;
 
	memset(val, 0, len);

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

	if ( len > 2 ) {
		val[2] = data[4];
		val[3] = data[5];
	}
 
	if ( len > 1 )
		val[1] = data[3];
 
	val[0] = data[2];

	return 0;
}

static int h2c_write_table(struct i2c_client *client,
			         const struct h2c_32_reg table[])
{
	int err;
	const struct h2c_32_reg *next;
	u32 val;

	for (next = table; next->addr != H2C_TABLE_END; next++) {
		if (next->addr == H2C_TABLE_WAIT_MS) {
			msleep(next->value);
			continue;
		}

		if ( next->addr == H2C_TABLE_REPEAT ) {
			int regcnt =  next->value >> 12; // how many registers to write repeatedly
			int rptcnt =  next->value & 0xfff; // how many times to repeat

			int c,n;
			const struct h2c_32_reg * rpt;
			for (c = 0; c < rptcnt; c++) {
  				for ( n = 0, rpt = next + 1; n < regcnt; n++, rpt++ ) {
					err = h2c_write_reg(client, rpt->addr, rpt->value, rpt->len);
					if (err) {
						dev_err(&client->dev, "%s: write error @%d\n", __func__,
							rpt - table);
						return err;
					}
				}
			}
			next += regcnt;
			continue;
		}

		val = next->value;

		err = h2c_write_reg(client, next->addr, val, next->len);
		if (err)
			return err;
	}
	return 0;
}



enum {
	TC358743_MODE_1920x1080,
	TC358743_MODE_1280x720,
	TC358743_MODE_640x480,
	TC358743_MODE_320x240,
	TC358743_MODE_INVALID
};



#if 0
static struct h2c_mode mode_table[] = {
	[TC358743_MODE_1920x1080] = { 1920, 1072 },
	[TC358743_MODE_1280x720]  = { 1280,  720 },
	[TC358743_MODE_640x480]   = {  640,  480 },
};
#endif

static const struct v4l2_frmsize_discrete tc358743_frmsizes[] = {
	{1920, 1072},
	{1280, 720},
	{640, 480},
};

#if 0
static int tc358743_write_bulk_reg(struct i2c_client *client, u8 *data, int len)
{
	int err;
	struct i2c_msg msg;

	if (!client->adapter)
		return -ENODEV;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err == 1)
		return 0;

	dev_err(&client->dev, "tc358743: i2c bulk transfer failed at %x\n",
		(int)data[0] << 8 | data[1]);

	return err;
}


static int tc358743_write_val(struct tc358743_priv *priv, const struct tc358743_reg *elem)
{
	const struct tc358743_reg *n_next;
	int err;
	u32 val = elem->val;
	u16 size = elem->size;
	/* When an override list is passed in, replace the reg */
	/* value to write if the reg is in the list            */
	u8 *b_ptr = priv->i2c_trans_buf + priv->i2c_trans_buf_filled;
	if (!priv->i2c_trans_buf_filled) {
		b_ptr = priv->i2c_trans_buf;
		*b_ptr++ = elem->addr >> 8;
		*b_ptr++ = elem->addr & 0xff;
		priv->i2c_trans_buf_filled = 2;
	}
	switch (size) {
	case 1: *b_ptr++ = val;
		break;
	case 2: *b_ptr++ = (u8)(val >> 8);
		*b_ptr++ = (u8)(val & 0xff);
		break;
	case 4: *b_ptr++ = (u8)(val >> 8);
		*b_ptr++ = (u8)(val & 0xff);
		*b_ptr++ = (u8)(val >> 24);
		*b_ptr++ = (u8)(val >> 16);
		break;
	default:
		dev_err(&client->dev, "%s: wrong value size %d\n", __func__, size);
		return -1;
	}
	priv->i2c_trans_buf_filled += size;

	n_next = elem + 1;
	if (n_next->addr != TC358743_TABLE_END &&
	    n_next->addr != TC358743_TABLE_WAIT_MS &&
	    priv->i2c_trans_buf_filled < SIZEOF_I2C_TRANSBUF &&
	    n_next->addr == elem->addr + size) {
		return 0;
	}

	err = tc358743_write_bulk_reg(priv->client,
		priv->i2c_trans_buf, priv->i2c_trans_buf_filled);
	if (err)
		return err;
	priv->i2c_trans_buf_filled = 0;
	return 0;

}

static int tc358743_write_table(struct tc358743_priv *info,
			      const struct tc358743_reg table[])
{
	int err;
	const struct tc358743_reg *next;
	info->i2c_trans_buf_filled = 0;

	for (next = table; next->addr != TC358743_TABLE_END; next++) {
		if (next->addr == TC358743_TABLE_WAIT_MS) {
			msleep(next->val);
			continue;
		}
		if (next->addr == TC358743_TABLE_REPEAT) {
			int num = next->size;
			int count = next->val;
			int c,n;
			for (c=0;c<count;c++) {
  				for (n=0;n<num;n++) {
					err = tc358743_write_val(info, next+1+n);
					if (err) {
						dev_err(&client->dev, "%s: write error @%d\n", __func__,
							next+1+n-table);
						return err;
					}
				}
			}
			next += num;
			continue;
		}
		err = tc358743_write_val(info, next);
		if (err) {
			dev_err(&client->dev, "%s: write error @%d\n", __func__,
				next-table);
			return err;
		}
	}
	return 0;
}

#endif

static int tc358743_find_mode(struct tc358743_priv *priv, u32 width, u32 height)
{
	if (width == 1920 && height == 1072)
		return TC358743_MODE_1920x1080;
	else if (width == 1280 && height == 720)
		return TC358743_MODE_1280x720;
	else if (width == 640 && height == 480)
		return TC358743_MODE_640x480;
	else {
		dev_warn(&priv->client->dev, "tc358743: %dx%d is not supported\n", width, height);
		return TC358743_MODE_1280x720;
	}
}

#if 0
/**
 * tc358743_reg_read - Read a value from a register in an tc358743 sensor device
 * @client: i2c driver client structure
 * @reg: register address / offset
 * @val: stores the value that gets read
 *
 * Read a value from a register in an tc358743 sensor device.
 * The value is returned in 'val'.
 * Returns zero if successful, or non-zero otherwise.
 */
static int tc358743_reg_read(struct i2c_client *client, u16 reg, u8 *val, u8 size)
{
	int ret;
	u8 data[2] = {0};
	struct i2c_msg msg[] ={
		 {
			.addr	= client->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= data,
		},
		 {
			.addr	= client->addr,
			.flags	= I2C_M_RD/*|I2C_M_NOSTART*/,
			.len	= size,
			.buf	= val,
		},
	};

	data[0] = (u8)(reg >> 8);
	data[1] = (u8)(reg & 0xff);

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0)
		goto err;
#if 0
	msg.flags = I2C_M_RD;
	msg.len = size;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0)
		goto err;
# endif
	*val = data[0];
	return 0;

err:
	dev_err(&client->dev, "Failed reading register 0x%02x!\n", reg);
	return ret;
}

/**
 * Write a value to a register in tc358743 sensor device.
 * @client: i2c driver client structure.
 * @reg: Address of the register to read value from.
 * @val: Value to be written to a specific register.
 * Returns zero if successful, or non-zero otherwise.
 */
static int tc358743_reg_write(struct i2c_client *client, u16 reg, u8* val, u8 size)
{
	int ret;
	//unsigned char data[3] = { (u8)(reg >> 8), (u8)(reg & 0xff), val };
	u8* data = kzalloc(size + 2, GFP_KERNEL);
	struct i2c_msg msg = {
		.addr	= client->addr,
		.flags	= 0,
		.len	= size + 2,
		.buf	= data,
	};

	data[0] = reg >> 8;
	data[1] = reg & 0xff;
	memcpy(data + 2, val, size);

	ret = i2c_transfer(client->adapter, &msg, 1);
	kfree(data);
	if (ret < 0) {
		dev_err(&client->dev, "Failed writing register 0x%02x!\n", reg);
		return ret;
	}

	return 0;
}

static int tc358743_write_bulk_reg(struct tc358743_priv *priv, int len)
{
	struct i2c_client *client = priv->client;
	u8 *data = priv->i2c_trans_buf;
	int err;
	struct i2c_msg msg;

	if (!client->adapter)
		return -ENODEV;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;

	err = i2c_transfer(client->adapter, &msg, 1);
	if (err != 1) {
		dev_err(&client->dev, "I2C bulk transfer failed at %x\n",
			(int)data[0] << 8 | data[1]);
		return err;
	}

	return 0;
}
static int tc358743_write_table(struct tc358743_priv *priv,
				const struct tc358743_reg table[],
				const struct tc358743_reg override_list[],
				int num_override_regs)
{
	int err;
	const struct tc358743_reg *next, *n_next;
	u8 *b_ptr = priv->i2c_trans_buf;
	unsigned int buf_filled = 0;
	unsigned int i;
	u16 val;

	for (next = table; next->addr != TC358743_TABLE_END; next++) {
		if (next->addr == TC358743_TABLE_WAIT_MS) {
			msleep(next->val);
			continue;
		}

		val = next->val;
		/* When an override list is passed in, replace the reg */
		/* value to write if the reg is in the list            */
		if (override_list) {
			for (i = 0; i < num_override_regs; i++) {
				if (next->addr == override_list[i].addr) {
					val = override_list[i].val;
					break;
				}
			}
		}

		if (!buf_filled) {
			b_ptr = priv->i2c_trans_buf;
			*b_ptr++ = next->addr >> 8;
			*b_ptr++ = next->addr & 0xff;
			buf_filled = 2;
		}
		*b_ptr++ = val;
		buf_filled++;

		n_next = next + 1;
		if (n_next->addr != TC358743_TABLE_END &&
			n_next->addr != TC358743_TABLE_WAIT_MS &&
			buf_filled < SIZEOF_I2C_TRANSBUF &&
			n_next->addr == next->addr + 1) {
			continue;
		}

		err = tc358743_write_bulk_reg(priv, buf_filled);
		if (err)
			return err;

		buf_filled = 0;
	}
	return 0;
}
#endif
/* -----------------------------------------------------------------------------
 * V4L2 subdev internal operations
 */

static void tc358743_set_default_fmt(struct tc358743_priv *priv)
{
	struct v4l2_mbus_framefmt *mf = &priv->mf;

	mf->width = tc358743_frmsizes[TC358743_MODE_1920x1080].width;
	mf->height = tc358743_frmsizes[TC358743_MODE_1920x1080].height;
	mf->code = V4L2_MBUS_FMT_YUYV8_2X8;
	mf->field = V4L2_FIELD_NONE;
	mf->colorspace = V4L2_COLORSPACE_JPEG;
}

static int h2c_set_table(struct tc358743_priv *priv, int mode)
{
	struct i2c_client *i2c_client = priv->client;
	int err = 0;

	dev_dbg(&i2c_client->dev, "%s: mode = %d\n", __func__, mode);
	disable_irq(priv->irq);
	switch (mode) {
		case TC358743_MODE_1920x1080:
			//err = h2c_write_table(i2c_client, yuv_color_bar_1080p);
			//err = h2c_write_table(i2c_client, video_mode_1080p_4lanes_colorbar);
			if ( priv->lanes == 4 )
				err = h2c_write_table(i2c_client, video_mode_1080p_4lanes);
			else
				err = h2c_write_table(i2c_client, yuv_1080p_24_30);
			
			break;
		case TC358743_MODE_1280x720:
			if ( priv->lanes == 4 )
				err = h2c_write_table(i2c_client, yuv_720p_4lanes );
			else
				err = h2c_write_table(i2c_client, yuv_720p_taec );
			break;
		case TC358743_MODE_640x480:
			if ( priv->lanes == 4 )
				err = h2c_write_table(i2c_client, yuv_480p_4lanes );
			else
				WARN(1, "2 lane 480p mode is not yet supported\n");
			//err = h2c_write_table(i2c_client, yuv_color_bar_480p_4lanes );
			break;
	}
	
	h2c_write_table(i2c_client, chip_int_init);
	tc358743_setup_interrupts(i2c_client);
	enable_irq(priv->irq);

	return err;
}

int h2c_release(struct tc358743_priv *priv)
{
	int err = 0;
	struct i2c_client *i2c_client = priv->client;

	dev_dbg(&i2c_client->dev, "%s: ++\n", __func__);


	// Put device in reset state.
	//err = h2c_write_reg_16(i2c_client, H2C_SYSTEM_CTRL_REG, 0x0F00);
	if (err) {
		dev_err(&i2c_client->dev, "%s: failed to reset device\n",
			__FUNCTION__);
		return err;
	}

	return 0;
}


static int tc358743_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct tc358743_priv *priv = to_tc358743(sd);
	int ret = 0;
#if 0
	__used int i, n = 5;
#endif
	
	dev_dbg(sd->v4l2_dev->dev, "%s(%d), last frame status: %d\n", __func__, enable, priv->last_frame_status);
	priv->streaming = 0;

	if (enable) {
#if 0 && WAIT_FOR_SYNC

		while ( priv->last_set_mode != priv->last_requested_mode || (priv->last_frame_status != 0)) {
			u8 reg;

			h2c_set_table(priv, priv->last_requested_mode);
			for( i = 0; i < 80; i++ ) {
				msleep(50);
				h2c_read_reg(priv->client, 0x8520, &reg, 1);
				printk("Status: %02x\n", reg);
			}
			if ( ( reg & 0xaf ) == 0xaf ) {
				priv->last_set_mode = priv->last_requested_mode;
				break;
			}
			if ( n-- == 0 ) {
				priv->last_set_mode = priv->last_requested_mode;
				break;
			}
			//if (ret)
				//return ret;
		}
#else
		h2c_set_table(priv, priv->last_requested_mode);
#endif
	} else {
//		tc358743_set_default_fmt(priv);
//		h2c_write_table(priv->client, chip_init);
//		tc358743_enable_hpd_interrupt(priv->client);
	}
	priv->streaming = enable;
	return ret;
}

/* Alter bus settings on camera side */
static int tc358743_set_bus_param(struct soc_camera_device *icd,
		unsigned long flags)
{
	return 0;
}

/* Request bus settings on camera side */
static unsigned long tc358743_query_bus_param(struct soc_camera_device *icd)
{
	struct soc_camera_link *icl = to_soc_camera_link(icd);

	unsigned long flags = SOCAM_PCLK_SAMPLE_RISING | SOCAM_MASTER |
		SOCAM_VSYNC_ACTIVE_HIGH | SOCAM_HSYNC_ACTIVE_HIGH |
		SOCAM_DATA_ACTIVE_HIGH | SOCAM_DATAWIDTH_10;

	return soc_camera_apply_sensor_flags(icl, flags);
}

static int tc358743_s_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *mf)
{
	struct tc358743_priv *priv = to_tc358743(sd);

	dev_dbg(sd->v4l2_dev->dev, "%s(%dx%d)\n", __func__, mf->width, mf->height);
	priv->last_requested_mode = tc358743_find_mode(priv, mf->width, mf->height);
	memcpy(&priv->mf, mf, sizeof(struct v4l2_mbus_framefmt));

	return 0;
}

static int tc358743_s_power(struct v4l2_subdev *sd, int on)
{
	struct tc358743_priv *priv = to_tc358743(sd);
	dev_dbg(sd->v4l2_dev->dev, "%s(%d)\n", __func__, on);
	if (on) {
		tc358743_s_fmt(sd, &priv->mf);
		tc358743_s_stream(sd, 1);
	} else
		tc358743_s_stream(sd, 0);

	return 0;
}

static int tc358743_try_fmt(struct v4l2_subdev *sd,
			  struct v4l2_mbus_framefmt *mf)
{
	int mode;
	struct tc358743_priv *priv = to_tc358743(sd);

	//dev_dbg(sd->v4l2_dev->dev, "%s(%dx%d)\n", __func__, mf->width, mf->height);
	mode = tc358743_find_mode(priv, mf->width, mf->height);
	mf->width = tc358743_frmsizes[mode].width;
	mf->height = tc358743_frmsizes[mode].height;

	if (mf->code != V4L2_MBUS_FMT_UYVY8_2X8 )
		mf->code = V4L2_MBUS_FMT_UYVY8_2X8 ;

	mf->field = V4L2_FIELD_NONE;
	mf->colorspace = V4L2_COLORSPACE_JPEG;

	tc358743_s_fmt(sd, mf);

	return 0;
}

static int tc358743_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			   enum v4l2_mbus_pixelcode *code)
{
	if (index >= 1)
		return -EINVAL;

	switch (index) {
	case 0:
		*code = V4L2_MBUS_FMT_UYVY8_2X8 ;
		break;
	}

	return 0;
}

/*
 * Frame size enumeration
 */
static int tc358743_enum_framesizes(struct v4l2_subdev *sd,
                struct v4l2_frmsizeenum *fsize)
{
	int i;
	int num_valid = -1;
	__u32 index = fsize->index;

	/*
	 * If a minimum width/height was requested, filter out the capture
	 * windows that fall outside that.
	 */
	//TODO revisit this
	for (i = 0; i < sizeof(tc358743_frmsizes) / sizeof(tc358743_frmsizes[0]); i++) {
		const struct v4l2_frmsize_discrete *win = &tc358743_frmsizes[index];
		if (win->width < 640)
			continue;
		if (win->height < 480)
			continue;
		if (index == ++num_valid) {
			fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
			fsize->discrete.width = win->width;
			fsize->discrete.height = win->height;
			return 0;
		}
	}

	return -EINVAL;
}



static int tc358743_cropcap(struct v4l2_subdev *sd, struct v4l2_cropcap *a)
{
	a->bounds.left		= 0;
	a->bounds.top		= 0;
	a->bounds.width		= tc358743_frmsizes[TC358743_MODE_1920x1080].width;
	a->bounds.height	= tc358743_frmsizes[TC358743_MODE_1920x1080].height;
	a->defrect		= a->bounds;
	a->type			= V4L2_BUF_TYPE_VIDEO_CAPTURE;
	a->pixelaspect.numerator	= 1;
	a->pixelaspect.denominator	= 1;

	return 0;
}

static int tc358743_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct v4l2_captureparm *cp = &parms->parm.capture;

	if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	memset(cp, 0, sizeof(*cp));
	cp->capability = V4L2_CAP_TIMEPERFRAME;
	cp->timeperframe.numerator = 1;
	cp->timeperframe.denominator = 60; //TODO - make mode-specific

	dev_dbg(&client->dev, "Frame interval: %u/%u s\n",
		cp->timeperframe.numerator, cp->timeperframe.denominator);

	return 0;
}

static int tc358743_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
//	struct i2c_client *client = v4l2_get_subdevdata(sd);
//	struct tc358743_priv *priv = to_tc358743(sd);

	return 0;
}
static int tc358743_g_crop(struct v4l2_subdev *sd, struct v4l2_crop *a)
{
	a->c.left		= 0;
	a->c.top		= 0;
	a->c.width		= tc358743_frmsizes[TC358743_MODE_1920x1080].width;
	a->c.height		= tc358743_frmsizes[TC358743_MODE_1920x1080].height;
	a->type			= V4L2_BUF_TYPE_VIDEO_CAPTURE;

	return 0;
}

static int tc358743_g_input_status(struct v4l2_subdev *sd, u32 *status) {
	struct tc358743_priv *priv = to_tc358743(sd);
//	u32 reg = 0;

	*status = 0;
	/*
	h2c_read_reg(priv->client, 0x8520, &reg, 1);
	//printk("HDMI status: %02x\n", reg);
	if ( (reg & 0xaf) != 0xaf )
		*status = *status | V4L2_IN_ST_NO_SYNC;
	*/
	if ( !priv->streaming )
		*status = *status | V4L2_IN_ST_NO_POWER;
	return 0;
}

/* Get chip identification */
static int tc358743_g_chip_ident(struct v4l2_subdev *sd,
			       struct v4l2_dbg_chip_ident *id)
{
	struct tc358743_priv *priv = to_tc358743(sd);

	id->ident = priv->ident;
	id->revision = priv->revision;

	return 0;
}

static void tc358743_clear_all_interrupts(struct i2c_client *client)
{
	h2c_write_reg(client, 0x0016, 0xff, 1);
	h2c_write_reg(client, 0x0017, 0xff, 1);
	h2c_write_reg(client, 0x0014, 0xff, 1);
	h2c_write_reg(client, 0x0015, 0xff, 1);
	h2c_write_reg(client, 0x0016, 0x33, 1);
	h2c_write_reg(client, 0x0017, 0x05, 1);
}

static void tc358743_disable_cec_rx_interrupt(struct i2c_client *client)
{
	u8 value[4] = {0, 0, 0, 0};
	// Reduce transmission overhead - only care about the first byte
	h2c_read_reg(client, 0x06C0, &value[0], 1);
	value[0] &= ~0x01;
	h2c_write_reg_multiple(client, 0x06C0, value, 4);
}

static void tc358743_clear_cec_rx_interrupt(struct i2c_client *client)
{
	u8 value[4] = {1, 0, 0, 0};
	// Clear global interrupt mask
	h2c_write_reg(client, 0x0014, 0x04, 1);
	h2c_write_reg(client, 0x0015, 0x00, 1);
	// Clear CEC rx interrupt mask 
	h2c_write_reg_multiple(client, 0x06CC, value, 4);
}

static void tc358743_enable_cec_rx_interrupt(struct i2c_client *client)
{
	u8 value[4] = {0, 0, 0, 0};
	h2c_read_reg(client, 0x06C0, &value[0], 1);
	value[0] |= 0x01;
	h2c_write_reg_multiple(client, 0x06C0, value, 4);
}

static void tc358743_disable_cec_tx_interrupt(struct i2c_client *client)
{
	u8 value[4] = {0, 0, 0, 0};
	// Reduce transmission overhead - only care about the first byte
	h2c_read_reg(client, 0x06C0, &value[0], 1);
	value[0] &= ~0x02;
	h2c_write_reg_multiple(client, 0x06C0, value, 4);
}

static void tc358743_clear_cec_tx_interrupt(struct i2c_client *client)
{
	u8 value[4] = {2, 0, 0, 0};
	// Clear global interrupt mask
	h2c_write_reg(client, 0x0014, 0x08, 1);
	h2c_write_reg(client, 0x0015, 0x00, 1);
	// Clear CEC rx interrupt mask 
	h2c_write_reg_multiple(client, 0x06CC, value, 4);
}

static void tc358743_enable_cec_tx_interrupt(struct i2c_client *client)
{
	u8 value[4] = {0, 0, 0, 0};
	h2c_read_reg(client, 0x06C0, &value[0], 1);
	value[0] |= 0x02;
	h2c_write_reg_multiple(client, 0x06C0, value, 4);
}

static int tc358743_cec_open(struct inode *inode, struct file *file)
{
	struct miscdevice *miscdev = file->private_data;
	struct tc358743_priv *priv = container_of(miscdev,
		struct tc358743_priv, misc_dev);
	file->private_data = priv;

	return 0;
}

static int tc358743_cec_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t tc358743_cec_write(struct file *file, const char __user *buffer,
	size_t count, loff_t *ppos)
{
	struct tc358743_priv *priv = file->private_data;
	u8 tx_buffer[MAX_BUFFER_SIZE];
	int i;
	u8 i2c_value[4] = {0, 0, 0, 0};
	u8 tx_ctrl_reg[4] = {0, 0, 0, 0};
	int tx_isr = 0;

	memset(tx_buffer, 0, MAX_BUFFER_SIZE*sizeof(tx_buffer[0]));

	count = count > MAX_BUFFER_SIZE ? MAX_BUFFER_SIZE : count;

	if (copy_from_user(tx_buffer, buffer, count))
		return -EFAULT;

	if ((tx_buffer[0] & 0x0f) == 0x0f) {
		tx_ctrl_reg[0] = 0x10; // Value register 0x0628 (CECTCTL) is either 0x10 or 0x00
	} else {
		tx_ctrl_reg[0] = 0;
	}

	mutex_lock(&priv->tx_mutex);

	h2c_write_reg_multiple(priv->client, 0x0628, tx_ctrl_reg, 4);

	for ( i = 0 ; i < count ; ++i ) {
		i2c_value[0] = tx_buffer[i];
		i2c_value[1] = ( i + 1 == count ) ? 1 : 0;
		h2c_write_reg_multiple(priv->client, 0x0674+i*4, i2c_value, 4);
	}

	// Trigers the transmission
	i2c_value[0] = 1;
	i2c_value[1] = 0;
	h2c_write_reg_multiple(priv->client, 0x0620, i2c_value, 4);

	mutex_unlock(&priv->tx_mutex);

	dev_err(&priv->client->dev, "%s starts waiting for the ACK or Tx completion\n", __func__);
	wait_event_interruptible_timeout(priv->tx_waitq, (tx_isr = priv->tx_isr), msecs_to_jiffies(1000));

	mutex_lock(&priv->mutex);
	priv->tx_isr = 0;
	mutex_unlock(&priv->mutex);

	if (tx_isr == H2C_CEC_TIEND)
		return count; // data transmission completed
	else if (tx_isr == H2C_CEC_TIACK)
		return 0; // no ACK detected
	else if (!tx_isr)
		return -ETIMEDOUT; // transmission timed out, nothing sent

	return 0;
}

static ssize_t tc358743_cec_read(struct file *file, char  __user *buffer,
	size_t count, loff_t *ppos)
{
	struct tc358743_priv *priv = file->private_data;
	u8 rx_buffer[MAX_BUFFER_SIZE];
	u8 reg_data[2];
	int i;

	memset(rx_buffer, 0, MAX_BUFFER_SIZE*sizeof(rx_buffer[0]));
	count = 0;

	//pr_err("%s starts waiting for the incoming data\n", __func__);
	// It waits for an irq event that indicates incoming data
	if (wait_event_interruptible(priv->rx_waitq, priv->rx_wake == 1)) {
		dev_warn(&priv->client->dev, "%s stopped by user interrupt\n", __func__);
		return -ERESTARTSYS;
	}

	// Read the buffer here.
	h2c_read_reg(priv->client, 0x06B4, (u8 *)&count, 1); // Check byte counter
	//dev_info(&client->dev, "received %u bytes data\n", count);
	for ( i = 0; i < count; ++i) {
		h2c_read_reg(priv->client, 0x0634+i*4, reg_data, 2);
		rx_buffer[i] = reg_data[0];
		if (reg_data[0] == CEC_OPCODE_NONE) {
			++count;
			rx_buffer[++i] = CEC_OPCODE_NONE; // Escape CEC_OPCODE_NONE value
		}
		if (reg_data[1] & CEC_EOM_MASK) { // end of message bit detected
			++count;
			rx_buffer[++i] = CEC_OPCODE_NONE; // Add a duff byte between messages
		}
	}

	if (copy_to_user(buffer, rx_buffer, count))
		return -EFAULT;

	mutex_lock(&priv->mutex);
	priv->rx_wake = 0;
	mutex_unlock(&priv->mutex);

	return count;
}


static long tc358743_cec_ioctl_unlocked(struct file *file, unsigned int cmd, unsigned long arg)
{
	long res = 0;
	struct tc358743_priv *priv = file->private_data;
	u8 ack_mask[4] = {0, 0, 0, 0};

	switch(cmd)
	{
	case CEC_IOCTL_SET_ACKMASK:
		if (copy_from_user(ack_mask, (void __user *)arg, LOGIC_ADDR_REG_SIZE)) {
			res = -EFAULT;
		}
		h2c_write_reg_multiple(priv->client, 0x0604, ack_mask, 4);
		break;
	case CEC_IOCTL_GET_ACKMASK:
		h2c_read_reg(priv->client, 0x0604, ack_mask, 2);
		if (copy_to_user((void __user *)arg, ack_mask, LOGIC_ADDR_REG_SIZE)) {
			res = -EFAULT;
		}
		break;
	default:
		break;
	}

	return res;
}

static DECLARE_WAIT_QUEUE_HEAD(tc358743_poll_wait);
static void wake_up_pollers(void) {
	wake_up(&tc358743_poll_wait);
}
static unsigned int tc358743_cec_poll(struct file *file, poll_table *wait)
{
	// pick up our private data
	struct tc358743_priv *priv = file->private_data;
	unsigned int ret = 0;

	poll_wait(file, &tc358743_poll_wait, wait);
	if (priv->rx_wake == 1) {
		ret |= POLLIN | POLLRDNORM;
	}
	if (priv->tx_isr) {
		ret |= POLLOUT | POLLWRNORM;
	}

	return ret;
}

static const struct file_operations tc358743_cec_fops = {
	.owner = THIS_MODULE,
	.open = tc358743_cec_open,
	.release = tc358743_cec_release,
	.read = tc358743_cec_read,
	.write = tc358743_cec_write,
	.unlocked_ioctl = tc358743_cec_ioctl_unlocked,
	.poll = tc358743_cec_poll,
};

static struct soc_camera_ops tc358743_ops = {
	.set_bus_param		= tc358743_set_bus_param,
	.query_bus_param	= tc358743_query_bus_param,
};

static struct v4l2_subdev_video_ops tc358743_video_ops = {
	.s_stream		= tc358743_s_stream,
	.s_mbus_fmt		= tc358743_s_fmt,
	.try_mbus_fmt		= tc358743_try_fmt,
	.enum_mbus_fmt		= tc358743_enum_fmt,
	.cropcap		= tc358743_cropcap,
	.g_crop			= tc358743_g_crop,
	.g_parm		    = tc358743_g_parm,
	.s_parm		    = tc358743_s_parm,
	.g_input_status = tc358743_g_input_status,
	.enum_framesizes = tc358743_enum_framesizes,
};

static struct v4l2_subdev_core_ops tc358743_core_ops = {
	.g_chip_ident		= tc358743_g_chip_ident,
	.s_power		= tc358743_s_power,
	.ioctl			= tc358743_ioctl,
};

static struct v4l2_subdev_ops tc358743_subdev_ops = {
	.core			= &tc358743_core_ops,
	.video			= &tc358743_video_ops,
};

#define IOCTL_SET_LAST_FRAME_STATUS 1

static long tc358743_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg){
		struct tc358743_priv* priv = to_tc358743(sd);
		switch(cmd) {
			case IOCTL_SET_LAST_FRAME_STATUS:
				priv->last_frame_status = (u32)arg;
				break;
		}
		return 0;
}

static void tc358743_enable_hpd_interrupt(struct i2c_client* client)
{
	u8 val;
	h2c_write_reg(client, 0x8512, 0xfc, 1); //Enable M_DDC and M_TMDS in the SYS_INT_MASK
#if 1
//	h2c_write_reg(client, 0x8513, 0xf0, 1); //Enable DC, TMDS, PXCLCK and PHYCLK ints
	h2c_write_reg(client, 0x8513, 0xd0, 1); //Enable HV Counter, DC, TMDS, PXCLCK and PHYCLK interrupt
	h2c_read_reg(client, 0x8531, &val, 1);
	h2c_write_reg(client, 0x8531, val | 0x01, 1); //Enable PHY_CTL to detect DDC
#endif											   //in PHY_CTL_0
	h2c_write_reg(client, 0x8544, 0x10, 1); 	   // DDC5V detection mode

	h2c_write_reg(client, 0x8502, 0xff, 1); 	   // clear interrupts

	h2c_write_reg(client, 0x854a, 0x01, 1); 	   // DDC5V detection mode
}

static void tc358743_enable_cec_interrupt(struct i2c_client* client)
{
	u8 value[4] = {0, 0, 0, 0};
	h2c_read_reg_multiple(client, 0x06C0, &value[0], 4);
	value[0] |= 0x03;
	// Enable Tx and Rx CEC on CECIMSK
	h2c_write_reg_multiple(client, 0x06C0, value, 4);
	// Clear interrupts
	h2c_write_reg_multiple(client, 0x06CC, value, 4);
}

static void tc358743_cec_init(struct i2c_client* client)
{
	u8 value[4] = {1, 0, 0, 0};
	h2c_write_reg_multiple(client, 0x0600, value, 4);
	h2c_write_reg_multiple(client, 0x060C, value, 4);
	h2c_write_reg_multiple(client, 0x0614, value, 4);
	value[0] = 0x10;
	h2c_write_reg_multiple(client, 0x0628, value, 4);
}

#if 1

static void tc358743_update_input_resolution(struct tc358743_priv *priv) {
	struct i2c_client *client = priv->client;
	u8 res, color;
	h2c_read_reg(client, 0x8521, &res, 1);
	h2c_read_reg(client, 0x8522, &color, 1);
	trace_printk("New resolution is %d, int? = %d\n",  res & 0xf, color & 1);
	if (priv->video_res != (res & 0xf)) {
		priv->video_res = res & 0xf;
		priv->video_update_flag = 1;
	}
}

static void tc358743_update_input_presence(struct tc358743_priv *priv) {
	struct i2c_client *client = priv->client;
        u8 val;
	h2c_read_reg(client, 0x8520, &val, 1);
	trace_printk("TMDS/DDC is %02x\n",  val);
	if (priv->video_is_present != ((val & 0x3) == 0x3)) {
		priv->video_is_present = ((val & 0x3) ==  0x3);
		priv->video_update_flag = 1;
	}
}

static void tc358743_handle_sys_int(struct tc358743_priv *priv) {
	struct i2c_client *client = priv->client;
	u8 val;
	h2c_read_reg(client, 0x8502, &val, 1);
	h2c_write_reg(client, 0x8502, 0xff, 1); 	   // clear interrupts
	trace_printk("System int mask %02x\n", val);
        tc358743_update_input_presence(priv);
}

static void tc358743_handle_clk_int(struct tc358743_priv *priv) {
	struct i2c_client *client = priv->client;
	u8 val;
	h2c_read_reg(client, 0x8503, &val, 1);
	h2c_write_reg(client, 0x8503, 0xff, 1); 	   // clear interrupts
	trace_printk("CLK int mask %02x\n", val);
	tc358743_update_input_resolution(priv);
	//if ( val & 0x10 ) { // HV counter change
	//}
}

static void tc358743_handle_packet_int(struct tc358743_priv *priv) {
	struct i2c_client *client = priv->client;
	u8 val;
	h2c_read_reg(client, 0x8504, &val, 1);
	h2c_write_reg(client, 0x8504, 0xff, 1); 	   // clear interrupts
	dev_dbg(&client->dev, "HDMI int mask %02x\n", val);
	switch(val) {
		case BIT(0):
			printk("Received AVI packet\n");
			break;
		case BIT(1):
			printk("Received AUD packet\n");
			break;
		case BIT(2):
			printk("Received MS packet\n");
			break;
		case BIT(3):
			printk("Received SPD packet\n");
			break;
		case BIT(4):
			printk("Received VS packet\n");
			break;
		case BIT(5):
			printk("Received ACP packet\n");
			break;
		case BIT(6):
			printk("Received ISRC packet\n");
			break;
		case BIT(7):
			printk("Received ISRC2 packet\n");
			break;
	}
}

static void tc358743_handle_hdmi_int(struct tc358743_priv *priv) {
	struct i2c_client *client = priv->client;
	u8 val, i;
	h2c_read_reg(client, 0x8501, &val, 1);
	if ( val & 0x01 ) { // SYS_INT
		h2c_read_reg(client, 0x8502, &val, 1);
		if ( val & 0x3 ) { // DDC or TMDS event
			tc358743_update_input_resolution(priv);
		}
	}
	for( i = 0; i < 0xf; i++ ) {
		h2c_read_reg(client, 0x8501 + i, &val, 1);
		//dev_dbg(&client->dev, "%04x => %02x\n", 0x8501 + i, val);
		h2c_write_reg(client, 0x8501 + i, 0xff, 1);
	}
}

static void tc358743_work(struct work_struct *work)
{
	struct tc358743_priv *priv = container_of(work, struct tc358743_priv, work);
	struct i2c_client *client = priv->client;
	u8 cec_rx_val, cec_tx_val;
	u16 val, err, r14val;
	u8 r8502val=0, r8503val=0, r8521val=0;

	mutex_lock(&priv->mutex);
	trace_printk("%s in\n", __func__);
	
	// Handling H2C interrupt
	// 1. Check the source (0x8501)
	// a) If
	
	err = h2c_read_reg(client, 0x0014, (u8*)&r14val, 2);
	if (  err ) {
		dev_err(&client->dev, "Failed to read interrupt status register (%d). Bailing out\n", err);
		goto eoi;
	}
	trace_printk("Detected interrupt. r14 Mask = %04x\n", r14val);

	if ( r14val &H2C_ISR_HDMI ) {

		val = 0;
		err = h2c_read_reg(client, 0x8501, (u8*)&val, 1);
		if ( err ) {
			dev_err(&client->dev, "Failed to read interrupt register (%d). Bailing out\n", err);
			goto eoi;
		}

		trace_printk("0x8501 = %02x\n", val);
		h2c_read_reg(client, 0x8502, &r8502val, 1);
		trace_printk("0x8502 = %02x\n", r8502val);
		h2c_read_reg(client, 0x8503, &r8503val, 1);
		trace_printk("0x8503 = %02x\n", r8503val);
		h2c_read_reg(client, 0x8521, &r8521val, 1);
		trace_printk("0x8521 = %02x\n", r8521val);

		if ( val & H2C_INT_SYS ) {
			trace_printk("Detected system interrupt\n");
			tc358743_handle_sys_int(priv);
		}

		if ( val & H2C_INT_PKT ) {
			trace_printk("Detected HDMI packet interrupt\n");
			tc358743_handle_packet_int(priv);
		}
		
		tc358743_handle_hdmi_int(priv);

		if ( val & H2C_INT_CLK ) {
			trace_printk("Detected clock interrupt\n");
			tc358743_handle_clk_int(priv);
		}

		if (priv->video_update_flag) {
			priv->video_update_flag = 0;
			sysfs_notify(&client->dev.kobj, NULL, "video_res");
		}
	}

	if ( r14val & H2C_ISR_CEC_R ) {
		err = h2c_read_reg(client, 0x062C, &cec_rx_val, 1);
	        if (  err ) {
			dev_err(&client->dev, "Failed to read rx interrupt register. Bailing out error %d\n", err);
			goto eoi;
		}

		tc358743_disable_cec_rx_interrupt(client);
		tc358743_clear_cec_rx_interrupt(client);
		if ( cec_rx_val & ( H2C_CEC_RIEND | H2C_CEC_RIOR ) ) {
			dev_dbg(&client->dev, "cec rx interrupt triggered\n");
			priv->rx_wake = 1;
			wake_up_interruptible(&priv->rx_waitq);
			wake_up_pollers();
			dev_dbg(&client->dev, "clear rx cec interrupt\n");
		}
		tc358743_enable_cec_rx_interrupt(client);
		trace_printk("Rx ISR is %02x\n", cec_rx_val);
	}

	if ( r14val & H2C_ISR_CEC_T ) {
		err = h2c_read_reg(client, 0x0630, &cec_tx_val, 1);
        	if (  err ) {
			dev_err(&client->dev, "Failed to read tx interrupt register. Bailing out error %d\n", err);
			goto eoi;
		}
		tc358743_disable_cec_tx_interrupt(client);
		tc358743_clear_cec_tx_interrupt(client);
		tc358743_enable_cec_tx_interrupt(client);
		if ( cec_tx_val ) {
			dev_dbg(&client->dev, "cec tx interrupt triggered\n");
			priv->tx_isr = cec_tx_val;
			wake_up_interruptible(&priv->tx_waitq);
			wake_up_pollers();
			dev_dbg(&client->dev, "clear tx cec interrupt\n");
		}
		trace_printk("Tx ISR is %02x\n", cec_tx_val);
	}
	
	tc358743_clear_all_interrupts(client);
eoi:
        mutex_unlock(&priv->mutex);

        enable_irq(priv->irq);
}

static irqreturn_t tc358743_irq(int irq, void *data)
{
	struct tc358743_priv *priv = (struct tc358743_priv *)data;
	disable_irq_nosync(priv->irq);
	schedule_work(&priv->work);
	return IRQ_HANDLED;
}

#endif

static void tc358743_setup_interrupts(struct i2c_client *client)
{
	u16 i;
	for ( i = 0x8512; i < 0x851f; i++ ) {
		h2c_write_reg(client, i, 0xff, 1); // mask interrupts
	}
	h2c_write_reg(client, 0x8512, ~0x03, 1); // enable HPD/DDC
	h2c_write_reg(client, 0x8513, ~0x30, 1); // enable HV counter change
}


static ssize_t tc358743_show_video_res(struct device *dev,
        struct device_attribute *attr, char *buf)
{
        struct tc358743_priv *priv = i2c_get_clientdata(to_i2c_client(dev));
        ssize_t ret;
		int current_resolution = priv->video_is_present ? priv->video_res : 0;
		ret = sprintf(buf, "%d\n", current_resolution);

        return ret;
}

static struct device_attribute tc358743_attributes[] = {
	{
        .attr = {.name = "video_res", .mode = 0444},
        .show = tc358743_show_video_res,
	},
};

static int tc358743_probe(struct i2c_client *client,
			 const struct i2c_device_id *did)
{
	struct tc358743_priv *priv;
	struct soc_camera_device *icd	= client->dev.platform_data;
	struct tc358743_platform_data* pdata = icd->link->priv;
	struct soc_camera_link *icl;
	u8 chipid[2];
	int ret, i;
	unsigned int irq;

	dev_dbg(&client->dev, "tc358743: %s\n", __func__);

	/* Checking soc-camera interface */
	if (!icd) {
		dev_err(&client->dev, "Missing soc-camera data!\n");
		return -EINVAL;
	}

	icl = to_soc_camera_link(icd);
	if (!icl) {
		dev_err(&client->dev, "Missing platform_data for driver\n");
		return -EINVAL;
	}
	
	/* Register TC358743 soc_camera device interface */
	priv = kzalloc(sizeof(struct tc358743_priv), GFP_KERNEL);
	if (!priv) {
		dev_err(&client->dev, "Failed to allocate private data!\n");
		return -ENOMEM;
	}
	
	priv->lanes = 4; //pdata->lanes;

	irq = client->irq;

	dev_dbg(&client->dev, "Configured irq %d\n", irq);

	INIT_WORK(&priv->work, tc358743_work);
	mutex_init(&priv->mutex);


	v4l2_i2c_subdev_init(&priv->subdev, client, &tc358743_subdev_ops);
	icd->ops = &tc358743_ops;

	/* set context info. */
	mutex_init(&priv->tx_mutex);
	priv->rx_wake = 0;
	init_waitqueue_head(&priv->rx_waitq);
	priv->tx_isr = 0;
	init_waitqueue_head(&priv->tx_waitq);

	priv->misc_dev.minor = MISC_DYNAMIC_MINOR;
	priv->misc_dev.name = TC358743_CEC_NAME;
	priv->misc_dev.fops = &tc358743_cec_fops;
	priv->misc_dev.parent = &client->dev;

	if (misc_register(&priv->misc_dev)) {
		printk(KERN_WARNING "Couldn't register device , %s.\n", TC358743_CEC_NAME);
		ret = -ENOMEM;
		goto err;
	}

	priv->subdev.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;

	priv->ident = V4L2_IDENT_TC358743;

	ret = h2c_read_reg(client, 0, &chipid[0], 2);
	if (ret) {
		dev_err(&client->dev, "Failure to read Chip ID\n");
		goto err;
	}

	priv->chip_id = (chipid[0] << 8) | chipid[1];
	priv->revision = (chipid[1] == 0x50) ? 0x1A : 0x1B;

	priv->client = client;

	tc358743_set_default_fmt(priv);

	h2c_write_table(client, chip_init);
	priv->video_res = 0;
	priv->video_is_present = 0;
	priv->video_update_flag = 0;

	// Initialise and enable CEC
	tc358743_cec_init(client);

	if (irq > 0) {
		priv->irq = irq;
		ret = request_irq(priv->irq, tc358743_irq, IRQF_TRIGGER_RISING|IRQF_ONESHOT, DRIVER_NAME,
				priv);
		if (ret)
				dev_err(&client->dev, "Failed to request irq %d: %d\n", priv->irq, ret);

		disable_irq(irq);
		tc358743_enable_hpd_interrupt(client);
		tc358743_enable_cec_interrupt(client);
		h2c_write_table(client, chip_int_init);
		enable_irq(irq);
	}

	for( i = 0; i < ARRAY_SIZE(tc358743_attributes); i++ ) {
		if ( (ret = device_create_file(&client->dev,
                &tc358743_attributes[i])) != 0 )
			dev_err(&client->dev, "Failed to create file %s: %d\n", tc358743_attributes[i].attr.name, ret);
	}

	return 0;

err:
	kfree(priv);

	return ret;
}

static int tc358743_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tc358743_priv *priv = to_tc358743(sd);
	int i;

	free_irq(priv->irq, tc358743_irq);
	misc_deregister(&priv->misc_dev);

	for( i = 0; i < ARRAY_SIZE(tc358743_attributes); i++ ) {
		device_remove_file(&client->dev,
                &tc358743_attributes[i]);
	}

	kfree(priv);
	return 0;
}

static const struct i2c_device_id tc358743_id[] = {
	{ DRIVER_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tc358743_id);

static struct i2c_driver tc358743_i2c_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.probe		= tc358743_probe,
	.remove		= tc358743_remove,
	.id_table	= tc358743_id,
};

/*
#define TC3587_CEC "tc3587_cec"
static struct platform_driver tc3587_cec_driver = {
	.driver = {
		.name = TC3587_CEC,
		.owner = THIS_MODULE,
	},
	.probe = tc3587_probe,
	.remove = tc3587_remove,
};
*/
//module_platform_driver(tc358743_i2c_driver);

static int __init tc358743_module_init(void)
{
	printk("tc358743: %s\n", __func__);
	return i2c_add_driver(&tc358743_i2c_driver);
}

static void __exit tc358743_module_exit(void)
{
	printk("tc358743: %s\n", __func__);
	i2c_del_driver(&tc358743_i2c_driver);
}


module_init(tc358743_module_init);
module_exit(tc358743_module_exit);

MODULE_DESCRIPTION("Toshiba TC358743 HDMI-CSI bridge driver");
MODULE_AUTHOR("Alex Feinman <alex@convergeddevices.net>");
MODULE_LICENSE("GPL v2");
