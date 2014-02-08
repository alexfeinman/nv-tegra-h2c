/*
 * Copyright (C) 2010 Motorola, Inc.
 * Copyright (C) 2011 NVIDIA Corporation.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#ifndef __TC358743_H__
#define __TC358743_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define TC358743_IOCTL_SET_MODE			_IOW('o', 1, struct tc358743_mode)
#define TC358743_IOCTL_SET_FRAME_LENGTH	_IOW('o', 2, __u32)
#define TC358743_IOCTL_SET_COARSE_TIME	_IOW('o', 3, __u32)
#define TC358743_IOCTL_SET_GAIN			_IOW('o', 4, __u16)
#define TC358743_IOCTL_GET_STATUS			_IOR('o', 5, __u8)
#define TC358743_IOCTL_SET_BINNING    	_IOW('o', 6, __u8)
#define TC358743_IOCTL_TEST_PATTERN		_IOW('o', 7, enum tc358743_test_pattern)
#define TC358743_IOCTL_SET_GROUP_HOLD	    _IOW('o', 8, struct tc358743_ae)
#define TC358743_IOCTL_SET_CAMERA_MODE	_IOW('o', 10, __u32)
#define TC358743_IOCTL_GET_SENSORDATA		_IOR('o', 11, struct tc358743_sensordata)

/* TC358743 registers */
#define TC358743_SRM_GRUP_ACCESS          (0x3212)
#define TC358743_ARRAY_CONTROL_01         (0x3621)
#define TC358743_ANALOG_CONTROL_D         (0x370D)
#define TC358743_TIMING_TC_REG_18         (0x3818)
#define TC358743_TIMING_CONTROL_HS_HIGH   (0x3800)
#define TC358743_TIMING_CONTROL_HS_LOW    (0x3801)
#define TC358743_TIMING_CONTROL_VS_HIGH   (0x3802)
#define TC358743_TIMING_CONTROL_VS_LOW    (0x3803)
#define TC358743_TIMING_HW_HIGH           (0x3804)
#define TC358743_TIMING_HW_LOW            (0x3805)
#define TC358743_TIMING_VH_HIGH           (0x3806)
#define TC358743_TIMING_VH_LOW            (0x3807)
#define TC358743_TIMING_TC_REG_18         (0x3818)
#define TC358743_TIMING_HREFST_MAN_HIGH   (0x3824)
#define TC358743_TIMING_HREFST_MAN_LOW    (0x3825)
#define TC358743_H_BINNING_BIT            (1 << 7)
#define TC358743_H_SUBSAMPLING_BIT        (1 << 6)
#define TC358743_V_BINNING_BIT            (1 << 6)
#define TC358743_V_SUBSAMPLING_BIT        (1 << 0)
#define TC358743_GROUP_HOLD_BIT		(1 << 7)
#define TC358743_GROUP_LAUNCH_BIT		(1 << 5)
#define TC358743_GROUP_HOLD_END_BIT	(1 << 4)
#define TC358743_GROUP_ID(id)		(id)

#define TC358743_CEC_NAME "tc3587_cec"

struct tc358743_mode {
	int xres;
	int yres;
	__u32 frame_length;
	__u32 coarse_time;
	__u16 gain;
};

struct tc358743_ae {
	__u32 frame_length;
	__u8 frame_length_enable;
	__u32 coarse_time;
	__u8 coarse_time_enable;
	__s32 gain;
	__u8 gain_enable;
};

struct tc358743_platform_data {
	int (*power_on)(void);
	int (*power_off)(void);
	int lanes;
};

#endif  /* __TC358743_H__ */

