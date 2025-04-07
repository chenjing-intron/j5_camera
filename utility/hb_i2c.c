/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[hb_i2c]:[%s][%d]" fmt, __func__, __LINE__

#include <stdio.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>
#include <pthread.h>
#include <stdlib.h>

#include "hb_i2c.h"
#include "inc/cam_common.h"
static int32_t cam_fd[CAM_FD_NUMBER];
static int32_t cam_bus_cnt[CAM_FD_NUMBER];
static mutex_package_t *i2c_mutex_lock[CAM_FD_NUMBER];

// PRQA S 3120,1338++

int hb_i2c_get_fd_by_bus(int bus_id)
{
	if (bus_id >= (int32_t)CAM_FD_NUMBER) {
		vin_err("invalid bus_id:%d\n", bus_id);
		return -1;
	}
	return cam_fd[bus_id];
}


/**
 * @brief hb_i2c_init : init i2c
 *
 * @param [in] bus : I2C bus number
 *
 * @return ret
 */
int32_t hb_vin_i2c_init(uint32_t bus)
{
	int32_t ret = RET_OK;
	char str[I2C_BUFF_SIZE] = {0};

	if(bus >= CAM_FD_NUMBER) {
		vin_err("Invalid bus num %d", bus); /* PRQA S ALL */
		return -RET_ERROR;
	}
	if(cam_fd[bus] <= 0) {
		cam_bus_cnt[bus] = 0;
		(void)snprintf(str, sizeof(str), "/dev/i2c-%d", bus);
		cam_fd[bus]= open(str, O_RDWR); /* PRQA S 0339 */
		if(cam_fd[bus] < 0) {
			vin_err("open i2c-%d fail\n", bus); /* PRQA S ALL */
			return -RET_ERROR;
		}
		/* create i2c mutex */
		i2c_mutex_lock[bus] = camera_create_mutex_package(bus);
		if(i2c_mutex_lock[bus] != NULL) {
			vin_info("bus %d mutex lock %p shmid %d \n", bus, i2c_mutex_lock[bus],
					i2c_mutex_lock[bus]->shmid);
		} else {
			vin_err("create_mutex_package fail, bus %d mutex lock is null\n", bus);
			close(cam_fd[bus]);
			cam_fd[bus] = -1;
			return -RET_ERROR;
		}
	}
	cam_bus_cnt[bus]++;
	vin_info("bus %d cam_bus_cnt[bus] %d cam_fd[bus] %d\n",
		bus, cam_bus_cnt[bus], cam_fd[bus]);

	return ret;
}
/**
 * @brief hb_vin_i2c_deinit : init i2c
 *
 * @param [in] bus : I2C bus number
 *
 * @return ret
 */
int32_t hb_vin_i2c_deinit(uint32_t bus)
{
	if(bus >= CAM_FD_NUMBER) {
		vin_err("Invalid bus num %d", bus); /* PRQA S ALL */
		return -RET_ERROR;
	}

	cam_bus_cnt[bus]--;
	if(cam_bus_cnt[bus] == 0) {
		if (cam_fd[bus] > 0) {
			(void)close(cam_fd[bus]);
			cam_fd[bus] = -1;
		}
		if (i2c_mutex_lock[bus] != NULL) {
			pthread_mutex_lock(&i2c_mutex_lock[bus]->lock);
			pthread_mutex_unlock(&i2c_mutex_lock[bus]->lock);
			camera_destory_mutex_package(i2c_mutex_lock[bus]);
			i2c_mutex_lock[bus] = NULL;
		}
	}

	vin_info("bus %d cam_bus_cnt[bus] %d cam_fd[bus] %d\n",/* PRQA S ALL */
		bus, cam_bus_cnt[bus], cam_fd[bus]);
	return 0;
}

int32_t hb_vin_i2c_lock(uint32_t bus)
{
	if (i2c_mutex_lock[bus] == NULL) {
		vin_err("bus %d mutex is not create!!!\n", bus);
		return -1;
	}
	pthread_mutex_lock(&i2c_mutex_lock[bus]->lock);
	return 0;
}

int32_t hb_vin_i2c_unlock(uint32_t bus)
{
	if (i2c_mutex_lock[bus] == NULL) {
		vin_err("bus %d mutex is not create!!!\n", bus);
		return -1;
	}
	pthread_mutex_unlock(&i2c_mutex_lock[bus]->lock);
	return 0;
}

int32_t hb_vin_i2c_timeout_set(uint32_t bus, uint32_t timeout_ms)
{
	int32_t ret;
	uint64_t timeout_10ms;

	if ((bus >= CAM_FD_NUMBER) || (cam_bus_cnt[bus] <= 0)) {
		vin_err("Invalid bus num %d", bus); /* PRQA S ALL */
		return -RET_ERROR;
	}

	/* 0: ignore */
	if (timeout_ms == 0UL) {
		return 0;
	}
	timeout_10ms = ((uint64_t)timeout_ms + 5U) / 10U;
	ret = ioctl(cam_fd[bus], I2C_TIMEOUT, timeout_10ms); /* PRQA S ALL */
	if (ret < 0) {
		vin_err("ioctl bus %d timeout %d(%d)ms fail %d\n", /* PRQA S ALL */ /* print api */
				bus, timeout_ms, timeout_10ms * 10, ret);
		return ret;
	}
	vin_dbg("ioctl bus %d timeout %d(%d)ms\n", /* PRQA S ALL */ /* print api */
			bus, timeout_ms, timeout_10ms * 10);
	return ret;
}

int32_t hb_vin_i2c_read_reg16_data16(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr)
{
	int32_t ret;
	int32_t value;
	struct i2c_rdwr_ioctl_data data;
	uint8_t sendbuf[I2C_SMBUS_BLOCK_MAX] = {0};
	uint8_t readbuf[I2C_SMBUS_BLOCK_MAX] = {0};
    struct i2c_msg msgs[I2C_RDRW_IOCTL_MAX_MSGS] = {0};

	sendbuf[0] = (uint8_t)((reg_addr >> 8u) & 0xffu);
	sendbuf[1] = (uint8_t)(reg_addr & 0xffu);

	data.msgs = msgs; /*PRQA S 5118*/
	data.nmsgs = 2;

	data.msgs[0].len = 2;
	data.msgs[0].addr = i2c_addr;
	data.msgs[0].flags = 0;
	data.msgs[0].buf = sendbuf;

	data.msgs[1].len = 2;
	data.msgs[1].addr = i2c_addr;
	data.msgs[1].flags = 1;
	data.msgs[1].buf = readbuf;

	ret = ioctl(cam_fd[bus], I2C_RDWR, (uint64_t)&data); /* PRQA S ALL */
	if (ret < 0) {
		vin_err("ioctl r16d16 read %d@0x%02x 0x%x fail %d\n", /* PRQA S ALL */ /* print api */
			bus, i2c_addr, reg_addr, ret);
		return ret;
	}
	value = (readbuf[0] << 8) | readbuf[1]; /* PRQA S ALL */
	vin_dbg("r16d16 read %d@0x%02x 0x%x value 0x%x\n", /* PRQA S ALL */ /* print api */
			bus, i2c_addr, reg_addr, value);

	return value;
}

int32_t hb_vin_i2c_read_reg16_data8(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr)
{
	int32_t ret;
	int32_t value;
	struct i2c_rdwr_ioctl_data data;
	uint8_t sendbuf[I2C_SMBUS_BLOCK_MAX] = {0};
	uint8_t readbuf[I2C_SMBUS_BLOCK_MAX] = {0};
    struct i2c_msg msgs[I2C_RDRW_IOCTL_MAX_MSGS] = {0};

	sendbuf[0] = (uint8_t)((reg_addr >> 8u) & 0xffu);
	sendbuf[1] = (uint8_t)(reg_addr & 0xffu);

	data.msgs = msgs; /*PRQA S 5118*/
	data.nmsgs = 2;

	data.msgs[0].len = 2;
	data.msgs[0].addr = i2c_addr;
	data.msgs[0].flags = 0;
	data.msgs[0].buf = sendbuf;

	data.msgs[1].len = 1;
	data.msgs[1].addr = i2c_addr;
	data.msgs[1].flags = 1;
	data.msgs[1].buf = readbuf;

	ret = ioctl(cam_fd[bus], I2C_RDWR, (uint64_t)&data); /* PRQA S ALL */
	if (ret < 0) {
		vin_err("ioctl r16d8 read %d@0x%02x 0x%x fail %d\n", /* PRQA S ALL */ /* print api */
			bus, i2c_addr, reg_addr, ret);
		return ret;
	}
	value = readbuf[0];
	vin_dbg("r16d8 read %d@0x%02x 0x%x value 0x%x\n", /* PRQA S ALL */ /* print api */
			bus, i2c_addr, reg_addr, value);
	return value;
}

int32_t hb_vin_i2c_read_reg8_data8(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr)
{
	int32_t ret = RET_OK;
	int32_t value;
	struct i2c_rdwr_ioctl_data data;
	uint8_t sendbuf[I2C_SMBUS_BLOCK_MAX] = {0};
	uint8_t readbuf[I2C_SMBUS_BLOCK_MAX] = {0};
    struct i2c_msg msgs[I2C_RDRW_IOCTL_MAX_MSGS] = {0};

	sendbuf[0] = (uint8_t)reg_addr;

	data.msgs = msgs; /*PRQA S 5118*/
	data.nmsgs = 2;

	data.msgs[0].len = 1;
	data.msgs[0].addr = i2c_addr;
	data.msgs[0].flags = 0;
	data.msgs[0].buf = sendbuf;

	data.msgs[1].len = 1;
	data.msgs[1].addr = i2c_addr;
	data.msgs[1].flags = 1;
	data.msgs[1].buf = readbuf;

	ret = ioctl(cam_fd[bus], I2C_RDWR, (uint64_t)&data); /* PRQA S ALL */
	if (ret < 0) {
		vin_err("ioctl r8d8 read %d@0x%02x 0x%x fail %d\n", /* PRQA S ALL */ /* print api */
			bus, i2c_addr, reg_addr, ret);
		return ret;
	}
	value = readbuf[0];
	vin_dbg("r8d8 read %d@0x%02x 0x%x value 0x%x\n", /* PRQA S ALL */ /* print api */
			bus, i2c_addr, reg_addr, value);

	return value;
}

int32_t hb_vin_i2c_read_reg8_data16(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr)
{
	int32_t ret;
	int32_t value;
	struct i2c_rdwr_ioctl_data data;
	uint8_t sendbuf[I2C_SMBUS_BLOCK_MAX] = {0};
	uint8_t readbuf[I2C_SMBUS_BLOCK_MAX] = {0};
    struct i2c_msg msgs[I2C_RDRW_IOCTL_MAX_MSGS] = {0};

	sendbuf[0] = (uint8_t)reg_addr;

	data.msgs = msgs; /*PRQA S 5118*/
	data.nmsgs = 2;

	data.msgs[0].len = 1;
	data.msgs[0].addr = i2c_addr;
	data.msgs[0].flags = 0;
	data.msgs[0].buf = sendbuf;

	data.msgs[1].len = 2;
	data.msgs[1].addr = i2c_addr;
	data.msgs[1].flags = 1;
	data.msgs[1].buf = readbuf;

	ret = ioctl(cam_fd[bus], I2C_RDWR, (uint64_t)&data); /* PRQA S ALL */
	if (ret < 0) {
		vin_err("ioctl r8d16 read %d@0x%02x 0x%x fail %d\n", /* PRQA S ALL */ /* print api */
			bus, i2c_addr, reg_addr, ret);
		return ret;
	}
	value = (readbuf[0] << 8) | readbuf[1]; /* PRQA S ALL */
	vin_dbg("r8d16 read %d@0x%02x 0x%x value 0x%x\n", /* PRQA S ALL */ /* print api */
			bus, i2c_addr, reg_addr, value);

	return value;
}

int32_t hb_vin_i2c_write_reg16_data16(uint32_t bus, uint8_t i2c_addr,
		uint16_t reg_addr, uint16_t value)
{
	int32_t ret;
	struct i2c_rdwr_ioctl_data data;
	uint8_t sendbuf[I2C_SMBUS_BLOCK_MAX] = {0};
    struct i2c_msg msgs[I2C_RDRW_IOCTL_MAX_MSGS] = {0};

	sendbuf[0] = (uint8_t)((reg_addr >> 8u) & 0xffu);
	sendbuf[1] = (uint8_t)(reg_addr & 0xffu);
	sendbuf[2] = (uint8_t)((value >> 8u) & 0xffu);
	sendbuf[3] = (uint8_t)(value & 0xffu);

	data.msgs = msgs;
	data.nmsgs = 1;

	data.msgs[0].len = 4;
	data.msgs[0].addr = i2c_addr;
	data.msgs[0].flags = 0;
	data.msgs[0].buf = sendbuf;

	ret = ioctl(cam_fd[bus], I2C_RDWR, (uint64_t)&data); /* PRQA S ALL */
	if (ret < 0) {
		vin_warn("ioctl r16d16 write %d@0x%02x 0x%x 0x%x fail %d\n", /* PRQA S ALL */ /* print api */
			bus, i2c_addr, reg_addr, value, ret);
		return ret;
	}
	return RET_OK;
}

int32_t hb_vin_i2c_write_reg16_data8(uint32_t bus, uint8_t i2c_addr,
		uint16_t reg_addr, uint8_t value)
{
	int32_t ret;
	struct i2c_rdwr_ioctl_data data;
	uint8_t sendbuf[I2C_SMBUS_BLOCK_MAX] = {0};
    struct i2c_msg msgs[I2C_RDRW_IOCTL_MAX_MSGS] = {0};

	sendbuf[0] = (uint8_t)((reg_addr >> 8u) & 0xffu);
	sendbuf[1] = (uint8_t)(reg_addr & 0xffu);
	sendbuf[2] = (uint8_t)(value & 0xffu);

	data.msgs = msgs;
	data.nmsgs = 1;

	data.msgs[0].len = 3;
	data.msgs[0].addr = i2c_addr;
	data.msgs[0].flags = 0;
	data.msgs[0].buf = sendbuf;

	if (i2c_addr == 0x11 || i2c_addr == 0x41 || i2c_addr == 0x29 || i2c_addr == 0x52 || i2c_addr == 0x6b)
	{
		char my_buf[200];
		snprintf(my_buf, sizeof(my_buf), "echo \"--CHJ--:addr=0x%x, reg_addr=0x%x, value=0x%x\" > /dev/kmsg", i2c_addr, reg_addr, value);
		system(my_buf);
	}

	ret = ioctl(cam_fd[bus], I2C_RDWR, (uint64_t)&data); /* PRQA S ALL */
	if (ret < 0) {
		vin_warn("ioctl r16d8 write %d@0x%02x 0x%x 0x%x fail %d\n", /* PRQA S ALL */ /* print api */
			bus, i2c_addr, reg_addr, value, ret);
		return ret;
	}
	return RET_OK;
}

int32_t hb_vin_i2c_write_reg8_data16(uint32_t bus, uint8_t i2c_addr,
		uint16_t reg_addr, uint16_t value)
{
	int32_t ret;
	struct i2c_rdwr_ioctl_data data;
	uint8_t sendbuf[I2C_SMBUS_BLOCK_MAX] = {0};
    struct i2c_msg msgs[I2C_RDRW_IOCTL_MAX_MSGS] = {0};

	sendbuf[0] = (uint8_t)reg_addr;
	sendbuf[1] = (uint8_t)((value >> 8u) & 0xffu);
	sendbuf[2] = (uint8_t)(value & 0xffu);

	data.msgs = msgs;
	data.nmsgs = 1;

	data.msgs[0].len = 3;
	data.msgs[0].addr = i2c_addr;
	data.msgs[0].flags = 0;
	data.msgs[0].buf = sendbuf;

	ret = ioctl(cam_fd[bus], I2C_RDWR, (uint64_t)&data); /* PRQA S ALL */
	if (ret < 0) {
		vin_warn("ioctl r8d16 write %d@0x%02x 0x%x 0x%x fail %d\n", /* PRQA S ALL */ /* print api */
			bus, i2c_addr, reg_addr, value, ret);
		return ret;
	}
	return RET_OK;
}

int32_t hb_vin_i2c_write_reg8_data8(uint32_t bus, uint8_t i2c_addr,
		uint16_t reg_addr, uint8_t value)
{
	int32_t ret;
	struct i2c_rdwr_ioctl_data data;
	uint8_t sendbuf[I2C_SMBUS_BLOCK_MAX] = {0};
    struct i2c_msg msgs[I2C_RDRW_IOCTL_MAX_MSGS] = {0};

	sendbuf[0] = (uint8_t)reg_addr;
	sendbuf[1] = value;

	data.msgs = msgs;
	data.nmsgs = 1;

	data.msgs[0].len = 2;
	data.msgs[0].addr = i2c_addr;
	data.msgs[0].flags = 0;
	data.msgs[0].buf = sendbuf;

	ret = ioctl(cam_fd[bus], I2C_RDWR, (uint64_t)&data); /* PRQA S ALL */
	if (ret < 0) {
		vin_warn("ioctl r8d8 write %d@0x%02x 0x%x 0x%x fail %d\n", /* PRQA S ALL */ /* print api */
			bus, i2c_addr, reg_addr, value, ret);
		return ret;
	}

	return RET_OK;
}

int32_t hb_vin_i2c_write_block(uint32_t bus, uint8_t i2c_addr,
	uint16_t reg_addr, uint32_t value, uint8_t cnt)
{
	int32_t ret;
	struct i2c_rdwr_ioctl_data data;
	uint8_t sendbuf[I2C_SMBUS_BLOCK_MAX] = {0};
    struct i2c_msg msgs[I2C_RDRW_IOCTL_MAX_MSGS] = {0};

	if (cnt > 4u)
		cnt = 4;

	data.msgs = msgs; /*PRQA S 5118*/
	data.nmsgs = cnt;

	data.msgs[0].len = 3;
	data.msgs[0].addr = i2c_addr;
	data.msgs[0].flags = 0;
	data.msgs[0].buf = sendbuf;
	data.msgs[0].buf[0] = (uint8_t)((reg_addr >> 8u) & 0xffu);
	data.msgs[0].buf[1] = (uint8_t)(reg_addr & 0xffu);
	data.msgs[0].buf[2] = (uint8_t)value & 0xffu;

	data.msgs[1].len = 3;
	data.msgs[1].addr = i2c_addr;
	data.msgs[1].flags = 0;
	data.msgs[1].buf = &sendbuf[3];
	data.msgs[1].buf[0] = (uint8_t)(((reg_addr+1u) >> 8u) & 0xffu);
	data.msgs[1].buf[1] = (uint8_t)((reg_addr+1u) & 0xffu);
	data.msgs[1].buf[2] = (uint8_t)((value >> 8u) & 0xffu);

	data.msgs[2].len = 3;
	data.msgs[2].addr = i2c_addr;
	data.msgs[2].flags = 0;
	data.msgs[2].buf = &sendbuf[6];
	data.msgs[2].buf[0] = (uint8_t)(((reg_addr+2u) >> 8u) & 0xffu);
	data.msgs[2].buf[1] = (uint8_t)((reg_addr+2u) & 0xffu);
	data.msgs[2].buf[2] = (uint8_t)((value >> 16u) & 0xffu);

	data.msgs[3].len = 3;
	data.msgs[3].addr = i2c_addr;
	data.msgs[3].flags = 0;
	data.msgs[3].buf = &sendbuf[9];
	data.msgs[3].buf[0] = (uint8_t)(((reg_addr+3u) >> 8u) & 0xffu);
	data.msgs[3].buf[1] = (uint8_t)((reg_addr+3u) & 0xffu);
	data.msgs[3].buf[2] = (uint8_t)((value >> 24u) & 0xffu);

	ret = ioctl(cam_fd[bus], I2C_RDWR, (uint64_t)&data); /* PRQA S ALL */
	if (ret < 0)
		vin_err("ioctl write %d@0x%02x block%u 0x%x=0x%x fail %d\n", /* PRQA S ALL */ /* print api */
			bus, i2c_addr, cnt, reg_addr, value, ret);

	return ret;
}

int32_t hb_vin_i2c_read_block_reg16(uint32_t bus, uint8_t i2c_addr,
		uint16_t reg_addr, unsigned char *buf, uint32_t count)
{
    int32_t ret;
	struct i2c_rdwr_ioctl_data data;
	uint8_t sendbuf[I2C_SMBUS_BLOCK_MAX] = {0};
    struct i2c_msg msgs[I2C_RDRW_IOCTL_MAX_MSGS] = {0};

	data.msgs = msgs;
	data.nmsgs = 2;

	data.msgs[0].len = 2;
	data.msgs[0].addr = i2c_addr;
	data.msgs[0].flags = 0;
	data.msgs[0].buf = sendbuf;
	data.msgs[0].buf[0] = (uint8_t)((reg_addr >> 8u) & 0xffu);
	data.msgs[0].buf[1] = (uint8_t)(reg_addr & 0xffu);

	data.msgs[1].len = (uint16_t)count;
	data.msgs[1].addr = i2c_addr;
	data.msgs[1].flags = 1;
	data.msgs[1].buf = buf;

	ret = ioctl(cam_fd[bus], I2C_RDWR, (uint64_t)&data); /* PRQA S ALL */
	if (ret < 0)
		vin_err("ioctl read %d@0x%02x r16block%u 0x%x fail %d\n", /* PRQA S ALL */ /* print api */
			bus, i2c_addr, count, reg_addr, ret);
	return ret;
}

int32_t hb_vin_i2c_read_block_reg8(uint32_t bus, uint8_t i2c_addr,
		uint16_t reg_addr, unsigned char *buf, uint32_t count)
{
    int32_t ret;
	struct i2c_rdwr_ioctl_data data;
	uint8_t sendbuf[I2C_SMBUS_BLOCK_MAX] = {0};
    struct i2c_msg msgs[I2C_RDRW_IOCTL_MAX_MSGS] = {0};

	data.msgs = msgs; /*PRQA S 5118*/
	data.nmsgs = 2;

	data.msgs[0].len = 2;
	data.msgs[0].addr = i2c_addr;
	data.msgs[0].flags = 0;
	data.msgs[0].buf = sendbuf;
	data.msgs[0].buf[0] = (uint8_t)(reg_addr & 0xffu);

	data.msgs[1].len = (uint16_t)count;
	data.msgs[1].addr = i2c_addr;
	data.msgs[1].flags = 1;
	data.msgs[1].buf = buf;

	ret = ioctl(cam_fd[bus], I2C_RDWR, (uint64_t)&data); /* PRQA S ALL */
	if (ret < 0)
		vin_err("ioctl read %d@0x%02x r8block%u 0x%x fail %d\n", /* PRQA S ALL */ /* print api */
			bus, i2c_addr, count, reg_addr, ret);
	return ret;
}

int32_t hb_vin_i2c_write_block_reg16(uint32_t bus, uint8_t i2c_addr,
	uint16_t reg_addr, const char *buf, uint32_t count)
{
	int32_t ret;
	struct i2c_rdwr_ioctl_data data;
	uint8_t sendbuf[I2C_SMBUS_BLOCK_MAX] = {0};
    struct i2c_msg msgs[I2C_RDRW_IOCTL_MAX_MSGS] = {0};

	if (count + 2 > (uint32_t)sizeof(sendbuf)) {
		vin_err("count+2 %d larger than max %d\n", count + 2, (uint32_t)sizeof(sendbuf));
		return -1;
	}

	data.msgs = msgs; /*PRQA S 5118*/
	data.nmsgs = 1;

	data.msgs[0].len = (uint16_t)(2u + count);
	data.msgs[0].addr = i2c_addr;
	data.msgs[0].flags = 0;
	data.msgs[0].buf = sendbuf;
	sendbuf[0] = (uint8_t)((reg_addr >> 8u) & 0xffu);
	sendbuf[1] = (uint8_t)(reg_addr & 0xffu);
	(void)memcpy(sendbuf+2, buf, count);    /* PRQA S ALL */

	ret = ioctl(cam_fd[bus], I2C_RDWR, (uint64_t)&data); /* PRQA S ALL */ /* PRQA S ALL */
	if (ret < 0)
		vin_err("ioctl write %d@0x%02x r16block%u 0x%x fail %d\n", /* PRQA S ALL */ /* print api */
			bus, i2c_addr, count, reg_addr, ret);

	return ret;
}

int32_t hb_vin_i2c_write_block_reg8(uint32_t bus, uint8_t i2c_addr,
		uint16_t reg_addr, unsigned char *buf, uint32_t count)
{
	int32_t ret;
	struct i2c_rdwr_ioctl_data data;
	uint8_t sendbuf[I2C_SMBUS_BLOCK_MAX] = {0};
    struct i2c_msg msgs[I2C_RDRW_IOCTL_MAX_MSGS] = {0};

	data.msgs = msgs;
	data.nmsgs = 2;

	data.msgs[0].len = 2;
	data.msgs[0].addr = i2c_addr;
	data.msgs[0].flags = 0;
	data.msgs[0].buf = sendbuf;
	data.msgs[0].buf[0] = (uint8_t)(reg_addr & 0xffu);

	data.msgs[1].len = (uint16_t)count;
	data.msgs[1].addr = i2c_addr;
	data.msgs[1].flags = 0;
	data.msgs[1].buf = buf;

	ret = ioctl(cam_fd[bus], I2C_RDWR, (uint64_t)&data); /* PRQA S ALL */
	if (ret < 0)
		vin_err("ioctl write %d@0x%02x r8block%u 0x%x fail %d\n", /* PRQA S ALL */ /* print api */
			bus, i2c_addr, count, reg_addr, ret);
	return ret;
}
// PRQA S --

