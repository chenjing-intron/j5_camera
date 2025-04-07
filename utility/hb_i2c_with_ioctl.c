/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2021 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/

#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <byteswap.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <inc/cam_common.h>
#include "./hb_i2c.h"

extern int32_t hb_i2c_get_fd_by_bus(int32_t bus_id);

uint32_t get_size(int32_t size)
{
    return (size > 0 ? size : 0);
}
// ioctl wrapper
static int32_t do_ioctl(int32_t fd, unsigned long request, unsigned long data)
{
    int32_t ret = ioctl(fd, request, data);
    if (ret >= 0) {
        ret = 0;
    }
    return ret;
}

/**
 * @brief check enable i2c dump flag
 * @return 0: disable, 1: enable
 */
static int is_enable_dump_i2c_op()
{
    static int enable_flag = -1;
    if (-1 == enable_flag) {
        enable_flag = 0;
        const char* str = getenv("HB_CAM_I2C_OP_DUMP");
        if (str) {
            enable_flag = (0 == atoi(str)) ? 0 : 1;
        }
        vin_info("enable_dump_i2c_op %d\n", enable_flag);
    }
    return enable_flag;
}

/**
 * @brief hb_i2c_write : write data block via i2c
 * @param [in] bus : I2C bus number
 * @param [in] i2c_addr : I2C slave address
 * @param [in] reg_addr : I2C device's register address
 * @param [in] reg_addr_len : The length of I2C device's register address, unit:byte, e.g: 1,2,...
 * @param [in] buf : the data buffer which will be send out
 * @param [in] buf_len : the length of data buffer
 * @return ret : 0: OKAY, other: error occurred
 */
int32_t hb_i2c_write(
    int32_t bus, uint8_t i2c_addr,
    const uint8_t *reg_addr, size_t reg_addr_len,
    const uint8_t *buf, uint8_t buf_len)
{
    int32_t ret = 0;

    int32_t msgs_buf_max = reg_addr_len + buf_len;
    if (msgs_buf_max > I2C_SMBUS_BLOCK_MAX) {
        vin_err("unsupport write reg_addr with len %d\n", msgs_buf_max);
        return -1;
    }

    // dump write details as i2ctransfer format
    char fmt_buf[128] = {0};
    if (is_enable_dump_i2c_op()) {
        int32_t fmt_buf_offset = 0;
        fmt_buf_offset += snprintf(fmt_buf+fmt_buf_offset,
            get_size(sizeof(fmt_buf)-fmt_buf_offset),
            "%d w%ld@0x%02x", bus, reg_addr_len + buf_len, i2c_addr);
        for (size_t i = 0; i < reg_addr_len; i++) {
            fmt_buf_offset += snprintf(fmt_buf+fmt_buf_offset,
                get_size(sizeof(fmt_buf)-fmt_buf_offset), " 0x%02x", reg_addr[i]);
        }
        for (size_t i = 0; i < buf_len; i++) {
            fmt_buf_offset += snprintf(fmt_buf+fmt_buf_offset,
                get_size(sizeof(fmt_buf)-fmt_buf_offset), " 0x%02x", buf[i]);
        }
        vin_dbg("%s\n", fmt_buf);
    }

    //
    char msgs_buf[I2C_SMBUS_BLOCK_MAX] = {0};
    struct i2c_msg msgs[I2C_RDRW_IOCTL_MAX_MSGS] = {0};

    int32_t offset = 0;
    char *raw_buf = msgs_buf;
    memcpy((uint8_t *)raw_buf, reg_addr, reg_addr_len);
    offset += (int32_t)reg_addr_len;
    memcpy((uint8_t *)raw_buf+offset, buf, buf_len);
    msgs[0].addr = i2c_addr;
    msgs[0].flags = 0;
    msgs[0].len = (uint16_t)reg_addr_len+buf_len;
    msgs[0].buf = raw_buf;

    struct i2c_rdwr_ioctl_data data;
    data.msgs = msgs;
    data.nmsgs = 1;
    ret = do_ioctl(
        hb_i2c_get_fd_by_bus(bus), I2C_RDWR, (unsigned long)&data);
    int32_t errno_ = errno;
    if (ret < 0) {
        vin_warn("i2c ioctl failed: %d, errno:%d(%s), cmd(%s)\n",
            ret, errno_, strerror(errno_), fmt_buf);
    }

    return ret;
}

/**
 * @brief hb_i2c_read : read data block via i2c
 * @param [in] bus : I2C bus number
 * @param [in] i2c_addr : I2C slave address
 * @param [in] reg_addr : I2C device's register address
 * @param [in] reg_addr_len : The length of I2C device's register address, unit:byte, e.g: 1,2,...
 * @param [out] buf : the data buffer which will be read in
 * @param [in] buf_len : the length of data buffer
 * @return ret : 0: OKAY, other: error occurred
 */
int32_t hb_i2c_read(
    int32_t bus, uint8_t i2c_addr,
    const uint8_t *reg_addr, size_t reg_addr_len, uint8_t *buf, uint8_t buf_len)
{
    int32_t ret = 0;

    const int32_t nmsgs = 2;
    struct i2c_msg msgs[nmsgs];
    char msgs_buf[I2C_SMBUS_BLOCK_MAX] = {0};
    
    memcpy((uint8_t *)msgs_buf, reg_addr, reg_addr_len);
    msgs[0].addr = i2c_addr;
    msgs[0].flags = 0;
    msgs[0].len = (uint16_t)reg_addr_len;
    msgs[0].buf = msgs_buf;

    // build read msg
    msgs[1].addr = i2c_addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = buf_len;
    msgs[1].buf = buf;

    // send out read ioctl cmd
    struct i2c_rdwr_ioctl_data data;
    data.msgs = msgs;
    data.nmsgs = (uint32_t)nmsgs;
    ret = do_ioctl(
        hb_i2c_get_fd_by_bus(bus), I2C_RDWR, (unsigned long)&data);
    int32_t errno_ = errno;
    if (ret < 0) {
        vin_info("i2c ioctl failed: %d, errno:%d(%s)n",
            ret, errno_, strerror(errno_));
    }
    else
    {
	vin_info("chenjing----\n");
    }

    return ret;
}
#if 0
int32_t hb_i2c_read_reg16_data16(int32_t bus, uint8_t i2c_addr, uint16_t reg_addr)
{
    uint16_t data = 0;
    reg_addr = bswap_16(reg_addr);
    int32_t ret = hb_i2c_read(
        bus, i2c_addr, (uint8_t *)&reg_addr, sizeof(reg_addr),
        (char*)&data, sizeof(data));
    if (ret != 0) {
        return -1;
    }
    return bswap_16(data);
}

int32_t hb_i2c_read_reg16_data8(int32_t bus, uint8_t i2c_addr, uint16_t reg_addr)
{
    uint8_t data = 0;
    reg_addr = bswap_16(reg_addr);
    int32_t ret = hb_i2c_read(bus, i2c_addr, (uint8_t *)&reg_addr, sizeof(reg_addr),
        (char*)&data, sizeof(data));
    if (ret != 0) {
        return -1;
    }
    return data;
}

int32_t hb_i2c_read_reg8_data8(int32_t bus, uint8_t i2c_addr, uint16_t reg_addr)
{
    uint8_t data = 0;
    int32_t ret = hb_i2c_read(bus, i2c_addr, (uint8_t *)&reg_addr, 1,
        (char*)&data, sizeof(data));
    if (ret != 0) {
        return -1;
    }
    return data;
}

int32_t hb_i2c_read_reg8_data16(int32_t bus, uint8_t i2c_addr, uint16_t reg_addr)
{
    uint16_t data = 0;
    int32_t ret = hb_i2c_read(bus, i2c_addr, (uint8_t *)&reg_addr, 1,
        (char*)&data, sizeof(data));
    if (ret != 0) {
        return -1;
    }
    return bswap_16(data);
}

int32_t hb_i2c_write_reg16_data16(
    int32_t bus, uint8_t i2c_addr, uint16_t reg_addr, uint16_t value)
{
    reg_addr = bswap_16(reg_addr);
    value = bswap_16(value);
    return hb_i2c_write(bus, i2c_addr,
        (const uint8_t *)&reg_addr, sizeof(reg_addr),
        (const uint8_t *)&value, sizeof(value));
}

int32_t hb_i2c_write_reg16_data8(
    int32_t bus, uint8_t i2c_addr, uint16_t reg_addr, uint8_t value)
{
    reg_addr = bswap_16(reg_addr);
    return hb_i2c_write(bus, i2c_addr,
        (const uint8_t *)&reg_addr, sizeof(reg_addr),
        (const uint8_t *)&value, sizeof(value));
}

int32_t hb_i2c_write_reg8_data16(
    int32_t bus, uint8_t i2c_addr, uint16_t reg_addr, uint16_t value)
{
    value = bswap_16(value);
    return hb_i2c_write(bus, i2c_addr,
        (const uint8_t *)&reg_addr, 1, (const uint8_t *)&value, sizeof(value));
}

int32_t hb_i2c_write_reg8_data8(
    int32_t bus, uint8_t i2c_addr, uint16_t reg_addr, uint8_t value)
{
    return hb_i2c_write(bus, i2c_addr,
        (const uint8_t *)&reg_addr, 1, (const uint8_t *)&value, sizeof(value));
}
#endif
