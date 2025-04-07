/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef __HB_I2C_H__
#define __HB_I2C_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include <sys/shm.h>
#include <pthread.h>

#define CAM_FD_NUMBER 10u
#define I2C_BUFF_SIZE 12

/* i2c mutex lock */
typedef struct mutex_package {
	pthread_mutex_t lock;
	pthread_mutexattr_t lock_attr;
	int shmid;
} mutex_package_t;

static inline int camera_create_flag(int index_of_key, unsigned int length)
{
	char *file_path = "/tmp";
	key_t key = ftok(file_path, index_of_key);

	int shmid = shmget(key, length, IPC_CREAT | 0666);
	return shmid;
}

static inline mutex_package_t* camera_create_mutex_package(int index)
{
	int shmid = camera_create_flag(index, sizeof(mutex_package_t));
	mutex_package_t *mp = (mutex_package_t*)shmat(shmid, NULL, SHM_R | SHM_W);
	if(mp == NULL) {
		return NULL;
	}
	mp->shmid = shmid;
	// Initialize the lock state and set the state to -- process sharing
	pthread_mutexattr_init(&(mp->lock_attr));
	pthread_mutexattr_setpshared(&(mp->lock_attr), PTHREAD_PROCESS_SHARED);
	// Initialize lock with lock state
	pthread_mutex_init(&(mp->lock), &(mp->lock_attr));
	return mp;
}

static inline int camera_destory_mutex_package(mutex_package_t *mp)
{
	if (mp) {
		pthread_mutex_destroy(&(mp->lock));
		pthread_mutexattr_destroy(&(mp->lock_attr));
#if 0
		ret = shmctl(mp->flag, IPC_RMID, NULL);
		HANDLE_RET_ERR(ret, "shmctl");
#endif
		shmdt(mp);
		mp = NULL;
	}
	return 0;
}

int32_t hb_vin_i2c_init(uint32_t bus);
int32_t hb_vin_i2c_deinit(uint32_t bus);

int32_t hb_vin_i2c_lock(uint32_t bus);
int32_t hb_vin_i2c_unlock(uint32_t bus);

int32_t hb_vin_i2c_read_reg16_data16(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr);
int32_t hb_vin_i2c_read_reg16_data8(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr);
int32_t hb_vin_i2c_read_reg8_data8(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr);
int32_t hb_vin_i2c_read_reg8_data16(uint32_t bus, uint8_t i2c_addr, uint16_t reg_addr);
int32_t hb_vin_i2c_write_reg16_data16(uint32_t bus, uint8_t i2c_addr,
		uint16_t reg_addr, uint16_t value);
int32_t hb_vin_i2c_write_reg16_data8(uint32_t bus, uint8_t i2c_addr,
		uint16_t reg_addr, uint8_t value);
int32_t hb_vin_i2c_write_reg8_data16(uint32_t bus, uint8_t i2c_addr,
		uint16_t reg_addr, uint16_t value);
int32_t hb_vin_i2c_write_reg8_data8(uint32_t bus, uint8_t i2c_addr,
		uint16_t reg_addr, uint8_t value);
int32_t hb_vin_i2c_write_block(uint32_t bus, uint8_t i2c_addr,
		uint16_t reg_addr, uint32_t value, uint8_t cnt);
int32_t hb_vin_i2c_read_block_reg16(uint32_t bus, uint8_t i2c_addr,
			uint16_t reg_addr, unsigned char *buf, uint32_t count);
int32_t hb_vin_i2c_read_block_reg8(uint32_t bus, uint8_t i2c_addr,
		uint16_t reg_addr, unsigned char *buf, uint32_t count);
int32_t hb_vin_i2c_write_block_reg16(uint32_t bus, uint8_t i2c_addr,
		uint16_t reg_addr, const char *buf, uint32_t count);
int32_t hb_vin_i2c_write_block_reg8(uint32_t bus,	uint8_t i2c_addr,
		uint16_t reg_addr, unsigned char *buf, uint32_t count);

int32_t hb_vin_i2c_timeout_set(uint32_t bus, uint32_t timeout_ms);

int32_t hb_i2c_write(
    int32_t bus, uint8_t i2c_addr,
    const uint8_t *reg_addr, size_t reg_addr_len,
    const uint8_t *buf, uint8_t buf_len);
int32_t hb_i2c_read(
    int32_t bus, uint8_t i2c_addr,
    const uint8_t *reg_addr, size_t reg_addr_len, uint8_t *buf, uint8_t buf_len);

#ifdef __cplusplus
}
#endif

#endif
