/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2022 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef UTILITY_CAMERA_DIAG_INC_CAM_SENSOR_DIAG_H_
#define UTILITY_CAMERA_DIAG_INC_CAM_SENSOR_DIAG_H_

#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <linux/i2c-dev.h>

#include "../../hb_cam_utility.h"
#include "./camera_diag_utility.h"

#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define MAX_BUF 128

#define BIT(i)		(1 << (i))

#define ERRB_POLLING_PERIOD_MS	200
#define PIN_RECOVERY_DELAY_TIME	2000000

#define FPS_MONITOR_PERIOD_US 1000000
#define FCNT_RETRY_MAX 4
#define FCNT_ERR_RANGE 2

#define DEBOUNCE_CNT    3

// Define Diag Sensor info
#define CAMERA_DIAG_OV_NAME "ovx"
#define CAMERA_DIAG_SONY_NAME "isx"
#define CAMERA_DIAG_ONSEMI_NAME "ar0"

// Define ModuleID
#define SENSOR_STREAM_OFF_ID    0xC002
#define SENSOR_VOLTAGE_ID       0xC003
#define SENSOR_FCNT_ID          0xC004
#define SENSOR_TEMPER_ID        0xC005
#define SENSOR_ROW_COLUMN_ID    0xC006
#define SENSOR_PLL_CLOCK_ID     0xC007
#define SENSOR_I2C_CRC_ID       0xC008
#define SENSOR_SCCB_ID          0xC009
#define SENSOR_RAM_CRC_ID       0xC00A
#define SENSOR_ROM_CRC_ID       0xC00B
#define SENSOR_ONLINE_PIXEL_ID  0xC00C
#define SENSOR_TEST_PATTERN_ID  0xC00D
#define SENSOR_ERRB_ID			0xC00E

typedef union debounce_count {
	uint64_t value;
	struct {
		uint64_t stream_cnt:2;
		uint64_t voltage_cnt:2;
		uint64_t fps_cnt:2;
		uint64_t temp_cnt:2;
		uint64_t row_column_id_cnt:2;
		uint64_t pll_clock_cnt:2;
		uint64_t i2c_crc_cnt:2;
		uint64_t sccb_cnt:2;
		uint64_t ram_crc_cnt:2;
		uint64_t rom_crc_cnt:2;
		uint64_t online_pixel_cnt:2;
		uint64_t test_pattern_cnt:2;
		uint64_t reserved1:40;
	};
} debounce_count_u;

typedef struct time_tv_s {
	struct timespec start;
	struct timespec end;
} time_tv_t;

typedef struct camera_diag_sensor_info_s {
  uint32_t port;
  uint32_t bus_num;
  uint32_t sensor_addr;
  uint32_t errb_gpio;
  uint32_t fps;
  int32_t gpio_fd;
  uint32_t gpio_status;
  char *sensor_name;
  bool pin_change_delay_done;
  time_tv_t time_tv;
  diag_mask_u diag_mask;
  fcnt_check_t fcnt_check;
  sensor_status_u sensor_status;
  debounce_count_u debounce_cnt;
} camera_diag_sensor_info_t;

int32_t camera_sensor_diag_init(board_info_t *board_info);
void camera_sensor_diag_deinit(void);
int32_t sensor_errb_fault_report(camera_diag_sensor_info_t *sen_if,
		bool new_state, uint32_t fault_mask, uint16_t module_id);

#endif  // UTILITY_CAMERA_DIAG_INC_CAM_SENSOR_DIAG_H_
