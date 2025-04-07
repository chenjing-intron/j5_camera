/***************************************************************************
 * COPYRIGHT NOTICE
 * Copyright 2022 Horizon Robotics.
 * All rights reserved.
 ***************************************************************************/

#include <getopt.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/prctl.h>
#include <sys/time.h>
#include <time.h>
#include <poll.h>
#include <fcntl.h>
#include "../../hb_i2c.h"

#include "../inc/cam_sensor_diag.h"

pthread_t cam_diag_ov_i2c_tid;
pthread_t cam_diag_sony_i2c_tid;
pthread_t cam_diag_ov_errb_tid;
pthread_t cam_diag_sony_errb_tid;

static int32_t ov_sensor_num = 0;
static int32_t sony_sensor_num = 0;
static int32_t ov_errb_num = 0;
static int32_t sony_errb_num = 0;

static bool sensor_i2c_diag_runing = false;
static bool sensor_errb_diag_runing = false;

static camera_diag_sensor_info_t ov_sensor_info[CAM_MAX_NUM] = {0};
static camera_diag_sensor_info_t sony_sensor_info[CAM_MAX_NUM] = {0};

static uint32_t monitor_period_us = 0;

static bool is_interrupt_gpio = true;
#if 0  // SENSOR_DIAG_TEST
struct timespec time_start;
struct timespec time_end;
#endif
static int32_t time_cost_us(struct timespec *start, struct timespec *end)
{
  int time_us = -1;
  time_us = ((end->tv_sec * 1000000 + end->tv_nsec / 1000) -
             (start->tv_sec * 1000000 + start->tv_nsec / 1000));
  return time_us;
}

int32_t sensor_errb_fault_report(camera_diag_sensor_info_t *sen_if,
              bool new_state, uint32_t fault_mask, uint16_t module_id)
{
  int32_t ret = RET_OK;
  bool last_state = false;
  uint8_t debounce_cnt = 0;
  if (sen_if == NULL) {
    vin_err("%d sensor_info is NULL \n", __LINE__);
    return -RET_ERROR;
  }
  last_state = sen_if->sensor_status.value & BIT(fault_mask);
  debounce_cnt = (sen_if->debounce_cnt.value & (3 << 2 * fault_mask)) >> 2 * fault_mask;
  if (new_state != last_state) {
    debounce_cnt++;
    if (debounce_cnt >= DEBOUNCE_CNT) {
      debounce_cnt = 0;
      vin_info("Detect port_%d %s fault type %02d fault state = %d\n",
          sen_if->port, sen_if->sensor_name, fault_mask, new_state);
      sen_if->sensor_status.value = (sen_if->sensor_status.value &
          (~(1 << fault_mask))) | (new_state << fault_mask);
      ret = hb_cam_diagnose(module_id, sen_if->port + 1, new_state);
      if (ret < 0) {
        vin_err("errb type %02d state report fail\n", fault_mask);
        return ret;
      }
    }
  } else {
    debounce_cnt = 0;
  }
  sen_if->debounce_cnt.value &= ~(3 << 2 * fault_mask);
  sen_if->debounce_cnt.value |= (debounce_cnt & 0x3) << 2 * fault_mask;

  return 0;
}

static int32_t open_gpio_file(int32_t gpio_id)
{
  char file_path[MAX_BUF];
  int32_t ret = 0, fd = 0;

  if (gpio_id < 0)
    return -RET_ERROR;

  (void)snprintf(file_path, sizeof(file_path), SYSFS_GPIO_DIR "/gpio%d", gpio_id);
  if (access(file_path, 0) < 0) {
		ret = vin_gpio_export(gpio_id);
		if (ret < 0) {
			vin_err("vin gpio export fail\n");
			goto err;
		}
	}

  ret = vin_gpio_set_dir(gpio_id, 0);
  if (ret < 0) {
    vin_err("vin gpio set dir fail\n");
    goto err;
  }

  if (is_interrupt_gpio) {
    ret = vin_gpio_set_edge(gpio_id, "both");
    if (ret < 0) {
      vin_err("vin gpio set edge fail\n");
      goto err;
    }
  }

  (void)snprintf(file_path, sizeof(file_path), SYSFS_GPIO_DIR "/gpio%d/value", gpio_id);
  fd = open(file_path, O_RDONLY);
  if (fd < 0) {
    vin_err("open_gpio_file (%s) failed, exit return err !!\n", file_path);
    goto err;
  }

  return fd;

err:
  ret = vin_gpio_unexport(gpio_id);
  if (ret < 0) {
    vin_err("vin gpio unexport fail\n");
  }
  return -RET_ERROR;
}

static void close_gpio_file(int32_t gpio_intr_fd, uint32_t gpio_id)
{
  int32_t ret = 0;
  char file_path[MAX_BUF];

  vin_info("close gpio file gpio_fd = %d, gpio_id = %d\n", gpio_intr_fd, gpio_id);
  if (gpio_intr_fd > 0) {
    close(gpio_intr_fd);
  }

  (void)snprintf(file_path, sizeof(file_path), SYSFS_GPIO_DIR "/gpio%d", gpio_id);
  if (access(file_path, 0) == 0) {
		ret = vin_gpio_unexport(gpio_id);
		if (ret < 0) {
			vin_err("vin gpio unexport fail\n");
		}
	}
}

static void *sensor_i2c_diag_thread(void *data)
{
  int32_t ret = RET_OK;
  int32_t index = 0;
  int32_t sensor_num = 0;
  int32_t sensor_type = 0;
  char tname[16] = {0};
  camera_diag_sensor_info_t *sensor_if = (camera_diag_sensor_info_t *)data;
  if (sensor_if == NULL) {
    vin_err("sensor diag i2c polling thread get sensor info is NULL\n");
    goto err;
  }

  if (!strncmp(sensor_if[0].sensor_name, CAMERA_DIAG_OV_NAME, 3)) {
    snprintf(tname, sizeof(tname), "ov_i2c_diag");
    sensor_num = ov_sensor_num;
    sensor_type = 1;
  } else if (!strncmp(sensor_if[0].sensor_name, CAMERA_DIAG_SONY_NAME, 3)) {
    snprintf(tname, sizeof(tname), "sony_i2c_diag");
    sensor_num = sony_sensor_num;
    sensor_type = 2;
  } else {
    vin_info("i2c diag reserved set ...\n");
    goto err;
  }
  prctl(PR_SET_NAME, tname);
  usleep(10000);

  for (int32_t i = 0; i < sensor_num; i++) {
    vin_info("type: %d index: %d port: %d, name: %s, bus: %d, addr: 0x%x, mask: 0x%02x, errb = %d\n",
        sensor_type, i, sensor_if[i].port, sensor_if[i].sensor_name, sensor_if[i].bus_num,
        sensor_if[i].sensor_addr, sensor_if[i].diag_mask.value, sensor_if[i].errb_gpio);
  }

  while (sensor_i2c_diag_runing) {
    usleep(monitor_period_us);
    // clock_gettime(CLOCK_MONOTONIC, &time_start);
    for (index = 0; index < sensor_num; index++) {
      if (sensor_type == 1) {
        ret = ov_sensor_get_status(sensor_if, index);
      } else if (sensor_type == 2) {
        ret = sony_sensor_get_status(sensor_if, index);
      } else {
        goto err;
      }
      if (ret < 0) {
        vin_err("sensor diag get_status type: %d index: %d err\n", sensor_type, index);
      }
    }
    // clock_gettime(CLOCK_MONOTONIC, &time_end);
    // vin_info("get sensor %d i2c diag period time diff = %d us\n", sensor_type, time_cost_us(&time_start, &time_end));
  }
err:
  vin_info("sensor diag type %d i2c polling thread exit \n", sensor_type);
  pthread_exit(NULL);
}

static int32_t errb_fault_judging(camera_diag_sensor_info_t *sensor_if, int32_t errb_num, int32_t sensor_type)
{
  int32_t ret = RET_OK;

  if (sensor_if == NULL)
    return -RET_ERROR;

  for (int32_t i = 0; i < errb_num; i++) {
    if (!sensor_if[i].pin_change_delay_done) {
      clock_gettime(CLOCK_MONOTONIC, &sensor_if[i].time_tv.end);
      if (time_cost_us(&sensor_if[i].time_tv.start, &sensor_if[i].time_tv.end) > PIN_RECOVERY_DELAY_TIME) {
        sensor_if[i].pin_change_delay_done = true;
      }
    }
    if (sensor_if[i].gpio_status == 0 || !sensor_if[i].pin_change_delay_done) {
      if (sensor_type == 1) {
        ret = ov_errb_fault_handle(sensor_if, i);
      } else if (sensor_type == 2) {
        ret = sony_errb_fault_handle(sensor_if, i);
      } else {
        vin_info("%d errb diag reserved set ...\n", __LINE__);
        return -RET_ERROR;
      }
      if (ret < 0) {
        vin_err("errb sensor_%d get fault info error !\n", sensor_if[i].port);
        return ret;
      }
    }
  }
  return 0;
}

static int32_t errb_status_judging(camera_diag_sensor_info_t *sensor_if, int32_t errb_num, struct pollfd *fds)
{
  int32_t ret = RET_OK;
  int32_t len = 0;
  char buff[1] = {0};
  uint32_t current_status = 0;

  if (sensor_if == NULL || fds == NULL)
    return -RET_ERROR;

  for (int32_t i = 0; i < errb_num; i++) {
    if (is_interrupt_gpio) {
      if (fds[i].revents & POLLPRI) {
        ret = lseek(sensor_if[i].gpio_fd, 0, SEEK_SET);
        if (ret == -1)
          vin_err("lseek fail\n");
        if ((len = read(fds[i].fd, &buff, sizeof(buff))) == -1) {
          int32_t errno_tmp = errno;
          vin_err("hb cam sen errb diag read err:%d\n", errno_tmp);
          return -RET_ERROR;
        }
      } else {
        continue;
      }
      current_status = atoi(buff);
    } else {
      ret = vin_gpio_get_value(sensor_if[i].errb_gpio, &current_status);
      if (ret != RET_OK) {
        vin_err("gpio %d get value failed \n", sensor_if[i].errb_gpio);
        continue;
      }
    }

    if (sensor_if[i].gpio_status != current_status) {
      sensor_if[i].gpio_status = current_status;
      sensor_if[i].pin_change_delay_done = false;
      clock_gettime(CLOCK_MONOTONIC, &sensor_if[i].time_tv.start);
      vin_info("errb diag port[%02d] %s pin status is change new status = %d\n",
        sensor_if[i].port, sensor_if[i].sensor_name, sensor_if[i].gpio_status);
      ret = hb_cam_diagnose(SENSOR_ERRB_ID, sensor_if[i].port + 1, !sensor_if[i].gpio_status);
      if (ret < 0) {
        vin_err("sensor diag report errb state: %d fail\n", sensor_if[i].gpio_status);
        return ret;
      }
    }
  }
  return 0;
}

static int32_t open_all_errb_gpio(camera_diag_sensor_info_t *sensor_if, struct pollfd *fds, int32_t errb_num)
{
  int32_t ret = RET_OK;
  char buff[1] = {0};

  if (sensor_if == NULL)
    return -RET_ERROR;

  for (int32_t i = 0; i < errb_num; i++) {
    if (sensor_if[i].errb_gpio == 0) {
      vin_warn("errb diag port[%02d] errb gpio is 0 !\n", sensor_if[i].port);
      return -RET_ERROR;
    }
    sensor_if[i].gpio_fd = open_gpio_file(sensor_if[i].errb_gpio);
    if (sensor_if[i].gpio_fd < 0) {
      vin_err("errb open gpio %d error, fd: %d !\n", sensor_if[i].errb_gpio, sensor_if[i].gpio_fd);
      return -RET_ERROR;
    }
    fds[i].fd = sensor_if[i].gpio_fd;
    fds[i].events = POLLPRI;
    ret = read(fds[i].fd, buff, 1);
    if (ret > 0) {
      sensor_if[i].gpio_status = atoi(buff);
    }
    sensor_if[i].pin_change_delay_done = false;
    clock_gettime(CLOCK_MONOTONIC, &sensor_if[i].time_tv.start);
    vin_info("errb diag name: %s, port: %d, gpio: %d, fd: %d, status: %d\n", sensor_if[i].sensor_name,
             sensor_if[i].port, sensor_if[i].errb_gpio, fds[i].fd, sensor_if[i].gpio_status);
  }
  return 0;
}

static void *sensor_errb_diag_thread(void *data)
{
  int32_t ret = RET_OK;
  int32_t errb_num = 0;
  int32_t sensor_type = 0;
  char tname[16] = {0};
  struct pollfd fds[CAM_MAX_NUM] = {0};
  camera_diag_sensor_info_t *sensor_if = (camera_diag_sensor_info_t *)data;
  if (sensor_if == NULL) {
    vin_err("sensor diag errb monitor thread get sensor info is NULL\n");
    return NULL;
  }

  if (!strncmp(sensor_if[0].sensor_name, CAMERA_DIAG_OV_NAME, 3)) {
    snprintf(tname, sizeof(tname), "ov_errb_diag");
    errb_num = ov_errb_num;
    sensor_type = 1;
  } else if (!strncmp(sensor_if[0].sensor_name, CAMERA_DIAG_SONY_NAME, 3)) {
    ret = sony_errb_diag_init_setting(sensor_if, sony_errb_num);
    if (ret < 0)
      return NULL;
    snprintf(tname, sizeof(tname), "sony_errb_diag");
    errb_num = sony_errb_num;
    sensor_type = 2;
  } else {
    vin_info("%d errb diag reserved set ...\n", __LINE__);
    return NULL;
  }
  prctl(PR_SET_NAME, tname);

  usleep(500000);
  ret = open_all_errb_gpio(sensor_if, fds, errb_num);
  if (ret < 0)
    goto err;

  while (sensor_errb_diag_runing) {
    if (is_interrupt_gpio) {
      ret = poll(fds, errb_num, ERRB_POLLING_PERIOD_MS);
      if (ret < 0) {
        vin_err("sensor errb pin poll error !\n");
      } else if (ret == 0) {
        ret = errb_fault_judging(sensor_if, errb_num, sensor_type);
        if (ret < 0)
          vin_err("sensor %d errb fault check fail\n", sensor_type);
      } else {
        ret = errb_status_judging(sensor_if, errb_num, fds);
        if (ret < 0)
          vin_err("sensor %d errb status check fail\n", sensor_type);
      }
    } else {
      usleep(ERRB_POLLING_PERIOD_MS * 1000);
      ret = errb_status_judging(sensor_if, errb_num, fds);
      if (ret < 0)
        vin_err("sensor %d errb status check fail\n", sensor_type);
      ret = errb_fault_judging(sensor_if, errb_num, sensor_type);
      if (ret < 0)
        vin_err("sensor %d errb fault check fail\n", sensor_type);
    }
  }
err:
  for (int32_t i = 0; i < errb_num; i++) {
    (void)close_gpio_file(sensor_if[i].gpio_fd, sensor_if[i].errb_gpio);
  }
  vin_info("sensor diag type %d errb daig thread exit \n", sensor_type);
  pthread_exit(NULL);
}

static int32_t camera_diag_sensor_getinfo(sensor_info_t *sensor_info,
                camera_diag_sensor_info_t *diag_sensor_info, int32_t sensor_num)
{
  if (sensor_num >= CAM_MAX_NUM || sensor_info == NULL)
    return -RET_ERROR;

  diag_sensor_info[sensor_num].fps = sensor_info->fps;
  diag_sensor_info[sensor_num].port = sensor_info->dev_port;
  diag_sensor_info[sensor_num].bus_num = sensor_info->bus_num;
  diag_sensor_info[sensor_num].errb_gpio = sensor_info->sensor_errb;
  diag_sensor_info[sensor_num].sensor_addr = sensor_info->sensor_addr;
  diag_sensor_info[sensor_num].sensor_name = sensor_info->sensor_name;
  diag_sensor_info[sensor_num].diag_mask.value = sensor_info->diag_mask.value;

  sensor_num++;

  return sensor_num;
}

static int32_t sensor_i2c_polling_diag_start(board_info_t *board_info)
{
  int32_t ret = RET_OK;

  if (board_info == NULL) {
    vin_err("board_info is null, return err !\n");
    return -RET_ERROR;
  }

  if (ov_sensor_num == 0 && sony_sensor_num == 0)
    return -RET_ERROR;

  monitor_period_us = board_info->monitor_period_ms * 1000;

  if (ov_sensor_num != 0) {
    sensor_i2c_diag_runing = true;
    ret = pthread_create(&cam_diag_ov_i2c_tid, NULL, sensor_i2c_diag_thread, (void *)ov_sensor_info);
    if (0 != ret) {
      vin_err("camera ov sensor diag thread create fail, err = %s\n", strerror(ret));
      return -ret;
    }
  }

  if (sony_sensor_num != 0) {
    sensor_i2c_diag_runing = true;
    ret = pthread_create(&cam_diag_sony_i2c_tid, NULL, sensor_i2c_diag_thread, (void *)sony_sensor_info);
    if (0 != ret) {
      vin_err("camera sony sensor diag thread create fail, err = %s\n", strerror(ret));
      return -ret;
    }
  }
  vin_info("-> now camera sensor diag i2c polling diag thread all start !\n");
  return 0;
}

static int32_t sensor_i2c_polling_diag_stop(void)
{
  if (ov_sensor_num == 0 && sony_sensor_num == 0)
    return 0;

  if (ov_sensor_num != 0) {
    sensor_i2c_diag_runing = false;
    pthread_join(cam_diag_ov_i2c_tid, NULL);
    ov_sensor_num = 0;
  }

  if (sony_sensor_num != 0) {
    sensor_i2c_diag_runing = false;
    pthread_join(cam_diag_sony_i2c_tid, NULL);
    sony_sensor_num = 0;
  }
  monitor_period_us = 0;
  vin_info("-> now camera sensor diag i2c polling diag thread all exit !\n");
  return 0;
}

static int32_t sensor_errb_monitor_diag_start(void)
{
  int32_t ret = RET_OK;
  int32_t board_id = 0;
  if (ov_sensor_num == 0 && sony_sensor_num == 0)
    return -RET_ERROR;

  board_id = vin_get_board_id();
  vin_info("errb board_id = 0x%x\n", board_id);
  if (board_id == BOARD_ID_MATRIXDSOLO ||
      board_id == BOARD_ID_MATRIXDSOLO_V2 ||
      board_id == BOARD_ID_MATRIXSOLO_V3) {
    is_interrupt_gpio = false;
  }

  if (ov_sensor_num != 0) {
    ov_errb_num = ov_sensor_num;
    sensor_errb_diag_runing = true;
    ret = pthread_create(&cam_diag_ov_errb_tid, NULL, sensor_errb_diag_thread, (void *)ov_sensor_info);
    if (0 != ret) {
      vin_err("camera ov sensor diag thread create fail, err = %s\n", strerror(ret));
      return -ret;
    }
  }
  if (sony_sensor_num != 0) {
    sony_errb_num = sony_sensor_num;
    sensor_errb_diag_runing = true;
    ret = pthread_create(&cam_diag_sony_errb_tid, NULL, sensor_errb_diag_thread, (void *)sony_sensor_info);
    if (0 != ret) {
			vin_err("camera sony sensor diag thread create fail, err = %s\n", strerror(ret));
			return -ret;
		}
  }
  vin_info("-> now camera sensor diag errb monitor diag thread all start\n");
  return 0;
}

static int32_t sensor_errb_monitor_diag_stop(void)
{
  if (ov_errb_num == 0 && sony_errb_num == 0)
    return 0;

  if (ov_errb_num != 0) {
    sensor_errb_diag_runing = false;
    pthread_join(cam_diag_ov_errb_tid, NULL);
    for (int32_t i = 0; i < ov_errb_num; i++) {
      ov_sensor_info[i].gpio_fd = -1;
      ov_sensor_info[i].errb_gpio = 0;
      ov_sensor_info[i].gpio_status = 0;
    }
    ov_errb_num = 0;
  }

  if (sony_errb_num != 0) {
    sensor_errb_diag_runing = false;
    pthread_join(cam_diag_sony_errb_tid, NULL);
    for (int32_t i = 0; i < sony_errb_num; i++) {
      sony_sensor_info[i].gpio_fd = -1;
      sony_sensor_info[i].errb_gpio = 0;
      sony_sensor_info[i].gpio_status = 0;
    }
    sony_errb_num = 0;
  }

  vin_info("-> now camera sensor diag errb monitor diag thread all exit\n");
  return 0;
}

int32_t camera_sensor_diag_init(board_info_t *board_info)
{
  int32_t ret = RET_OK;
  uint32_t port = 0;
  sensor_info_t *sensor_if = NULL;

  if (board_info == NULL) {
    vin_err("%d board_info is NULL sensor diag init fail\n", __LINE__);
    return -RET_ERROR;
  }

  for (port = 0; port < board_info->port_number; port++) {
    sensor_if = &board_info->sensor_info[port];

    if (sensor_if == NULL) {
      vin_err("sensor_if %d is NULL sensor diag init fail\n", port);
      return -RET_ERROR;
    }

    if (sensor_if->sensor_name == NULL)
      continue;

    if (!strncmp(sensor_if->sensor_name, CAMERA_DIAG_OV_NAME, 3) && sensor_if->resolution != 1080) {  // skip ov2311
      ov_sensor_num = camera_diag_sensor_getinfo(sensor_if, ov_sensor_info, ov_sensor_num);
      if (ov_sensor_num < 0) {
        vin_err("camera sensor diag get ov sensor info fail\n");
        return -RET_ERROR;
      }
    } else if (!strncmp(sensor_if->sensor_name, CAMERA_DIAG_SONY_NAME, 3)) {
      sony_sensor_num = camera_diag_sensor_getinfo(sensor_if, sony_sensor_info, sony_sensor_num);
      if (sony_sensor_num < 0) {
        vin_err("camera sensor diag get sony sensor info fail\n");
        return -RET_ERROR;
      }
    } else {
      vin_info("camera sensor diag sensor info mismatch \n");
      continue;
    }
  }

  ret = sensor_i2c_polling_diag_start(board_info);
  if (ret < 0) {
    vin_err("caemra sensor diag i2c polling start fail\n");
    return -RET_ERROR;
  }

  ret = sensor_errb_monitor_diag_start();
  if (ret < 0) {
    vin_err("caemra sensor diag errb monitor start fail\n");
    return -RET_ERROR;
  }

  return ret;
}

void camera_sensor_diag_deinit(void)
{
  int32_t ret = RET_OK;

  ret = sensor_i2c_polling_diag_stop();
  if (ret < 0) {
    vin_err("caemra sensor diag i2c polling stop fail\n");
  }
  ret = sensor_errb_monitor_diag_stop();
  if (ret < 0) {
    vin_err("caemra sensor diag errb monitor stop fail\n");
  }
  return;
}
