/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2022 Horizon Robotics.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[ser_diag][%s]:[%s:%d]" fmt, __FILE__, __func__, __LINE__

#include <sys/epoll.h>
#include <sys/poll.h>
#include "../inc/deserial_diag.h"
#include "../../hb_cam_gpio.h"
#include "../../hb_cam_utility.h"

#define MAX_BUF 128
#define GPIO_LEVEL (0)
#define MAX96712_DEC_C (0x37)
#define MAX96712_DEC_D (0x38)
#define MAX96712_DEC_A (0x35)
#define MAX96712_DEC_B (0x36)
#define MAX9296_DEC_A (0x22)

#define MAX96712_IDLE_C (0x3b)
#define MAX96712_IDLE_D (0x3c)
#define MAX96712_IDLE_A (0x39)
#define MAX96712_IDLE_B (0x3a)
#define MAX9296_IDLE_A (0x24)

static uint8_t fail_mark = 0;

static register_config_t max96712_diag_cfg[] = {
	{0x0005, 0xc0},  // ERRB_EN(bit6, default 1)
	{0x0029, 0x08},  // INTR6 LCRC_ERR_OEN(bit3), FSYNC_ERR_OEN bit(0)
	{0x0025, 0x0f},  // INTR2 DEC_ERR_FLAG_A,B,C,D
	{0x002b, 0x0f},  // INTR8 IDLE_ERR_FLAG_A,B,C,D
	{0x002d, 0xff},  // INTR10 RT_CNT_FLAG_A,B,C,D  MAX_RT_FLAG_A,B,C,D
	{0x0044, 0x80},  // MEM_ECC_ERR2_INT
	{0x0506, 0x73},  // enadble linkA CC ARQ, MAX_RT_ERR_OEN_C(b1), RT_CNT_OEN_C(b0)
	{0x0516, 0x73},  // enadble linkB CC ARQ, MAX_RT_ERR_OEN_C(b1), RT_CNT_OEN_C(b0)
	{0x0526, 0x73},  // enadble linkC CC ARQ, MAX_RT_ERR_OEN_C(b1), RT_CNT_OEN_C(b0)
	{0x0536, 0x73},  // enaable linkD CC ARQ, MAX_RT_ERR_OEN_C(b1), RT_CNT_OEN_C(b0)
};

static register_config_t max96718_diag_cfg[] = {
	{0x0005, 0xc0},  // ERRB_EN(bit7, bit6, default 1)
	{0x001a, 0x03},  // DEC_ERR_B,A(bit1,0)
	{0x3010, 0xc0},  // I2C_UART_MSGCNTR_ERR_OEN(bit7), I2C_UART_CRC_ERR_OEN(bit6)
};

static int64_t get_cost_time_ms(const struct timespec *start, const struct timespec *end)
{
	int64_t time_ms;
	time_ms = (int64_t)(((end->tv_sec * CALC_UNIT_TIME) + (end->tv_nsec /CALC_UNIT_TIME/CALC_UNIT_TIME)) -
		((start->tv_sec * CALC_UNIT_TIME) + (start->tv_nsec /CALC_UNIT_TIME/CALC_UNIT_TIME)));
	return time_ms;
}

bool is_unlock(deserial_info_t *deserial_info, uint16_t reg_addr)
{
	int32_t i = 0;
	int32_t val = 0;
	uint8_t lock_stat = 0;
	for(i = 0; i < 2; i++) {
		val = read_reg(deserial_info, reg_addr);
		lock_stat = !!(val & BIT(3));
		if (lock_stat == 1) {  // 1: locked, 0: unlock
			break;
		}
		usleep(5000);  // 5ms
	}
	return (i == 2 && lock_stat == 0) ? true : false;
}

int32_t is_board_type(int board_id) {
	int32_t ret = 0;

	if (board_id < MATRIXMAX_ID && board_id > MATRIXMIN_ID) {
		if (board_id < SOLO_MAX_ID && board_id > SOLO_MIN_ID) {
			ret = SOLO;
		} else if (board_id < DUO_MAX_ID && board_id > DUO_MIN_ID) {
			if (board_id % 2 == 0) {
				ret = DUO_B;
			} else {
				ret = DUO_A;
			}
		}
	}
	return ret;
}

int32_t read_reg(deserial_info_t *deserial_info, uint16_t reg_addr)
{
	int32_t i;
	int32_t ret = 0;
	int32_t retry_times = 1;

	for (i = 0; i < retry_times; i++) {
		ret = hb_vin_i2c_read_reg16_data8(
				deserial_info->bus_num,
				deserial_info->deserial_addr,
				reg_addr);
		if (ret >= 0) {
			break;
		} else {
			vin_info("read %s reg_addr 0x%x failed\n",
					deserial_info->deserial_name, reg_addr);
			usleep(2000);  // 2ms
		}
	}

	if (i == retry_times) {
		vin_err("read %s reg_addr 0x%x failed %d times\n",
					deserial_info->deserial_name, reg_addr, i);
		ret = -RET_ERROR;
	}
	return ret;
}

int32_t write_reg(deserial_info_t *deserial_info, uint16_t reg_addr, uint8_t reg_val)
{
	int32_t i;
	int32_t ret = 0;
	int32_t retry_times = 3;

	for (i = 0; i < retry_times; i++) {
		ret = hb_vin_i2c_write_reg16_data8(
				deserial_info->bus_num,
				deserial_info->deserial_addr,
				reg_addr,
				reg_val);
		if (ret == 0) {
			break;
		} else if (ret < 0) {
			vin_info("write %s reg_addr 0x%x failed\n",
					deserial_info->deserial_name, reg_addr);
			usleep(2000);  // 2ms
		}
	}

	if (i == retry_times) {
		vin_err("write %s reg_addr 0x%x failed %d times\n",
					deserial_info->deserial_name, reg_addr, i);
		ret = -RET_ERROR;
	}
	return ret;
}

void reg_register_ops(deserial_info_t *des_info, register_general_fun_t * reg_ops, uint32_t num)
{
	des_info->reg_ops = reg_ops;
	des_info->reg_ops_num = num;
}

void reg_register_poll_ops(deserial_info_t *des_info, register_general_fun_t * reg_ops, uint32_t num)
{
	des_info->reg_poll_ops = reg_ops;
	des_info->reg_poll_ops_num = num;
}

int32_t deserial_reg_init(deserial_info_t *deserial_info,
							register_config_t reg_cfg[], int array_num)
{
	int32_t i;
	int32_t ret = 0;
	int count = array_num;

	if (count == 0) {
		return 0;
	}

	for (i = 0; i < count; i++) {
		ret = write_reg(deserial_info,
					reg_cfg[i].reg, reg_cfg[i].val);
		if (ret < 0)
			break;
	}

	return ret;
}

uint8_t fault_judging(register_bit_info_t *arg, uint8_t cur_target)
{
	uint8_t ret = NONE_RESULT;
	uint8_t default_fail_val = 1;
	fault_statistics_t *p_better, *p_fault;
	int64_t better_time, fault_time, diff_time;

	if (arg == NULL) {
		return CFG_INCORRECT;
	}
	p_better = &(arg->better_bit);
	p_fault = &(arg->fault_bit);
	better_time = arg->fault_clear_time;
	fault_time = arg->fault_confirm_time;
	if (better_time >= (ERRB_LOOP_TIME * ERRB_LOOP_CNT) /CALC_UNIT_TIME
		|| fault_time >= (ERRB_LOOP_TIME * ERRB_LOOP_CNT) /CALC_UNIT_TIME) {
		return CFG_INCORRECT;
	}

	if (arg->target_fail_val == 0)
		default_fail_val = arg->target_fail_val;  // set special fail val

	if (cur_target == !default_fail_val) {
		if (p_fault->cnt != 0)
			p_fault->cnt = 0;
		if (arg->fault == 0) {
			ret = NONE_RESULT;
			return ret;
		}
		if (p_better->cnt == 0) {
			clock_gettime(CLOCK_MONOTONIC, &(p_better->start));
		}
		p_better->cnt++;
		if (p_better->cnt >= better_time/(ERRB_LOOP_TIME/CALC_UNIT_TIME)) {
			clock_gettime(CLOCK_MONOTONIC, &(p_better->end));
			diff_time = get_cost_time_ms(&(p_better->start), &(p_better->end));
			if (diff_time > better_time) {
				// better
				arg->fault = 0;
				ret = FAULT_TO_BET;
				vin_info("FAULT_TO_BET time: %ldms, cnt:%d\n", diff_time, p_better->cnt);
				p_better->cnt = 0;
			}
		}
	} else if (cur_target == default_fail_val) {
		if (p_better->cnt != 0)
			p_better->cnt = 0;
		if (arg->fault == 1) {
			ret = NONE_RESULT;
			return ret;
		}
		if (p_fault->cnt == 0) {
			clock_gettime(CLOCK_MONOTONIC, &(p_fault->start));
		}
		p_fault->cnt++;
		p_fault->value = cur_target;
		if (p_fault->cnt >= fault_time/(ERRB_LOOP_TIME/CALC_UNIT_TIME)) {
			clock_gettime(CLOCK_MONOTONIC, &(p_fault->end));
			diff_time = get_cost_time_ms(&(p_fault->start), &(p_fault->end));
			if (diff_time > fault_time) {
				// bad
				arg->fault = 1;
				ret = BET_TO_FAULT;
				vin_info("BET_TO_FAULT time: %ldms, cnt:%d \n", diff_time, p_fault->cnt);
				p_fault->cnt = 0;
			}
		}
	}

	return ret;
}

static int32_t open_gpio(int gpio_id, mon_pin_info_t *mon_pin)
{
    char file_path[MAX_BUF];
	int ret = 0;

	int m_fd;

	if(gpio_id < 0 || !mon_pin) {
		ret = -RET_ERROR;
		goto err;
	}
	snprintf(file_path, sizeof(file_path), "/sys/class/gpio/gpio%d", gpio_id);
	if (access(file_path, F_OK) < 0) {
		ret = vin_gpio_export(gpio_id);
		if (ret < 0) {
			vin_err("vin gpio export fail\n");
			ret = -RET_ERROR;
			goto err;
		}
	}

	ret = vin_gpio_set_dir(gpio_id, 0);  // 0:gpio in
	if (ret < 0) {
		vin_err("vin gpio set dir fail\n");
		ret = -RET_ERROR;
		goto err_free;
	}

	ret = vin_gpio_set_edge(gpio_id, "both");
	if (ret < 0) {
		vin_err("vin gpio set mode fail\n");
		ret = -RET_ERROR;
		goto err_free;
	}

	snprintf(file_path, sizeof(file_path), "/sys/class/gpio/gpio%d/value", gpio_id);

	m_fd = open(file_path, O_RDONLY | O_NONBLOCK);
	if (m_fd < 0) {
		vin_err("open_file (%s) failed\n", file_path);
		ret = -RET_ERROR;
		goto err_free;
	} else {
		mon_pin->fd = m_fd;
	}

	return ret;

err_free:
	vin_gpio_unexport(gpio_id);
err:
	return ret;
}


static void close_gpio(mon_pin_info_t *mon_pin)
{
	if (mon_pin && mon_pin->fd) {
		close(mon_pin->fd);
	}
}

static bool errb_is_low(camera_component_diagnose_t *ccd, uint8_t dex_index, uint8_t gpio_index)
{
	bool ret = false;
	int gpio_value;

	char buffer[1] = {0};
	if (lseek(ccd->mon_pin[dex_index][gpio_index].fd, 0, SEEK_SET) < 0)
		vin_err("lseek fail\n");
	if (read(ccd->mon_pin[dex_index][gpio_index].fd, buffer, sizeof(buffer)) < 0) {
		vin_err("read err:%d\n", gpio_index);
	}
	gpio_value = atoi(buffer);


	ret = (gpio_value == GPIO_LEVEL) ? true : false;
	return ret;
}

shm_msg* shm_alloc()
{
	shm_msg *shm = NULL;
	shm = (shm_msg *) malloc(sizeof(shm_msg));
	if (shm != NULL) {
		memset(shm, 0, sizeof(*shm));
	}
	return shm;
}

void shm_free(void *shm)
{
	if (shm) {
		free(shm);
		shm = NULL;
	}
}

static void gpio_value_debounce(camera_component_diagnose_t *ccd, uint8_t des_index, uint8_t pin_index)
{
	static int cur_gpio = 0;
	for (int cnt = 0; cnt < 2; cnt++) {
		if (errb_is_low(ccd, des_index, pin_index)) {
			cur_gpio = (cur_gpio <= 0) ? 1 : 2;
		} else {
			cur_gpio = (cur_gpio >= 0) ? -1 : -2;
		}
		if (-2 == cur_gpio || 2 == cur_gpio) {  // 2 low,   -2,high
			if (cur_gpio == 2) {
				ccd->mon_pin[des_index][pin_index].value = 0;
			} else {
				ccd->mon_pin[des_index][pin_index].value = 1;
			}
			vin_info("gpio%d status is %d\n",
						(ccd->mon_pin[des_index][pin_index]).pin_num,
							(ccd->mon_pin[des_index][pin_index]).value);
			cur_gpio = 0;
		} else {
			usleep(3000);  // 3ms
		}
	}
}

static void gpio_message_send(deserial_info_t *deserial_info, int32_t value)
{
	shm_msg *shm_msg_gpio = NULL;
	shm_msg_gpio = shm_alloc();
	if (shm_msg_gpio != NULL) {
		shm_msg_gpio->data_id = GPIO_EVENT;
		shm_msg_gpio->status = value;
		vin_queue_enq(deserial_info->q_msg, shm_msg_gpio);
		hb_set_status(deserial_info, TH_STATUS_GPIO);
		vin_info("i2c%d %s trigger gpio event \n",
					deserial_info->bus_num,
						deserial_info->deserial_name);
	}
}

void *errb_lock_pin_thread(void *data)
{
	int nfds = 0;
	int index, des_index, pin_index;
	deserial_info_t *deserial_info;
	board_info_t *board_info = (board_info_t *)data;
	camera_component_diagnose_t *ccd = &(board_info->ccd);
	struct epoll_event ev_arry[MON_PIN_NUM];
	int des_num = board_info->deserial_num;
	static int last_gpio[SERDES_NUMBER][PIN_COUNT] = {{1, 1}, {1, 1}, {1, 1}, {1, 1}};

	vin_info("errb linklock monitor thread starting!\n");
	while (ccd->mon_thread_status == THREAD_RUN) {
		nfds = epoll_wait(ccd->epfd, ev_arry, MON_PIN_NUM, 50);
		if (nfds < 0) {
			vin_err("epoll_wait faild\n");
			return NULL;
		} else if (nfds == 0) {
			// vin_info("50ms time out\n");
		}

		for(index = 0; index < nfds; index++) {
			if (POLLPRI == (POLLPRI & ev_arry[index].events)) {
				for (des_index = 0; des_index < des_num; des_index++) {
					deserial_info = &(board_info->deserial_info[des_index]);
					for (pin_index = 0; pin_index < deserial_info->ccd_pin_num; pin_index++) {
						if (ev_arry[index].data.fd == ccd->mon_pin[des_index][pin_index].fd) {
							vin_info("gpio%d is ready\n", ccd->mon_pin[des_index][pin_index].pin_num);
							gpio_value_debounce(ccd, des_index, pin_index);
						}
					}
				}
			}
		}

		for (des_index = 0; des_index < des_num; des_index++) {
			deserial_info = &(board_info->deserial_info[des_index]);
			for (pin_index= 0; pin_index < deserial_info->ccd_pin_num; pin_index++) {
				if (ccd->mon_pin[des_index][pin_index].value != last_gpio[des_index][pin_index]) {
					gpio_message_send(deserial_info, ccd->mon_pin[des_index][pin_index].value);
					last_gpio[des_index][pin_index] = ccd->mon_pin[des_index][pin_index].value;
				}
			}
		}
	}
	ccd->mon_thread_status = THREAD_EXIT;
	vin_info("errb linklock monitor thread exit\n");

	return NULL;
}

static int32_t errb_gpio_init(board_info_t *board_info)
{
	int32_t ret = 0;
	int epfd_m;
	uint32_t des_num = 0;
	struct epoll_event ev;
	camera_component_diagnose_t *ccd;
	deserial_info_t *des_info = NULL;

	ccd = &(board_info->ccd);
	des_num = board_info->deserial_num;

	epfd_m = epoll_create(1);
	if (epfd_m < 0) {
		vin_err("create epoll failed %s\n", strerror(ret));
		return -RET_ERROR;
	} else {
		ccd->epfd = epfd_m;
	}

	for (int32_t i = 0; i < des_num; i++) {
		des_info = &(board_info->deserial_info[i]);
		for (int32_t j = 0; j < des_info->ccd_pin_num; j++) {
			ret = open_gpio(des_info->pin[j].pin_num, &(ccd->mon_pin[i][j]));
			if (ret < 0) {
				vin_err("gpio%d open failed \n", des_info->pin[j].pin_num);
			} else {
				ccd->mon_pin[i][j].pin_num = des_info->pin[j].pin_num;
				bzero(&ev, sizeof(struct epoll_event));
				ev.events = EPOLLPRI;
				ev.data.fd = (ccd->mon_pin[i][j]).fd;
				ret = epoll_ctl(ccd->epfd, EPOLL_CTL_ADD, (ccd->mon_pin[i][j]).fd, &ev);
				if (ret < 0) {
					vin_err("epoll_ctl() failed\n");
					return -RET_ERROR;
				}
			}
		}
	}
	return 0;
}

void errb_gpio_deinit(board_info_t *board_info)
{
	int ret = 0;
	int32_t i = 0;
	uint32_t des_num = 0;
	struct epoll_event ev;
	camera_component_diagnose_t *ccd;
	deserial_info_t *des_info = NULL;

	ccd = &(board_info->ccd);
	des_num = board_info->deserial_num;

	for (i = 0; i < des_num; i++) {
		des_info = &(board_info->deserial_info[i]);
		for (int32_t j = 0; j < des_info->ccd_pin_num; j++) {
			bzero(&ev, sizeof(struct epoll_event));
			ev.events = POLLPRI;
			ev.data.fd = (ccd->mon_pin[i][j]).fd;
			if (ccd->epfd) {
				ret = epoll_ctl(ccd->epfd, EPOLL_CTL_DEL, (ccd->mon_pin[i][j]).fd, &ev);
				if (ret < 0) {
					vin_err("epoll_ctl() failed\n");
				}
			}
			close_gpio(&(ccd->mon_pin[i][j]));
		}
	}
	if (ccd->epfd) {
		close(ccd->epfd);
		ccd->epfd = 0;
	}
}

static int32_t errb_max96712_reg_init(deserial_info_t *des_info)
{
	int32_t ret = 0;
	uint32_t count = 0;
	switch (is_board_type(des_info->board_id)) {
	case SOLO:
		if ((des_info->bus_num == I2C_BUS_2) && (des_info->deserial_addr != 0xff)) {
			count = sizeof(max96712_diag_cfg) / sizeof(max96712_diag_cfg[0]);
			ret = deserial_reg_init(des_info, max96712_diag_cfg, count);
		}
		break;
	case DUO_A:
		if ((des_info->bus_num == I2C_BUS_2) && (des_info->deserial_addr != 0xff)) {
			count = sizeof(max96712_diag_cfg) / sizeof(max96712_diag_cfg[0]);
			ret = deserial_reg_init(des_info, max96712_diag_cfg, count);
		}
		break;
	case DUO_B:
		if ((des_info->bus_num == I2C_BUS_3) && (des_info->deserial_addr != 0xff)) {
			count = sizeof(max96712_diag_cfg) / sizeof(max96712_diag_cfg[0]);
			ret = deserial_reg_init(des_info, max96712_diag_cfg, count);
		}
		break;
	default:
		break;
	}
	return ret;
}

static int32_t errb_max96718_reg_init(deserial_info_t *des_info)
{
	int32_t ret = 0;
	uint32_t count = 0;
	switch (is_board_type(des_info->board_id)) {
	case SOLO:
		if ((des_info->bus_num == I2C_BUS_0) && (des_info->deserial_addr != 0xff)) {
			count = sizeof(max96718_diag_cfg) / sizeof(max96718_diag_cfg[0]);
			ret = deserial_reg_init(des_info, max96718_diag_cfg, count);
		} else if ((des_info->bus_num == I2C_BUS_5) && (des_info->deserial_addr != 0xff)) {
			count = sizeof(max96718_diag_cfg) / sizeof(max96718_diag_cfg[0]);
			ret = deserial_reg_init(des_info, max96718_diag_cfg, count);
		}
		break;
	case DUO_A:
		if ((des_info->bus_num == I2C_BUS_0) && (des_info->deserial_addr != 0xff)) {
			count = sizeof(max96718_diag_cfg) / sizeof(max96718_diag_cfg[0]);
			ret = deserial_reg_init(des_info, max96718_diag_cfg, count);
		}
		break;
	case DUO_B:
		if ((des_info->bus_num == I2C_BUS_5) && (des_info->deserial_addr != 0xff)) {
			count = sizeof(max96718_diag_cfg) / sizeof(max96718_diag_cfg[0]);
			ret = deserial_reg_init(des_info, max96718_diag_cfg, count);
		}
		break;
	default:
		break;
	}
	return ret;
}

static int32_t errb_feature_reg_init(board_info_t *board_info)
{
	int32_t ret = 0;
	deserial_info_t *des_info = NULL;
	uint32_t des_num = board_info->deserial_num;

	for (int32_t i = 0; i < des_num; i++) {
		des_info = &(board_info->deserial_info[i]);
		if (!des_info || des_info->ccd_pin_num == 0)
			continue;
		vin_info("%s, i2c%d, addr:0x%02x\n",
					des_info->deserial_name, des_info->bus_num, des_info->deserial_addr);
		if (strcmp(des_info->deserial_name, "max96712") == 0) {
			ret = errb_max96712_reg_init(des_info);
		} else if (strcmp(des_info->deserial_name, "max96718") == 0) {
			ret = errb_max96718_reg_init(des_info);
		}
		if (ret < 0) {
			vin_err(" poll register init failed\n");
			break;
		}
	}
	return ret;
}


static int32_t errb_resource_init(board_info_t *board_info)
{
	int32_t ret = 0;
	uint32_t pin_cnt = 0;
	uint32_t des_num = 0;
	deserial_info_t *des_info = NULL;
	camera_component_diagnose_t *ccd = NULL;
	des_num = board_info->deserial_num;
	ccd = &(board_info->ccd);

	ccd->mon_thread_status = THREAD_NONE;
	for (int32_t i = 0; i < des_num; i++) {
		des_info = &(board_info->deserial_info[i]);
		if(des_info && des_info->ccd_pin_num) {
			pin_cnt++;
			MUTEX_INIT(des_info->mutex_status);
			MUTEX_INIT(des_info->listen_mutex);
			COND_INIT(des_info->cond_status);
			des_info->errb_thread_status = TH_STATUS_NONE;
			des_info->q_msg = vin_queue_create();
			if (des_info->q_msg == NULL) {
				vin_err("queue create failed\n");
				return -RET_ERROR;
			}
			if(!strcmp(des_info->deserial_name, "max96712")) {
				max96712_reg_register_ops(des_info);
			} else if (!strcmp(des_info->deserial_name, "max96718")) {
				max96718_reg_register_ops(des_info);
			}
		}
	}

	if(pin_cnt != 0) {
		ret = pthread_create(&ccd->mon_pid, NULL,
					errb_lock_pin_thread, (void *)board_info);
		if (ret == 0) {
			ccd->mon_thread_status = THREAD_RUN;
		} else {
			ccd->mon_thread_status = THREAD_NONE;
			vin_err("create thread failed %s\n", strerror(ret));
			return ret;
		}
	}

	for (int32_t i = 0; i < des_num; i++) {
		des_info = &(board_info->deserial_info[i]);
		if (des_info && des_info->ccd_pin_num) {
			ret = errb_loop_thread_create(des_info);
			if (ret < 0) {
				return ret;
			}
		}
	}
	return ret;
}


static void errb_resource_deinit(board_info_t *board_info)
{
	int32_t i = 0;
	uint32_t pin_cnt = 0;
	uint32_t des_num = 0;
	deserial_info_t *des_info = NULL;
	camera_component_diagnose_t *ccd = NULL;
	des_num = board_info->deserial_num;
	ccd = &(board_info->ccd);

	if (ccd->mon_thread_status == THREAD_RUN) {
		ccd->mon_thread_status = THREAD_STOP;
		while (ccd->mon_thread_status >= THREAD_STOP
				&& ccd->mon_thread_status <= THREAD_RUN) {
			i++;
			usleep(6000);  // 6ms
			if (i == 10) {
				pthread_cancel(ccd->mon_pid);
				vin_err("cancel mon pin thread,status:%d\n",
						 ccd->mon_thread_status);
				break;
			}
		}
	}
	if (ccd->mon_thread_status > THREAD_NONE)
		pthread_join(ccd->mon_pid, NULL);

	for (i = 0; i < des_num; i++) {
		des_info = &(board_info->deserial_info[i]);
		if(des_info && des_info->deserial_name) {
			if (des_info->errb_thread_status > TH_STATUS_NONE) {
				hb_set_status(des_info,
							TH_STATUS_ACTIVE |
							TH_STATUS_PAUSE |
							TH_STATUS_STOP |
							TH_STATUS_GPIO);
				vin_info("set i2c%d %s stop \n",
						board_info->deserial_info[i].bus_num,
						board_info->deserial_info[i].deserial_name);
			}
		}
	}

	for (i = 0; i < des_num; i++) {
		des_info = &(board_info->deserial_info[i]);
		if (des_info->ccd_pin_num != 0) {
			errb_loop_thread_deinit(des_info);
		}
		if(!strcmp(des_info->deserial_name, "max96712")) {
			max96712_reg_unregister_ops(des_info);
		} else if (!strcmp(des_info->deserial_name, "max96718")) {
			max96718_reg_unregister_ops(des_info);
		}
		hb_unregister_serdes_listen_callback(des_info);
		MUTEX_DESTROY(des_info->mutex_status);
		MUTEX_DESTROY(des_info->listen_mutex);
		COND_DESTROY(des_info->cond_status);
		vin_queue_destroy(des_info->q_msg, shm_free);
	}
}

int32_t deserial_errb_init(board_info_t *board_info)
{
	int32_t ret = 0;
	uint32_t des_num = 0;
	deserial_info_t *deserial_info = NULL;

	if (!board_info)
		return -RET_ERROR;

	des_num = board_info->deserial_num;
	if (des_num == 0) {
		vin_info("des_num:%d\n", des_num);
		ret = -RET_ERROR;
		goto fail_0;
	}
	vin_info("deserial errb init\n");
	ret = errb_feature_reg_init(board_info);
	if (ret < 0) {
		vin_err("errb reg init failed\n");
		goto fail_0;
	}

	ret = errb_gpio_init(board_info);
	if (ret < 0) {
		vin_err("errb gpio init failed\n");
		goto fail_1;
	}

	ret = errb_resource_init(board_info);
	if (ret < 0) {
		vin_err("errb resource init failed\n");
		goto fail_2;
	}

	vin_info("deserial errb init end\n");
	return ret;
fail_2:
	errb_resource_deinit(board_info);
fail_1:
	errb_gpio_deinit(board_info);
fail_0:
	fail_mark = 1;
	return ret;
}

int32_t deserial_errb_deinit(board_info_t *board_info)
{
	int i = 0;
	int ret = 0;
	uint32_t des_num = 0;
	deserial_info_t *deserial_info = NULL;

	if (!board_info)
		return -RET_ERROR;

	des_num = board_info->deserial_num;
	if (des_num == 0) {
		vin_info("des_num:%d\n", des_num);
		return ret;
	}

	if (!fail_mark) {
		errb_resource_deinit(board_info);
		errb_gpio_deinit(board_info);
	}

	vin_info("deserial errb deinit end\n");
	return ret;
}
