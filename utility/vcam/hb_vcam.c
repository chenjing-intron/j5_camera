/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#define pr_fmt(fmt)		"[hb_vcam]:[%s][%d]" fmt, __func__, __LINE__
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/mman.h>
#include <sys/un.h>
#include <pthread.h>

#include "hb_vcam.h"
#include "cam_common.h"

#define VCAM_DEV_NAME		"vcam"
#define VCAM_IOC_MAGIC		'v'
#define VCAM_DEV			"/dev/vcam"
#define	VCAM_SUN_PATH		"/userdata/server_socket"
#define	VCAM_RETRY_NEXT_MSG	3
#define VCAM_ALIG_64K(x)	(((x) + 0xFFFF) & ~0xFFFF)

#define HB_VCAM_INIT		_IO(VCAM_IOC_MAGIC, 0)
#define HB_VCAM_DEINIT		_IO(VCAM_IOC_MAGIC, 1)
#define HB_VCAM_START		_IO(VCAM_IOC_MAGIC, 2)
#define HB_VCAM_STOP		_IO(VCAM_IOC_MAGIC, 3)
#define HB_VCAM_NEXT_GROUP	_IO(VCAM_IOC_MAGIC, 4)
#define HB_VCAM_FRAME_DONE	_IO(VCAM_IOC_MAGIC, 5)
#define HB_VCAM_FREE_IMG	_IO(VCAM_IOC_MAGIC, 6)
#define HB_VCAM_GET_IMG		_IO(VCAM_IOC_MAGIC, 7)
#define HB_VCAM_MEMINFO		_IO(VCAM_IOC_MAGIC, 8)
#define HB_VCAM_CLEAN		_IO(VCAM_IOC_MAGIC, 9)

static int32_t g_vcam_fd;	// vcam fd
static int32_t g_sock_fd;	// socket fd
static pthread_t g_vcam_pid;
static int32_t g_mem_fd;	// mem fd
static int32_t g_mem_len;	// the mem size
static uint64_t g_vaddr_base;	// vcam Virtual address
static uint64_t g_paddr_base;	// vcam Physical address

/*
 *vcam_next_msg_thread
 *
 * write next group info for daemon
 * if there is no group can be userd of get fail, will send null to daemon
 *
 */
void *vcam_next_msg_thread(void *data)
{
	int32_t ret = 0;
	hb_vcam_msg_t next_msg;
	/* Threads block and do not consume CPU resources until the wait_event is waken up */
	while(1) {
		ret = ioctl(g_vcam_fd, HB_VCAM_NEXT_GROUP, &next_msg);  // block ioctl
		if(ret < 0) {
			vin_err("get next group fail\n");
			continue;
		}
		next_msg.info_type = CMD_VCAM_NEXT_REQUST;
		ret = write(g_sock_fd, &next_msg, sizeof(hb_vcam_msg_t));
		if(ret != sizeof(hb_vcam_msg_t)) {
			vin_err("hb_vin_vcam_next_group write fail ret %d\n", ret);
			continue;
		}
	}
	pthread_exit(NULL);
}

/*
 * send msg to daemon
 * send cmd type nest frame
 * send next group info
 * send img info
 */
int32_t hb_vin_vcam_next_group()
{
	hb_vcam_msg_t next_group;
	int32_t ret;
	ret = ioctl(g_vcam_fd, HB_VCAM_NEXT_GROUP, &next_group);  // block ioctl
	if(ret < 0) {
		vin_err("get next group fail\n");
		return -1;
	}
	next_group.info_type = CMD_VCAM_NEXT_REQUST;
	ret = write(g_sock_fd, &next_group, sizeof(hb_vcam_msg_t));
	if(ret != sizeof(hb_vcam_msg_t)) {
		vin_err("hb_vin_vcam_next_group write fail ret %d\n", ret);
		return -1;
	}
	return 0;
}

/*
 * to get img info;
 * is block api + timeout;
 *
 */
int32_t hb_vin_vcam_get_img(cam_img_info_t *cam_img_info)
{
	int32_t ret = 0;

	if (cam_img_info == NULL) {
		vin_err("cam img info is null !!\n");
		return -1;
	}
	ret = ioctl(g_vcam_fd, HB_VCAM_GET_IMG, cam_img_info);
	if(ret < 0) {
		vin_err("vcam get img ioctl fail\n");
		return -1;
	}
	cam_img_info->img_addr.y_vaddr = g_vaddr_base + (cam_img_info->img_addr.y_paddr - g_paddr_base);
	cam_img_info->img_addr.c_vaddr = g_vaddr_base + (cam_img_info->img_addr.c_paddr - g_paddr_base);
	return ret;
}

/*
 * to free img info;
 *
 */
int32_t hb_vin_vcam_free_img(cam_img_info_t *cam_img_info)
{
	int32_t ret = 0;

	if (cam_img_info == NULL) {
		vin_err("cam img info is null !!\n");
		return -1;
	}
	ret = ioctl(g_vcam_fd, HB_VCAM_FREE_IMG, cam_img_info);
	if(ret < 0) {
		vin_err("vcam free img ioctl fail\n");
		return -1;
	}
	return ret;
}

/*
 * vcam clean group 
 *
 */
int32_t hb_vin_vcam_clean(cam_img_info_t *cam_img_info)
{
	int32_t ret = 0;

	if (cam_img_info == NULL) {
		vin_err("cam img info is null !!\n");
		return -1;
	}
	ret = ioctl(g_vcam_fd, HB_VCAM_CLEAN, cam_img_info);
	if(ret < 0) {
		vin_err("vcam clean ioctl fail !!\n");
		return -1;
	}
	return ret;
}

/*
 * open vcam dev
 * connect daemon service
 * init vcam config (may be need)
*/
int32_t hb_vin_vcam_init(hb_vcam_msg_t *init_msg)
{
	int32_t ret;
    struct sockaddr_un address;
	struct mem_info_t mem_info;

	vin_info("vcam init\n");
	if (init_msg == NULL) {
		vin_err("init msg is NULL\n");
		return -1;
	}
	if ((g_vcam_fd = open(VCAM_DEV, O_RDWR)) < 0) {
		vin_err("vcam init open vcam fail\n");
		return -1;
	}
	if ((g_mem_fd = open("/dev/mem", O_RDWR)) < 0) {
		vin_err("vcam init open dev mem fail\n");
		return -1;
	}
	ret = ioctl(g_vcam_fd, HB_VCAM_INIT, init_msg);
	if (ret < 0) {
		vin_err("vcam init ioctl fail %d\n", ret);
		return -1;
	}
	if ((g_sock_fd = socket(AF_UNIX, SOCK_STREAM, 0)) == -1) {
		vin_err("create socket fd fail, may be no vcam daemon !!\n");
		return -1;
	}
	address.sun_family = AF_UNIX;
	strcpy(address.sun_path, VCAM_SUN_PATH);
	vin_info("%s\n", address.sun_path);
	/* connect to server */
	ret = connect (g_sock_fd, (struct sockaddr *)&address, sizeof (address));
	if (ret == -1) {
		vin_err("connect [%s] fail\n", address.sun_path);
		return -1;
	}
	/* send to server init cmd */
	init_msg->info_type = CMD_VCAM_INIT;
	ret = write(g_sock_fd, init_msg, sizeof(hb_vcam_msg_t));
	if(ret != sizeof(hb_vcam_msg_t)) {
		vin_err("hb_vin_vcam_init write fail ret %d\n", ret);
		return -1;
	}
	/* create vcam thread */
	ret = pthread_create(&g_vcam_pid, NULL, vcam_next_msg_thread, NULL);
	if (ret) {
		vin_err("vcam thread create fail\n");
		return -1;
	}
	ret = pthread_detach(g_vcam_pid);
	if (ret) {
		vin_err("vcam thread detach fail\n");
		return -1;
	}
	/* for mmap vaddr */
	ret = ioctl(g_vcam_fd, HB_VCAM_MEMINFO, &mem_info);
	if (ret < 0) {
		vin_err("get mem info ioctl fail\n");
		return -1;
	}
	g_mem_len = mem_info.size;
	g_paddr_base = mem_info.base;
	g_vaddr_base = (uint64_t)mmap(NULL, mem_info.size, PROT_READ|PROT_WRITE, MAP_SHARED, g_mem_fd, mem_info.base);
	return 0;
}

/*
 * send msg to daemon:
 * send cmd type start frame
 * send first frame addr msg
 */
int32_t hb_vin_vcam_start()
{
	int32_t ret;
	hb_vcam_msg_t first_group;

	vin_info("vcam start\n");
	ret = ioctl(g_vcam_fd, HB_VCAM_START, &first_group);
	if (ret < 0) {
		vin_err("vcam start get first group fail\n");
		return -1;
	}
	first_group.info_type = CMD_VCAM_START;
	ret = write(g_sock_fd, &first_group, sizeof(hb_vcam_msg_t));
	if(ret != sizeof(hb_vcam_msg_t)) {
		vin_err("hb_vin_vcam_start write fail ret %d\n", ret);
		return -1;
	}
	return 0;
}

/*
 * send msg to daemon:
 * send cmd type stop frame
 */
int32_t hb_vin_vcam_stop()
{
	int32_t ret;
	hb_vcam_msg_t stop_msg;

	vin_info("vcam stop\n");
	ret = ioctl(g_vcam_fd, HB_VCAM_STOP, &stop_msg);  // block ioctl
	if (ret < 0) {
		vin_err("vcam stop ioctl fail\n");
		return -1;
	}
	stop_msg.info_type = CMD_VCAM_STOP;
	ret = write(g_sock_fd, &stop_msg, sizeof(hb_vcam_msg_t));
	if(ret != sizeof(hb_vcam_msg_t)) {
		vin_err("%s hb_vin_vcam_stop write fail ret %d\n", 
			ret);
	}
	vin_info("vcam stop\n");
	return 0;
}

/* disconnect daemon service */
int32_t hb_vin_vcam_deinit()
{
	int32_t ret;
	hb_vcam_msg_t deinit_msg;

	vin_info("vcam deinit\n");
	ret = ioctl(g_vcam_fd, HB_VCAM_DEINIT, &deinit_msg);  // block ioctl
	if (ret < 0) {
		vin_err("vcam deinit ioctl fail\n");
	}
	deinit_msg.info_type = CMD_VCAM_DEINIT;
	ret = write(g_sock_fd, &deinit_msg, sizeof(hb_vcam_msg_t));  // TBD need flags or not
	if(ret != sizeof(hb_vcam_msg_t)) {
		vin_err("hb_vin_vcam_deinit write fail ret %d\n", 
			ret);
	}
	munmap((void *)g_vaddr_base, g_mem_len);
	close(g_mem_fd);
	close(g_sock_fd);
	close(g_vcam_fd);
	vin_info("vcam deinit end\n");
	return 0;
}
