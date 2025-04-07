/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef HB_X2A_VIO_DEV_IOCTL_H
#define HB_X2A_VIO_DEV_IOCTL_H
#include <stdint.h>
#include <unistd.h>
#include <poll.h>
#include "hb_buffer_mgr.h"
#include "hb_vin_data_info.h"

#define DEV_POLL_TIMEOUT 5000 //ms

#define VIO_IOC_MAGIC 'p'

typedef struct dynamic_fps_s {
	uint32_t skip_frame;
	uint32_t in_fps;
	uint32_t out_fps;
} dynamic_fps_t;

#define VIO_IOC_INIT             \
	_IOC((uint32_t)_IOC_WRITE, (uint32_t)VIO_IOC_MAGIC, 0u, (uint32_t)sizeof(int32_t))
#define VIO_IOC_STREAM           \
	_IOC((uint32_t)_IOC_WRITE, (uint32_t)VIO_IOC_MAGIC, 1u, (uint32_t)sizeof(int32_t))
#define VIO_IOC_QBUF             \
	_IOC((uint32_t)_IOC_WRITE, (uint32_t)VIO_IOC_MAGIC, 2u, (uint32_t)sizeof(int32_t))
#define VIO_IOC_DQBUF            \
	_IOC((uint32_t)_IOC_READ, (uint32_t)VIO_IOC_MAGIC, 3u, (uint32_t)sizeof(int32_t))
#define VIO_IOC_REQBUFS       	 \
	_IOC((uint32_t)_IOC_WRITE, (uint32_t)VIO_IOC_MAGIC, 4u, (uint32_t)sizeof(int32_t))
#define VIO_IOC_END_OF_STREAM    \
	_IOC((uint32_t)_IOC_WRITE, (uint32_t)VIO_IOC_MAGIC, 5u, (uint32_t)sizeof(int32_t))
#define VIO_IOC_BIND_GROUP       \
	_IOC((uint32_t)_IOC_WRITE, (uint32_t)VIO_IOC_MAGIC, 6u, (uint32_t)sizeof(int32_t))
#define VIO_IOC_SET_PARAM        \
	_IOC((uint32_t)_IOC_WRITE, (uint32_t)VIO_IOC_MAGIC, 7u, (uint32_t)sizeof(int32_t))
#define VIO_IOC_QUERYBUF       	 \
	_IOC((uint32_t)(((uint32_t)_IOC_WRITE) | ((uint32_t)_IOC_READ)) , \
	(uint32_t)VIO_IOC_MAGIC, 8u, (uint32_t)sizeof(int32_t))
#define VIO_IOC_GET_FREE_BUF     \
	_IOC((uint32_t)_IOC_READ, (uint32_t)VIO_IOC_MAGIC, 9u, (uint32_t)sizeof(int32_t))
#define VIO_IOC_PUT_FREE_BUF     \
	_IOC((uint32_t)_IOC_WRITE, (uint32_t)VIO_IOC_MAGIC, 10u, (uint32_t)sizeof(int32_t))
#define VIO_IOC_FORCE_STOP       \
	_IOC((uint32_t)_IOC_WRITE, (uint32_t)VIO_IOC_MAGIC, 11u, (uint32_t)sizeof(int32_t))
#define VIO_IOC_FLUSH_FRAME      \
	_IOC((uint32_t)_IOC_WRITE, (uint32_t)VIO_IOC_MAGIC, 12u, (uint32_t)sizeof(int32_t))
#define VIO_IOC_RESET      	 	 \
	_IOC((uint32_t)_IOC_WRITE, (uint32_t)VIO_IOC_MAGIC, 13u, (uint32_t)sizeof(int32_t))
#define VIO_IOC_DYNAMIC_FPS      \
	_IOC((uint32_t)_IOC_WRITE, (uint32_t)VIO_IOC_MAGIC, 16u, (uint32_t)sizeof(int32_t))
#define VIO_IOC_EMB_QUERYBUF     \
	_IOC((uint32_t)(((uint32_t)_IOC_WRITE) | ((uint32_t)_IOC_READ)) , \
	(uint32_t)VIO_IOC_MAGIC, 17u, (uint32_t)sizeof(int32_t))
#define VIO_IOC_EMB_QBUF      \
	_IOC((uint32_t)_IOC_WRITE, (uint32_t)VIO_IOC_MAGIC, 18u, (uint32_t)sizeof(int32_t))
#define VIO_IOC_EMB_DQBUF            \
	_IOC((uint32_t)_IOC_READ, (uint32_t)VIO_IOC_MAGIC, 19u, (uint32_t)sizeof(int32_t))
#define VIO_IOC_GET_STAT	     _IOR((uint32_t)VIO_IOC_MAGIC, 21, int)
#define VIO_IOC_WAIT_FE			 _IO((uint32_t)VIO_IOC_MAGIC, 22)

#define MAGIC_NUMBER	0x12345678
#define PAGE_SHIFT	12

struct buffer_info {
	uint32_t index;
	uint32_t planecount;
	int32_t share_id[HB_VIO_BUFFER_MAX_PLANES];
	uint64_t planeSize[HB_VIO_BUFFER_MAX_PLANES];
	uint64_t paddr[HB_VIO_BUFFER_MAX_PLANES];
	char *addr[HB_VIO_BUFFER_MAX_PLANES];
};

struct frame_info {
	uint32_t frame_id;
	uint64_t timestamps;
	struct timeval tv;
	uint32_t format;
	uint32_t height;
	uint32_t width;
	uint64_t addr[7];
	uint32_t pre_int;
	uint32_t num_planes;
	int32_t bufferindex;
	uint32_t pixel_length;
	uint32_t dynamic_flag;
};

void vin_print_frame_info(struct frame_info *info);
int32_t vin_entity_put_done_buf(buffer_mgr_t * mgr, hb_vio_buffer_t * buf);
int32_t vin_dev_node_dqbuf_poll(int32_t fd, struct frame_info *info,
	int32_t need_poll);
int32_t vin_dev_node_qbuf(int32_t fd, const hb_vio_buffer_t * buf);
void vin_frame_info_to_buf(image_info_t * to_buf, struct frame_info *info);
int32_t vin_node_open(const char *filename, int32_t oflag, ...);
int32_t vin_ioc_init(int32_t fd, void *cfg);
int32_t vin_ioc_stream(int32_t fd, uint32_t *value);
int32_t vin_ioc_reqbufs(int32_t fd, uint32_t *buffers);
int32_t vin_ioc_querybuf(int32_t fd, struct buffer_info *buffer);
int32_t vin_ioc_dqbuf(int32_t fd, struct frame_info *frameinfo);
int32_t vin_ioc_emb_querybuf(int fd, struct buffer_info *buffer);
int32_t vin_ioc_get_emb_buf(int fd, struct frame_info *frameinfo);
int32_t vin_ioc_put_emb_buf(int fd, struct frame_info *frameinfo);
int32_t vin_ioc_reset(int32_t fd);
int32_t vin_ioc_set_dynamic_fps(int32_t fd, dynamic_fps_t *param);
int32_t vin_ioc_get_stat_info(int32_t fd, struct vio_statinfo *statinfo);
int32_t vin_ioc_set_param(int32_t fd, void *param);
int32_t vin_ioc_bind_group(int32_t fd, uint32_t *instance);
int32_t vin_ioc_end_of_stream(int32_t fd, uint32_t *instance);
int32_t vin_ioc_wait_fe(int32_t fd);
int32_t vin_node_close(int32_t fd);
#endif //HB_X2A_VIO_DEV_IOCTL_H
