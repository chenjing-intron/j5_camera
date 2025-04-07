/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <fcntl.h>
#include "dev_ioctl.h"
#include "hb_utils.h"

static int32_t vin_ioc_qbuf(int32_t fd, struct frame_info *frameinfo);
//PRQA S ALL ++
void vin_print_frame_info(struct frame_info *info)
{
	vin_dbg("frameInfo bufferindex(%d)\n", info->bufferindex);
	vin_dbg("frameInfo format(%d)\n", info->format);
	vin_dbg("frameInfo width(%d)\n", info->width);
	vin_dbg("frameInfo height(%d)\n", info->height);
	vin_dbg("frameInfo frame_id(%d)\n", info->frame_id);
	vin_dbg("frameInfo planes(%d)\n", info->num_planes);
	vin_dbg("frameInfo addr0(%u)\n", info->addr[0]);
	vin_dbg("frameInfo addr1(%u)\n", info->addr[1]);
}
//PRQA S ALL --

int32_t vin_dev_node_qbuf(int32_t fd, const hb_vio_buffer_t *buf)
{
	int32_t ret = -1;
	uint32_t i;
	struct frame_info frameinfo;

	frameinfo.bufferindex = buf->img_info.buf_index;
	if (buf->img_info.planeCount > HB_VIO_BUFFER_MAX_PLANES) {
		vin_err("qbuf wrong planecount %d\n", buf->img_info.planeCount);/*PRQA S 1036,4423,1035*/
		return ret;
	}
	for (i = 0; i < buf->img_info.planeCount; i++) {
		frameinfo.addr[i] = buf->img_addr.paddr[i];
	}
	frameinfo.num_planes = buf->img_info.planeCount;

	ret = vin_ioc_qbuf(fd, &frameinfo);
	if (ret < 0) {
		vin_err("dev%d qbuf failed: %d, %s\n", fd, errno, strerror(errno));/*PRQA S 1036,4423,1035*/
	} else {
		vin_dbg("dev%d qbuf buf index %d done\n", fd, frameinfo.bufferindex);/*PRQA S 1036,4423,1035*/
	}
	return ret;
}

int32_t vin_dev_node_dqbuf_poll(int32_t fd, struct frame_info *info, int32_t need_poll)
{
	int32_t ret = -1;
	if ((fd < 0) || (info == NULL)) {
		vin_err("invalid param(%d).\n", fd);/*PRQA S 1036,4423,1035*/
		return ret;
	}

	if (need_poll == 1) {
		struct pollfd pfds = { 0 };

		pfds.fd = fd;
		pfds.events = POLLIN | POLLERR;/*PRQA S 4542*/
		pfds.revents = 0;

		ret = poll(&pfds, 1, DEV_POLL_TIMEOUT);
		if (ret < 0) {
			vin_err("dev poll err: %d, %s\n", errno, strerror(errno));/*PRQA S 1036,4423,1035*/
			return ret;
		}
		if (ret == 0) {
			vin_err("dev poll Timeout(%d): %d, %s\n", DEV_POLL_TIMEOUT,/*PRQA S 1036,4423,1035*/
				errno, strerror(errno));
			return -1;
		}
		//ret > 0 : POLLIN(data ready) or POLLERR(frame drop)
		if ((pfds.revents & POLLIN) != 0) {/*PRQA S 4542*/
			ret = vin_ioc_dqbuf(fd, info);
			if (ret < 0) {
				vin_err("failed to ioctl: dq (%d - %s)", errno, strerror(errno));/*PRQA S 1036,4423,1035*/
				return ret;
			}
			vin_dbg("dev(%d) dq success index %d\n", fd, info->bufferindex);/*PRQA S 1036,4423,1035*/
		}

		if ((pfds.revents & POLLERR) != 0) {/*PRQA S 4542*/
			vin_err("dev fd(%d) frame drop!\n", fd);/*PRQA S 1036,4423,1035*/
			return -1;
		}
	} else {
		ret = vin_ioc_dqbuf(fd, info);
		if (ret < 0) {
			vin_err("failed to ioctl: dq (%d - %s)", errno, strerror(errno));/*PRQA S 1036,4423,1035*/
			return ret;
		}
		vin_dbg("dev(%d) dq success index %d\n", fd, info->bufferindex);/*PRQA S 1036,4423,1035*/
	}
	return ret;
}

//PRQA S ALL ++
static int32_t __node_open(const char *filename, int32_t oflag, va_list ap)
{
	mode_t mode = 0;
	int32_t fd;
	if ((oflag & O_CREAT) != 0)
		mode = va_arg(ap, int);
	fd = open(filename, oflag, mode);
	return fd;
}
int32_t vin_node_open(const char *filename, int32_t oflag, ...)
{
	va_list ap;
	int32_t fd;

	va_start(ap, oflag);
	fd = __node_open(filename, oflag, ap);
	va_end(ap);

	return fd;
}
//PRQA S ALL --

int32_t vin_ioc_init(int32_t fd, void *cfg)
{
	int32_t ret = -1;
	if ((fd < 0) || (cfg == NULL)) {
		vin_err("invalid param: %d", fd);/*PRQA S 1036,4423,1035*/
		return ret;
	}
	ret = ioctl(fd, VIO_IOC_INIT, cfg);
	if (ret < 0) {
		vin_err("failed to ioctl: VIO_IOC_INIT (%d - %s)\n",/*PRQA S 1036,4423,1035*/
				errno, strerror(errno));
		return ret;
	}
	return ret;
}

int32_t vin_ioc_stream(int32_t fd, uint32_t *value)
{
	int32_t ret = -1;
	if (fd < 0) {
		vin_err("invalid param: %d", fd);/*PRQA S 1036,4423,1035*/
		return ret;
	}
	ret = ioctl(fd, VIO_IOC_STREAM, value);
	if (ret < 0) {
		vin_err("failed to ioctl: VIO_IOC_STREAM (%d - %s)\n",/*PRQA S 1036,4423,1035*/
				errno, strerror(errno));
		return ret;
	}
	return ret;
}

int32_t vin_ioc_reqbufs(int32_t fd, uint32_t *buffers)
{
	int32_t ret = -1;
	if (fd < 0) {
		vin_err("invalid param: %d", fd);/*PRQA S 1036,4423,1035*/
		return ret;
	}
	ret = ioctl(fd, VIO_IOC_REQBUFS, buffers);
	if (ret < 0) {
		vin_err("failed to ioctl: VIO_IOC_REQBUFS (%d - %s)\n",/*PRQA S 1036,4423,1035*/
				errno, strerror(errno));
		return ret;
	}

	return ret;
}

int32_t vin_ioc_querybuf(int32_t fd, struct buffer_info *buffer)
{
	int32_t ret = -1;
	if ((fd < 0) || (buffer == NULL)) {
		vin_err("invalid param: %d", fd);/*PRQA S 1036,4423,1035*/
		return ret;
	}
	ret = ioctl(fd, VIO_IOC_QUERYBUF, buffer);
	if (ret < 0) {
		vin_err("failed to ioctl: VIO_IOC_QUERYBUF (%d - %s)\n",/*PRQA S 1036,4423,1035*/
				errno, strerror(errno));
		return ret;
	}
	return ret;
}

static int32_t vin_ioc_qbuf(int32_t fd, struct frame_info *frameinfo)
{
	int32_t ret = -1;
	if ((fd < 0) || (frameinfo == NULL)) {
		vin_err("invalid param: %d", fd);/*PRQA S 1036,4423,1035*/
		return ret;
	}
	ret = ioctl(fd, VIO_IOC_QBUF, frameinfo);
	if (ret < 0) {
		vin_err("failed to ioctl: VIO_IOC_QBUF (%d - %s)\n",/*PRQA S 1036,4423,1035*/
				errno, strerror(errno));
		return ret;
	}
	return ret;
}

int32_t vin_ioc_dqbuf(int32_t fd, struct frame_info *frameinfo)
{
	int32_t ret = -1;
	if ((fd < 0) || (frameinfo == NULL)) {
		vin_err("invalid param: %d", fd);/*PRQA S 1036,4423,1035*/
		return ret;
	}
	ret = ioctl(fd, VIO_IOC_DQBUF, frameinfo);
	if (ret < 0) {
		vin_err("failed to ioctl: VIO_IOC_DQBUF (%d - %s)\n",/*PRQA S 1036,4423,1035*/
				errno, strerror(errno));
		return ret;
	}
	return ret;
}

//only used for src node in fb mode
int vin_ioc_get_emb_buf(int fd, struct frame_info *frameinfo)
{
	int ret = -1;
	if (fd < 0 || frameinfo == NULL) {
		vin_err("invalid param: %d", fd);
		return ret;
	}
	ret = ioctl(fd, VIO_IOC_EMB_DQBUF, frameinfo);
	if (ret) {
		vin_err("failed to ioctl: SRC GET FREE BUF(%d - %s)\n",
				errno, strerror(errno));
		return ret;
	}
	return ret;
}

//only used for src node in fb mode
int vin_ioc_put_emb_buf(int fd, struct frame_info *frameinfo)
{
	int ret = -1;
	if (fd < 0 ) {
		vin_err("invalid param: %d", fd);
		return ret;
	}
	ret = ioctl(fd, VIO_IOC_EMB_QBUF, frameinfo);
	if (ret) {
		vin_err("failed to ioctl: SRC PUT FREE BUF(%d - %s)\n",
				errno, strerror(errno));
		return ret;
	}
	return ret;
}

int vin_ioc_emb_querybuf(int fd, struct buffer_info *buffer)
{
	int ret = -1;
	if (fd < 0 || buffer == NULL) {
		vin_err("invalid param: %d", fd);
		return ret;
	}
	ret = ioctl(fd, VIO_IOC_EMB_QUERYBUF, buffer);
	if (ret) {
		vin_err("failed to ioctl: VIO_IOC_QUERYBUF (%d - %s)\n",
				errno, strerror(errno));
		return ret;
	}
	return ret;
}

int32_t vin_ioc_set_param(int32_t fd, void *param)
{
	int32_t ret = -1;
	if ((fd < 0) || (param == NULL)) {
		vin_err("invalid param: %d", fd);/*PRQA S 1036,4423,1035*/
		return ret;
	}
	ret = ioctl(fd, VIO_IOC_SET_PARAM, param);
	if (ret < 0) {
		vin_err("failed to ioctl: VIO_IOC_SET_PARAM (%d - %s)\n",/*PRQA S 1036,4423,1035*/
				errno, strerror(errno));
		return ret;
	}
	return ret;
}

int32_t vin_ioc_set_dynamic_fps(int32_t fd, dynamic_fps_t *param)
{
	int32_t ret = -1;
	if ((fd < 0) || (param == NULL)) {
		vin_err("invalid param: %d", fd);/*PRQA S 1036,4423,1035*/
		return ret;
	}
	ret = ioctl(fd, VIO_IOC_DYNAMIC_FPS, param);
	if (ret < 0) {
		vin_err("failed to ioctl: VIO_IOC_DYNAMIC_FPS (%d - %s)\n",/*PRQA S 1036,4423,1035*/
				errno, strerror(errno));
		return ret;
	}
	return ret;
}

int32_t vin_ioc_get_stat_info(int32_t fd, struct vio_statinfo *statinfo)
{
	int32_t ret = -1;
	if ((fd < 0) || (statinfo == NULL)) {
		vin_err("invalid param: %d", fd);/*PRQA S 1036,4423,1035*/
		return ret;
	}
	ret = ioctl(fd, VIO_IOC_GET_STAT, statinfo);
	if (ret < 0) {
		vin_err("failed to ioctl: VIO_IOC_GET_STAT (%d - %s)\n",/*PRQA S 1036,4423,1035*/
				errno, strerror(errno));
		return ret;
	}
	return ret;
}

int32_t vin_ioc_bind_group(int32_t fd, uint32_t *instance)
{
	int32_t ret = -1;
	if (fd < 0) {
		vin_err("invalid param: %d", fd);/*PRQA S 1036,4423,1035*/
		return ret;
	}
	ret = ioctl(fd, VIO_IOC_BIND_GROUP, instance);
	if (ret < 0) {
		vin_err("failed to ioctl: VIO_IOC_BIND_GROUP (%d - %s)\n",/*PRQA S 1036,4423,1035*/
				errno, strerror(errno));
		return ret;
	}
	return ret;
}

int32_t vin_ioc_end_of_stream(int32_t fd, uint32_t *instance)
{
	int32_t ret = -1;
	if (fd < 0) {
		vin_err("invalid param: %d", fd);/*PRQA S 1036,4423,1035*/
		return ret;
	}
	ret = ioctl(fd, VIO_IOC_END_OF_STREAM, instance);
	if (ret < 0) {
		vin_err("failed to ioctl: VIO_IOC_BIND_GROUP (%d - %s)\n",/*PRQA S 1036,4423,1035*/
				errno, strerror(errno));
		return ret;
	}
	return ret;
}

int32_t vin_ioc_reset(int32_t fd)
{
	int32_t ret = -1;
	if (fd < 0) {
		vin_err("invalid param: %d", fd);/*PRQA S 1036,4423,1035*/
		return ret;
	}
	ret = ioctl(fd, VIO_IOC_RESET);
	if (ret < 0) {
		vin_err("failed to ioctl: VIO_IOC_RESET (%d - %s)\n",/*PRQA S 1036,4423,1035*/
				errno, strerror(errno));
		return ret;
	}
	return ret;
}

int32_t vin_ioc_wait_fe(int32_t fd)
{
	int32_t ret = -1;
	if (fd < 0) {
		vin_err("invalid param: %d", fd);/*PRQA S 1036,4423,1035*/
		return ret;
	}
	ret = ioctl(fd, VIO_IOC_WAIT_FE);
	if (ret < 0) {
		vin_err("failed to ioctl: VIO_IOC_WAIT_FE (%d - %s)\n",/*PRQA S 1036,4423,1035*/
				errno, strerror(errno));
		return ret;
	}
	return ret;
}

int32_t vin_node_close(int32_t fd)
{
	int32_t ret = -1;
	if (fd < 0) {
		vin_err("invalid param: %d", fd);/*PRQA S 1036,4423,1035*/
	} else {
		ret = close(fd);
	}
	return ret;
}
