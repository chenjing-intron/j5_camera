/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef HB_X2A_VIO_HB_VIO_BUFFER_MGR_H
#define HB_X2A_VIO_HB_VIO_BUFFER_MGR_H

#include <stdbool.h>
#include <semaphore.h>
#include "hb_utils.h"
#include "list.h"

#define HB_VIO_BUFFER_MAX (32u)
#define HB_VIO_BUFFER_MAX_PLANES 3u

#define DEBUG_INFO_TRACE
#define DEBUG_KERNEL_GROUP_BIND

typedef struct list_head queue_node;

#define bufmgr_en_lock(this)\
	do {\
		pthread_mutex_lock(&this->lock);\
	} while (0)

#define bufmgr_un_lock(this)\
	do {\
		pthread_mutex_unlock(&this->lock);\
	} while (0)

#define HW_FORMAT_YUV422_8BIT	0x1Eu
#define HW_FORMAT_YUV422_10BIT	0x1Fu
#define HW_FORMAT_RAW8		0x2Au
#define HW_FORMAT_RAW10 	0x2Bu
#define HW_FORMAT_RAW12 	0x2Cu
#define HW_FORMAT_RAW14 	0x2Du
#define HW_FORMAT_RAW16 	0x2Eu
#define HW_FORMAT_RAW20 	0x2Fu

typedef struct buf_offset_s {
	uint16_t width;
	uint16_t height;
	uint16_t stride_size;
	uint32_t offset[3];	//y,uv  or raw/ dol2 / dol3
} buf_offset_t;

typedef enum VIO_DATA_TYPE_S {
	HB_VIO_DATA_TYPE_INVALID = -1,
	HB_VIO_IPU_DS0_DATA = 0,
	HB_VIO_IPU_DS1_DATA,
	HB_VIO_IPU_DS2_DATA,
	HB_VIO_IPU_DS3_DATA,
	HB_VIO_IPU_DS4_DATA,
	HB_VIO_IPU_US_DATA,	// 5
	HB_VIO_PYM_FEEDBACK_SRC_DATA,	// for debug
	HB_VIO_PYM_DATA,//7
	HB_VIO_SIF_FEEDBACK_SRC_DATA,
	HB_VIO_SIF_RAW_DATA,
	HB_VIO_SIF_YUV_DATA,//10
	HB_VIO_ISP_YUV_DATA,	// for debug, a process result for raw feedback
	HB_VIO_GDC_DATA,
	HB_VIO_GDC1_DATA,
	HB_VIO_IARWB_DATA,
	HB_VIO_GDC_FEEDBACK_SRC_DATA,//15
	HB_VIO_GDC1_FEEDBACK_SRC_DATA,
	HB_VIO_PYM_LAYER_DATA,
	HB_VIO_MD_DATA,
	HB_VIO_ISP_RAW_DATA,
	HB_VIO_PYM_COMMON_DATA,
	HB_VIO_PYM_DATA_V2,//21
	HB_VIO_CIM_RAW_DATA,
	HB_VIO_CIM_YUV_DATA,
	HB_VIO_EMBED_DATA,
	HB_VIO_DATA_TYPE_MAX
} VIO_DATA_TYPE_E;

typedef enum buffer_state {
	BUFFER_AVAILABLE,
	BUFFER_PROCESS,
	BUFFER_DONE,
	BUFFER_REPROCESS,
	BUFFER_USER,
	BUFFER_INVALID
} buffer_state_e;

/* info :
 * y,uv             2 plane
 * raw              1 plane
 * raw, raw         2 plane(dol2)
 * raw, raw, raw    3 plane(dol3)
 **/
typedef struct address_info_s {
	uint16_t width;
	uint16_t height;
	uint16_t stride_size;
	char *addr[HB_VIO_BUFFER_MAX_PLANES];
	uint64_t paddr[HB_VIO_BUFFER_MAX_PLANES];
} address_info_t;

typedef struct image_info_s {
	uint16_t sensor_id;
	uint32_t pipeline_id;
	uint32_t frame_id;
	uint64_t time_stamp;//HW time stamp
	struct timeval tv;//system time of hal get buf
	int32_t buf_index;
	int32_t img_format;
	int32_t fd[HB_VIO_BUFFER_MAX_PLANES];//ion buf fd
	uint32_t size[HB_VIO_BUFFER_MAX_PLANES];
	uint32_t planeCount;
	uint32_t dynamic_flag;
	uint32_t water_mark_line;
	VIO_DATA_TYPE_E data_type;
	buffer_state_e state;
	uint64_t		desc;//temp desc for isp raw feedback. TODO:need remove
} image_info_t;

/*
 * buf type  fd[num]                        size[num]                   addr[num]
 *
 * sif buf : fd[0(raw)]                     size[0(raw)]                addr[0(raw)]
 * sif dol2: fd[0(raw),1(raw)]              size[0(raw),1(raw)]         addr[0(raw),1(raw)]
 * sif dol3: fd[0(raw),1(raw),2(raw)]       size[0(raw),1(raw),2(raw)]  addr[0(raw),1(raw),2(raw)]
 * ipu buf : fd[0(y),1(c)]                  size[0(y),1(c)]             addr[0(y),1(c)]
 * pym buf : fd[0(all channel)]             size[0(all output)]         addr[0(y),1(c)] * 24
 * */

typedef struct hb_vio_buffer_s {
	image_info_t img_info;
	address_info_t img_addr;
} hb_vio_buffer_t;

typedef struct buffer_node_s {
	queue_node node;
	hb_vio_buffer_t vio_buf;
} buf_node_t;

enum Format {
	HB_RGB,
	HB_YUV422,
	HB_YUV420SP,
	HB_YUV444,
	HB_RAW,//raw8
	HB_RAW10,
	HB_RAW12,
	HB_RAW16,
	HB_RAW24,
	HB_YUV420SP_RAW12, /* yuv420 & raw12 */
	HB_YUV422_RAW12, /* yuv422 & raw12 */
	HB_YUV420SP_RAW16, /* yuv420 & raw16 */
	HB_YUV422_RAW16, /* yuv422 & raw16 */
	HB_IR8,
	HB_FORMAT_MAX
};

typedef struct buffer_mgr_s {
	VIO_DATA_TYPE_E buffer_type;	//pym buffer is special
	uint32_t pipeline_id;
	uint32_t num_buffers;
	void *buf_nodes;	//buf_node_t or pym_buf_adv_node_t * num_buffers
	uint32_t queued_count[BUFFER_INVALID];
	queue_node buffer_queue[BUFFER_INVALID];
	sem_t sem[BUFFER_INVALID];
	pthread_mutex_t lock;
} buffer_mgr_t;
buffer_mgr_t *cim_buf_mgr_create(uint32_t pipeline_id,
				    VIO_DATA_TYPE_E buffer_type);
void cim_buf_mgr_destroy(buffer_mgr_t * this);

int32_t cim_buf_mgr_init(buffer_mgr_t * this, uint32_t buffer_num);
void cim_buf_mgr_deinit(buffer_mgr_t * this);

#endif //HB_X2A_VIO_HB_VIO_BUFFER_MGR_H
