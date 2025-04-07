/*************************************************************************
 *                     COPYRIGHT NOTICE
 *            Copyright 2016-2020 Horizon Robotics, Inc.
 *                   All rights reserved.
 *************************************************************************/
#ifndef UTILITY_HB_VCAM_H_
#define UTILITY_HB_VCAM_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "hb_vin_interface.h"

/* for msg type */
enum vcam_cmd {
	CMD_VCAM_INIT = 1,
	CMD_VCAM_START,
	CMD_VCAM_NEXT_REQUST,
	CMD_VCAM_STOP,
	CMD_VCAM_DEINIT,
	CMD_VCAM_MAX
};

/* for image format */
#define VCAM_YUV_420_8		0x18
#define VCAM_YUV_420_10		0x19
#define VCAM_YUV_422_8		0x1E
#define VCAM_YUV_422_10		0x1F
#define VCAM_RAW_8			0x2A
#define VCAM_RAW_10			0x2B
#define VCAM_RAW_12			0x2C
#define VCAM_RAW_14			0x2D
/* for init info, may be TBD */
typedef struct vcam_init_info_s {
	int32_t ipu_enable;
	int32_t width;
	int32_t height;
	int32_t stride;
	int32_t format;
} vcam_init_info_t;

/* for vcam image info */
typedef struct vcam_img_info_s {
	uint32_t width;
	uint32_t height;
	uint32_t stride;
	uint32_t format;
} vcam_img_info_t;

/* for vcam slot info */
typedef struct vcam_slot_info_s {
	int32_t						slot_id;
	int32_t						cam_id;
	int32_t						frame_id;
	int64_t					timestamp;
	vcam_img_info_t			img_info;
} vcam_slot_info_t;

/*
 * for vcam group info
 * group_size = slot_size * slot_num;
 * every_group_addr = base + g_id * group_size;
*/
typedef struct vcam_group_info_s {
	int32_t			g_id;		// group id
	uint64_t	base;		// first group paddr
	int32_t			slot_size;  // a slot size
	int32_t			slot_num;   // a group have slot_num slot
	int32_t			flag;		// 0 free 1 busy
} vcam_group_info_t;

/* for vcam msg info */
typedef struct hb_vcam_msg_s {
	int32_t					info_type;
	vcam_group_info_t	group_info;
	vcam_slot_info_t	slot_info;
} hb_vcam_msg_t;

/* vcam memory info */
struct mem_info_t {
	int32_t size;
	uint64_t base;
};

int32_t hb_vin_vcam_init();
int32_t hb_vin_vcam_start();
int32_t hb_vin_vcam_stop();
int32_t hb_vin_vcam_deinit();
int32_t hb_vin_vcam_next_group();
int32_t hb_vin_vcam_get_img(cam_img_info_t *cam_img_info);
int32_t hb_vin_vcam_free_img(cam_img_info_t *cam_img_info);
int32_t hb_vin_vcam_clean(cam_img_info_t *cam_img_info);

#ifdef __cplusplus
}
#endif

#endif  // UTILITY_HB_VCAM_H_
