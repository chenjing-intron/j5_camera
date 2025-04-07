/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef J5_VIO_UTILS_HB_UTILS_H_
#define J5_VIO_UTILS_HB_UTILS_H_

#include "cJSON.h"
#include <stdio.h>
#include <stdint.h>
#include <sys/time.h>
#include "vin_log.h"

#define HB_VIO_NAME_STR_SIZE (128)
#define HB_VIO_MAX_WEIGHT 3840
#define HB_VIO_MAX_height 2160

#define min_value(a, b) ((a) < (b) ? (a) : (b))


static inline uint32_t ALIGN_UP(uint32_t a, uint32_t size) {
	return ((a + size - 1) & (~(size - 1)));
}

#define ALIGN_DOWN(a, size)  (a & (~(size - 1)))

#define ALIGN_UP2(d)   ALIGN_UP(d, 2)
#define ALIGN_UP4(d)   ALIGN_UP(d, 4)
#define ALIGN_UP16(d)  ALIGN_UP(d, 16)
#define ALIGN_UP32(d)  ALIGN_UP(d, 32)
#define ALIGN_UP64(d)  ALIGN_UP(d, 64)

#define ALIGN_DOWN2(d)    ALIGN_DOWN(d, 2)
#define ALIGN_DOWN4(d)    ALIGN_DOWN(d, 4)
#define ALIGN_DOWN16(d)   ALIGN_DOWN(d, 16)
#define ALIGN_DOWN32(d)   ALIGN_DOWN(d, 32)
#define ALIGN_DOWN64(d)   ALIGN_DOWN(d, 64)
#define SCALE3_2(a)     ((a*3)>>1)

#define PAGE_SIZE	4096
#define PAGE_SHITF	12

int32_t vin_dumpToFile(char *filename, char *srcBuf, uint32_t size);
int32_t vin_dumpToFile2plane(char *filename, char *srcBuf, char *srcBuf1,
		     uint32_t size, uint32_t size1);
int32_t hb_cim_utils_get_json_value(const cJSON *node, const char *string);

#endif //J5_VIO_UTILS_HB_UTILS_H_
