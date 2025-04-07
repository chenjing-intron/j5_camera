/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt) "[hb cim utils]:"fmt

#include <string.h>
#include <stdlib.h>
#include "hb_utils.h"

//PRQA S ALL ++
int32_t vin_dumpToFile(char *filename, char *srcBuf, uint32_t size)
{
	FILE *yuvFd;
	char *buffer;

	yuvFd = fopen(filename, "w+");

	if (yuvFd == NULL) {
		vin_err("ERRopen(%s) fail", filename);/*PRQA S 1035,4423,1036*/
		return -1;
	}

	buffer = (char *)malloc(size);

	if (buffer == NULL) {
		vin_err(":malloc file");/*PRQA S 1035,4423,1036*/
		fclose(yuvFd);
		return -1;
	}

	memcpy(buffer, srcBuf, size);

	fflush(stdout);

	fwrite(buffer, 1, size, yuvFd);

	fflush(yuvFd);

	if (yuvFd)
		fclose(yuvFd);
	if (buffer)
		free(buffer);

	vin_err("filedump(%s, size(%d) is successed!!", filename, size);/*PRQA S 1035,4423,1036*/

	return 0;
}

int32_t vin_dumpToFile2plane(char *filename, char *srcBuf, char *srcBuf1,
		     uint32_t size, uint32_t size1)
{

	FILE *yuvFd;
	char *buffer;

	yuvFd = fopen(filename, "w+");

	if (yuvFd == NULL) {
		vin_err("open(%s) fail", filename);/*PRQA S 1035,4423,1036*/
		return -1;
	}

	buffer = (char *)malloc(size + size1);

	if (buffer == NULL) {
		vin_err("ERR:malloc file");/*PRQA S 1035,4423,1036*/
		fclose(yuvFd);
		return -1;
	}

	memcpy(buffer, srcBuf, size);
	memcpy(buffer + size, srcBuf1, size1);

	fflush(stdout);

	fwrite(buffer, 1, size + size1, yuvFd);

	fflush(yuvFd);

	if (yuvFd)
		fclose(yuvFd);
	if (buffer)
		free(buffer);

	vin_err("DEBUG:filedump(%s, size(%d) is successed!!", filename, size);/*PRQA S 1035,4423,1036*/

	return 0;
}
//PRQA S ALL --

int32_t hb_cim_utils_get_json_value(const cJSON *node, const char *string)
{
	cJSON *p_node;
	if((node == NULL) || (string == NULL)) {
		vin_warn("node or string is null");/*PRQA S 1035,4423,1036*/
		return 0;
	}
	p_node = cJSON_GetObjectItem(node, string);
	if(p_node == NULL) {
		vin_dbg("%s has empty json value", string);/*PRQA S 1035,4423,1036*/
		return 0;
	}
	return p_node->valueint;
}
