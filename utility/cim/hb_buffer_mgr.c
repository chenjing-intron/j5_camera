/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#include <string.h>
#include <pthread.h>
#include "hb_buffer_mgr.h"

static const char *const buf_state_name[BUFFER_INVALID] = {
	"Available",
	"Process",
	"Done",
	"Reprocess",
	"User"
};

buffer_mgr_t *cim_buf_mgr_create(uint32_t pipeline_id,
				    VIO_DATA_TYPE_E buffer_type)
{
	buffer_mgr_t *me = NULL;

	if ((buffer_type > HB_VIO_DATA_TYPE_INVALID) && (buffer_type < HB_VIO_DATA_TYPE_MAX)) {
		me = (buffer_mgr_t *) malloc(sizeof(buffer_mgr_t));/*PRQA S 5118*/
		if (me != NULL) {
			(void)memset((void *)me, 0, sizeof(buffer_mgr_t));
			me->pipeline_id = pipeline_id;
			me->buffer_type = buffer_type;
			vin_dbg("mgr create done! pipeline_id = %u, buffer_type = %d \n",/*PRQA S 1036,4423,1035*/
			     me->pipeline_id, me->buffer_type);
		} else {
			vin_err("mgr create failed, malloc failed !!!\n");/*PRQA S 1036,4423,1035*/
		}
	} else {
		vin_err("mgr create failed !!! pipeline_id = %u, invaild buf type = %d\n",/*PRQA S 1036,4423,1035*/
		     pipeline_id, buffer_type);
	}

	return me;
}

void cim_buf_mgr_destroy(buffer_mgr_t * this)
{
	if (this != NULL) {
		free((void *)this);/*PRQA S 5118*/
		this = NULL;
	}
}

int32_t cim_buf_mgr_init(buffer_mgr_t * this, uint32_t buffer_num)
{
	int32_t ret = -1;
	uint32_t i;

	if ((this == NULL) || (buffer_num > HB_VIO_BUFFER_MAX)) {
		vin_err("invaild param.\n");/*PRQA S 1036,4423,1035*/
		return ret;
	}

	this->buf_nodes = malloc(sizeof(buf_node_t) * buffer_num);/*PRQA S 2707,5118*/
	(void)memset((void *)this->buf_nodes, 0, sizeof(buf_node_t) * buffer_num);
	for (i = 0; i < buffer_num; ++i) {
		((buf_node_t *) (this->buf_nodes) + i)->vio_buf.img_info.buf_index = -1;/*PRQA S 0488*/
		((buf_node_t *) (this->buf_nodes) + i)->vio_buf.img_info.pipeline_id = this->pipeline_id;/*PRQA S 0488*/
		((buf_node_t *) (this->buf_nodes) + i)->vio_buf.img_info.data_type = this->buffer_type;/*PRQA S 0488*/
	}

	this->num_buffers = buffer_num;
	vin_dbg("this->num_buffers = %d, buffer type = %d", this->num_buffers,/*PRQA S 1036,4423,1035*/
		this->buffer_type);
	(void)pthread_mutex_init(&(this->lock), NULL);/*PRQA S 2736*/

	return 0;
}

void cim_buf_mgr_deinit(buffer_mgr_t * this)
{
	if (this != NULL) {
		if (this->buf_nodes != NULL)
			free((void *)this->buf_nodes);/*PRQA S 5118*/
		(void)pthread_mutex_destroy(&this->lock);
	}
}
