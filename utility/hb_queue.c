/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2016 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#define pr_fmt(fmt)		"[%s]:[%s:%d]" fmt, __FILE__, __func__, __LINE__

#include "utility/hb_queue.h"

vin_queue* vin_queue_create()
{
	vin_queue *q;
	q = (vin_queue*)malloc(sizeof(*q));
	if (q == NULL) {
		vin_err("malloc fail \n");
		return NULL;
	}

	memset(q, 0, sizeof(*q));
	MUTEX_INIT(q->mutex);
	COND_INIT(q->cond);

	return q;
}

void vin_queue_clear(vin_queue *q, void (*free_data)(void*))
{
	vin_q_node *node, *next;
	if (!q)
		return;

	MUTEX_LOCK(q->mutex);
	next = q->head;
	while (next) {
		node = next;
		next = next->next;

		if (free_data) {
			free_data(node->data);
		} else {
			if (node->data)
				vin_info("memory leak: %p not free", node->data);
		}

		free(node);
	}
	q->head = NULL;
	q->tail = NULL;
	MUTEX_UNLOCK(q->mutex);
}

void vin_queue_destroy(vin_queue *q, void (*free_data)(void*))
{
	vin_q_node *node, *next;
	if (!q)
		return;

	/*
	* clear queue before destory
	*/
	vin_queue_clear(q, free_data);

	MUTEX_DESTROY(q->mutex);
	COND_DESTROY(q->cond);
	free(q);
}

void vin_queue_enq(vin_queue *q, void *data)
{
	vin_q_node *node;
	if (!q)
		return;
	node = (vin_q_node*)malloc(sizeof(*node));
	if (node == NULL) {
		vin_err("malloc fail\n");
		return;
	}
	memset(node, 0, sizeof(*node));
	node->data = data;

	MUTEX_LOCK(q->mutex);
	if (q->head == NULL) {
		q->head = node;
		q->tail = node;
	} else {
		q->tail->next = node;
		q->tail = node;
	}
	MUTEX_UNLOCK(q->mutex);
}


void* vin_queue_deq(vin_queue *q)
{
	vin_q_node *node;
	void *data;
	if (!q)
		return NULL;
	MUTEX_LOCK(q->mutex);
	node = q->head;
	if (q->head) {
		q->head = q->head->next;
		node->next = NULL;
	}
	MUTEX_UNLOCK(q->mutex);

	data = NULL;
	if (node) {
		data = node->data;
		free(node);
	}
	return data;
}

void vin_queue_enq_signal(vin_queue *q, void *data)
{
	vin_q_node *node;
	if (!q)
		return;
	node = (vin_q_node*)malloc(sizeof(*node));
	if (node == NULL) {
		vin_err("malloc fail\n");
		return;
	}
	memset(node, 0, sizeof(*node));
	node->data = data;
	MUTEX_LOCK(q->mutex);
	if (q->head == NULL) {
		q->head = node;
		q->tail = node;
	} else {
		q->tail->next = node;
		q->tail = node;
	}
	COND_SIGNAL(q->cond);
	MUTEX_UNLOCK(q->mutex);
}

void* vin_queue_deq_wait(vin_queue *q)
{
	vin_q_node *node;
	void *data;
	if (!q)
		return NULL;
	MUTEX_LOCK(q->mutex);
	node = q->head;
	if (q->head) {
		q->head = q->head->next;
		node->next = NULL;
	}

	data = NULL;
	if (node) {
		data = node->data;
		free(node);
		MUTEX_UNLOCK(q->mutex);
	} else {
		COND_WAIT(q->cond, q->mutex);
		MUTEX_UNLOCK(q->mutex);
		data = vin_queue_deq(q);
	}
	return data;
}

bool vin_queue_empty(vin_queue *q)
{
	return (q->head == NULL);
}
