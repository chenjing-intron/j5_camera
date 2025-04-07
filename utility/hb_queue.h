/***************************************************************************
* COPYRIGHT NOTICE
* Copyright 2019 Horizon Robotics, Inc.
* All rights reserved.
***************************************************************************/
#ifndef UTILITY_HB_QUEUE_H_
#define UTILITY_HB_QUEUE_H_

#include <pthread.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "../inc/vin_log.h"

#define MUTEX_HANDLE(mutex_handle) pthread_mutex_t mutex_handle
#define MUTEX_INIT(mutex_handle) pthread_mutex_init(&mutex_handle, NULL)
#define MUTEX_LOCK(mutex_handle) pthread_mutex_lock(&mutex_handle)
#define MUTEX_UNLOCK(mutex_handle) pthread_mutex_unlock(&mutex_handle)
#define MUTEX_DESTROY(mutex_handle) pthread_mutex_destroy(&mutex_handle)
#define COND_HANDLE(cond_handle) pthread_cond_t cond_handle
#define COND_INIT(cond_handle) pthread_cond_init(&cond_handle, NULL)
#define COND_WAIT(cond_handle, mutex_handle) pthread_cond_wait(&cond_handle, &mutex_handle)
#define COND_SIGNAL(cond_handle) pthread_cond_signal(&cond_handle)
#define COND_BROADCAST(cond_handle) pthread_cond_broadcast(&cond_handle)
#define COND_DESTROY(cond_handle) pthread_cond_destroy(&cond_handle)


typedef struct vin_q_node {
	void *data;
	struct vin_q_node *next;
} vin_q_node;

typedef struct vin_queue {
	struct vin_q_node *head;
	struct vin_q_node *tail;
	MUTEX_HANDLE(mutex);
	COND_HANDLE(cond);
} vin_queue;

vin_queue* vin_queue_create();
void vin_queue_clear(vin_queue *q, void (*free)(void*));
void vin_queue_destroy(vin_queue *q, void (*free_data)(void*));
void vin_queue_enq(vin_queue *q, void *data);
void* vin_queue_deq(vin_queue *q);
bool vin_queue_empty(vin_queue *q);
void vin_queue_enq_signal(vin_queue *q, void *data);
void* vin_queue_deq_wait(vin_queue *q);
#endif  // UTILITY_HB_QUEUE_H_
