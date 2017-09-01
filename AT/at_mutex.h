/*
 * at_mutex.h
 *
 * Created by Cloudersemi CR600 SDK Team 2017-03-09
 *
 * Copyright (c) 2016-2017, Shanghai Clouder Semiconductor Co.,Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * o Neither the name of Shanghai Clouder Semiconductor Co.,Ltd. nor the names
 *   of its contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __AT_MUTEX_H__
#define __AT_MUTEX_H__

#include "at_config.h"
#include "freertos\FreeRTOS.h"
#include "freertos\semphr.h"

#define AT_MUTEX_DEFAULT_BLOCK_TIME    portMAX_DELAY

typedef struct cs_mutex
{
    /* this is freertos mutex lock handle */
    xSemaphoreHandle mutex;

    /* xSemaphoreTake block time */
    TickType_t  block_time;

    /* take this mutex */
    uint8_t (* take)(struct cs_mutex *);

    /* give this mutex */
    uint8_t (* give)(struct cs_mutex *);
} cs_mutex_t;

typedef struct cs_queue
{
    /* this is freertos queue lock handle */
    QueueHandle_t queue;

    /* queue length */
    uint16_t length;

    /* item size */
    uint16_t size;

    /* send block time */
    TickType_t  snd_block_time;

    /* recv block time */
    TickType_t  recv_block_time;

    /* send queue */
    uint8_t (* snd)(struct cs_queue *, void *p_data);

    /* send queue from isr */
    uint8_t (* snd_isr)(struct cs_queue *, void *p_data);

    /* receive queue */
    uint8_t (* rcv)(struct cs_queue *, void *p_data);
} cs_queue_t;

void queue_init(cs_queue_t *queue, uint16_t length, uint16_t size);
void mutex_init(cs_mutex_t *mutex);

#endif
