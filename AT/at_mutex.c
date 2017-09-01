/*
 * at_mutex.c
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

#include "at_mutex.h"
#include "string.h"

uint8_t mutex_take(struct cs_mutex *p_mutex);
uint8_t mutex_give(struct cs_mutex *p_mutex);
uint8_t queue_snd(struct cs_queue *p_queue, void *p_data);
uint8_t queue_snd_isr(struct cs_queue *p_queue, void *p_data);
uint8_t queue_rcv(struct cs_queue *p_queue, void *p_data);

void mutex_init(cs_mutex_t *p_mutex)
{
    AT_ASSERT(p_mutex);
    memset(p_mutex, 0, sizeof(cs_mutex_t));

    p_mutex->mutex      = xSemaphoreCreateMutex();
    AT_ASSERT(p_mutex->mutex);

    p_mutex->block_time = AT_MUTEX_DEFAULT_BLOCK_TIME;
    p_mutex->take       = mutex_take;
    p_mutex->give       = mutex_give;
}

uint8_t mutex_take(struct cs_mutex *p_mutex)
{
    AT_ASSERT(p_mutex);
    if (xSemaphoreTake(p_mutex->mutex, p_mutex->block_time) != pdPASS)
        return AT_FALSE;
    return AT_TRUE;
}

uint8_t mutex_give(struct cs_mutex *p_mutex)
{
    AT_ASSERT(p_mutex);
    if (xSemaphoreGive(p_mutex->mutex) != pdPASS)
        return AT_FALSE;
    return AT_TRUE;
}

void queue_init(cs_queue_t *p_queue, uint16_t length, uint16_t size)
{
    AT_ASSERT(p_queue);

    memset(p_queue, 0, sizeof(cs_queue_t));

    p_queue->length = length;
    p_queue->size = size;

    p_queue->queue      = xQueueCreate(p_queue->length, p_queue->size);
    AT_ASSERT(p_queue->queue);

    p_queue->snd_block_time  = AT_MUTEX_DEFAULT_BLOCK_TIME;
    p_queue->recv_block_time = AT_MUTEX_DEFAULT_BLOCK_TIME;
    p_queue->snd       = queue_snd;
    p_queue->snd_isr   = queue_snd_isr;
    p_queue->rcv       = queue_rcv;
}

uint8_t queue_snd(struct cs_queue *p_queue, void *p_data)
{
    AT_ASSERT(p_queue);
    if (xQueueSend(p_queue->queue, p_data, p_queue->snd_block_time) != pdPASS)
        return AT_FALSE;
    return AT_TRUE;
}

/* this function must call by irq */
uint8_t queue_snd_isr(struct cs_queue *p_queue, void *p_data)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (errQUEUE_FULL == xQueueSendFromISR(p_queue->queue, p_data, &xHigherPriorityTaskWoken))
    {
        /* did not send to the queue */
        return AT_FALSE;
    }

    /* check the priority task woken flag */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    return AT_TRUE;
}

uint8_t queue_rcv(struct cs_queue *p_queue, void *p_data)
{
    AT_ASSERT(p_queue);
    if (xQueueReceive(p_queue->queue, p_data, p_queue->recv_block_time) != pdPASS)
        return AT_FALSE;
    return AT_TRUE;
}
