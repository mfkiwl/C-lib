/*
 * at.h
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

#ifndef __AT_H
#define __AT_H

#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>
#include "at_uart.h"
#include "FreeRTOSConfig.h"
#include "cr600_tools.h"
#include "cr600_partition.h"

typedef enum
{
    MODE_NORMAL = 0,
    MODE_TRANSPARENT
} at_tranfermode_t;

/* at cmd type */
typedef struct
{
    uint8_t const *at_cmd_name;
    void (*at_test_cmd)(void);
    void (*at_query_cmd)(void);
    void (*at_setup_cmd)(uint8_t *pPara);
    void (*at_exe_cmd)(void);
} at_fun_t;

/* current cr600 parameter pointer */
//extern cr600_param_t *cr600_param_cur;

/* at init, using at module need call at_init() */
extern void at_init(void);

/* this is printf that well route to UDP or usart
 * warnning: when route to UDP, at_printf will call 
 * xQueueSend() send data to tcpip_thread task.
 */
extern int at_printf(const char *fmt, ...);

/* send data-block, route to UDP or usart 
 * warnning: when route to UDP, at_printf will call 
 * xQueueSend() send data to tcpip_thread task.
*/
extern void at_send_block(const uint8_t *data, uint32_t len);

/* print ok */
extern void at_print_ok(void);

/* print error */
extern void at_print_error(void);

/* print paramter error */
extern void at_print_error_param(void);

/* register custom AT cmd to at module */
extern uint8_t at_register(at_fun_t *fun);

/* register custom AT cmds array to at module */
extern uint8_t at_register_array(at_fun_t *fun, uint32_t array_size);

/* usart handle to receive usart data */
extern void AT_UART_IRQHandler(uint8_t ch);

#endif
