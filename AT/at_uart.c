/*
 * at_uart.c
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

#include "at_uart.h"

extern cs_queue_t queue_cmd;

void at_uart_init_default(void)
{
    USART_InitTypeDef USART_InitS;

    RCC_PeriphClockCmd(RCC_Periph_USART2Clock, ENABLE);

    USART_InitS.USART_BaudRate    = 115200;
    USART_InitS.USART_WordLength  = USART_WordLength_8b;
    USART_InitS.USART_StopBits    = USART_StopBits_1;
    USART_InitS.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitS.USART_Parity      = USART_Parity_No;
    USART_InitS.USART_Mode        = USART_Mode_Tx | USART_Mode_Rx;

    USART_DeInit(USART2);
    
    USART_Init(USART2, &USART_InitS);

    USART_ITConfig(AT_CONFIG_USART, USART_IT_RXNE, ENABLE);   // enable the rx interrupt

    NVIC_EnableIRQ(USART2_IRQn);
}

#ifdef AT_USE_USART

cs_buffer_t usart_buffer;

/*********** uart init *************/
void at_uart_init(cr600_param_t *config)
{
    USART_InitTypeDef USART_InitS;

    RCC_PeriphClockCmd(RCC_Periph_USART2Clock, ENABLE);

    USART_InitS.USART_BaudRate    = config->uart.baudrate;
    USART_InitS.USART_WordLength  = config->uart.databits - 5;
    USART_InitS.USART_StopBits    = (config->uart.stopbits == 1) ? USART_StopBits_1 : USART_StopBits_2;
    USART_InitS.USART_HardwareFlowControl =
        (config->uart.flowcontrol == 0) ? USART_HardwareFlowControl_None : USART_HardwareFlowControl_RTS_CTS;
    if (config->uart.parity == 0)
        USART_InitS.USART_Parity  = USART_Parity_No;
    else if (config->uart.parity == 1)
        USART_InitS.USART_Parity  = USART_Parity_Even;
    else if (config->uart.parity == 2)
        USART_InitS.USART_Parity  = USART_Parity_Odd;
    USART_InitS.USART_Mode        = USART_Mode_Tx | USART_Mode_Rx;

    USART_DeInit(USART2);
    USART_Init(USART2, &USART_InitS);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    NVIC_EnableIRQ(USART2_IRQn);
}

void AT_UART_IRQHandler(uint8_t ch)
{
    uint32_t *read_idx = &usart_buffer.read_index;
    uint32_t *save_idx = &usart_buffer.save_index;
    uint8_t  *buf      = usart_buffer.buffer;
    at_queue_msg_t msg = {CMD_ROUTE_USART, {0}};
	
    uint16_t usart_data = ch;
    
	
	/* enter into param update */
	if(at_param_update == 1 && *save_idx != 0 && *read_idx != 0)
	{
		*save_idx = *read_idx = 0;
	}
	buf[*save_idx] = usart_data;
	*save_idx += 1;
	
	if(at_param_update == 1)
		*save_idx &= (AT_BUFFER_SIZE - 1);
	else
		*save_idx &= (AT_CMD_MAX_LEN - 1);
	
	if (at_param_update == 0 && (usart_data == '\n'))
	{
		if (*save_idx <= *read_idx)
		{
			memcpy(&msg.cmd_data, &buf[*read_idx], AT_CMD_MAX_LEN - *read_idx);
			memcpy((uint8_t *)&msg.cmd_data + AT_CMD_MAX_LEN - *read_idx, &buf[0], *save_idx);
		}
		else
		{
			memcpy(&msg.cmd_data, &buf[*read_idx], *save_idx - *read_idx);
		}
		queue_cmd.snd_isr(&queue_cmd, (void *)&msg);
		*read_idx = *save_idx;
	}
	if(at_param_update == 1 && *save_idx == at_param_size + 12)
	{
		if(*(uint32_t *)buf == at_param_addr && *((uint32_t *)(buf + 4)) == at_param_size)
		{
			memset(&msg.cmd_data, 0, AT_CMD_MAX_LEN);
			strcpy((char *)msg.cmd_data, at_param_prompt);
			queue_cmd.snd_isr(&queue_cmd, (void *)&msg);
		}
		else
		{
			*save_idx = *read_idx = 0;
			at_param_update = at_param_addr = at_param_size = 0;
			at_print_error();
			return;
		}
	}
}

#endif /* AT_USE_USART */
