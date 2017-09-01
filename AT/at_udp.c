/*
 * at_udp.c
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

#include "at_udp.h"
#include "lwip/sockets.h"
#include "lwip/dhcp.h"

#include "freertos\FreeRTOS.h"
#include "freertos\semphr.h"
#include "freertos\task.h"

extern cs_queue_t queue_cmd;

#ifdef AT_USE_UDP
cs_buffer_t udp_buffer;

static void process_udp_buffer(uint8_t *buf, uint32_t len)
{
    uint8_t *read_idx = buf;
    uint8_t *save_idx = buf;

    at_queue_msg_t msg = {CMD_ROUTE_UDP, {0}};

    if (at_param_update == 1)
    {
        if(*(uint32_t *)buf == at_param_addr && *((uint32_t *)(buf + 4)) == at_param_size)
        {
            memset(&msg.cmd_data, 0, AT_CMD_MAX_LEN);
            strcpy((char *)msg.cmd_data, at_param_prompt);
            queue_cmd.snd(&queue_cmd, (void *)&msg);
            return;
        }
        else
        {
            at_param_update = at_param_addr = at_param_size = 0;
            at_print_error();
            return;
        }
    }
    while (read_idx < buf + len)
    {
        if (*read_idx == '\n')
        {
            memset(&msg.cmd_data, 0, AT_CMD_MAX_LEN);
            memcpy(&msg.cmd_data, save_idx, read_idx - save_idx + 1);
            
            /* some tcp/udp test tools just send '\n' as enter flag, not '\r\n'. */
            if((read_idx - save_idx > 1) && msg.cmd_data[read_idx - save_idx - 1] != '\r')
                msg.cmd_data[read_idx - save_idx] = '\r';
            
            queue_cmd.snd(&queue_cmd, (void *)&msg);

            save_idx = read_idx + 1;
        }
        read_idx++;
    }
}

static int32_t socket_fd = 0;
static struct sockaddr_in client_addr;

void udp_send_block(const uint8_t *data, uint32_t len)
{
    sendto(socket_fd, (const char *)data, len, 0, (const struct sockaddr *) &client_addr, sizeof(struct sockaddr_in));
}

extern struct netif *netif_default;

void task_cmd_udp_recv(void *unused)
{
    int32_t err_code = 0;
    int32_t sin_len = sizeof(struct sockaddr_in);
    int opt = 1;
    
    struct sockaddr_in server_addr;

    /* lwip not init */
    if (netif_default == NULL)
    {
        at_printf("AT UDP has not detect LWIP, LWIP not init?  Now you can only use AT with USART2.\r\n");
        vTaskDelete(NULL);
        return;
    }
    /* Clear the address */
    memset(&server_addr, 0x00, sizeof(server_addr));

    /* Fill in the address form */
    server_addr.sin_len = sizeof(server_addr);
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = PP_HTONS(AT_CONFIG_UDP_PORT);
    server_addr.sin_addr.s_addr = PP_HTONL(INADDR_ANY);

    memset(&udp_buffer, 0, sizeof(cs_buffer_t));
    while (1)
    {
        /* Create the socket */
        socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_fd < 0)
        {
            /* Create socket failed */
            at_printf("at udp create socket failed,errno:%d\r\n", lwip_strerror(errno));
            vTaskDelete(NULL);
            return;
        }

        /* set socket to broadcast mode */
        setsockopt(socket_fd, SOL_SOCKET, SO_BROADCAST, (char *)&opt, sizeof(opt));  
    
        /* Bind the socket */
        err_code = bind(socket_fd, (struct sockaddr *)&server_addr, sizeof(server_addr));
        if (err_code < 0)
        {
            /* Connection failed & release the resource */
            at_printf("bind failed,errno:%s\r\n", lwip_strerror(errno));
            close(socket_fd);
            vTaskDelete(NULL);
            return;
        }

        at_printf("AT udp server start\r\n");

        while (1)
        {
            if (at_param_update != 1)
                memset(&udp_buffer.buffer, 0, AT_BUFFER_SIZE);
            /* Wait for incoming data on the opened socket. */
            err_code = recvfrom(socket_fd, &udp_buffer.buffer, AT_BUFFER_SIZE, 0, (struct sockaddr *)&client_addr, (socklen_t *)&sin_len);

            if (err_code < 0)
            {
                at_printf("send error:%s....now recreate socket\r\n", lwip_strerror(errno));
                close(socket_fd);
                break;
            }
            else
            {
                process_udp_buffer((uint8_t *)&udp_buffer.buffer, err_code);
            }
            vTaskDelay(1);
        }
    }
}

#endif
