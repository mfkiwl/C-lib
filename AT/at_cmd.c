/*
 * at_cmd.c
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

#include "at_cmd.h"
#include "stdio.h"
#include "stdarg.h"
#include "at_config.h"
#include "at_uart.h"
#include "at_udp.h"
#include "at_rtt.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#define AT_PARAM_TIMEOUT        3000

typedef struct cmd_item
{
    const at_fun_t *at_fun;
    struct cmd_item *next;
} at_cmd_list_t;

extern cs_buffer_t usart_buffer;
extern cs_buffer_t udp_buffer;

bool g_echo_flag;
static bool g_at_inited = false;

static TaskHandle_t at_parse_handler = NULL;
static TaskHandle_t at_udp_handler = NULL;
static TaskHandle_t at_rtt_handler = NULL;

cmd_route_t current_cmd_route = CMD_ROUTE_USART;
static cs_mutex_t at_printf_mutex = {NULL, 0, NULL, NULL};

//static cr600_param_t s_cr600_param;
cs_queue_t queue_cmd = {0};
//cr600_param_t *cr600_param_cur = &s_cr600_param;
static char at_printf_buf[512] = {0};
static void task_at_parse(void *unused);

TimerHandle_t   AT_SoftTimerHandle = NULL;

/* cmd list */
static at_fun_t at_fun[] =
{
    {"AT", NULL, NULL, NULL, at_exe_at},
    {"ATE", NULL, NULL, at_setup_echo, NULL},
    {"AT+RST", at_test_rst, at_test_rst, NULL, at_exe_rst},
    {"AT+GMR", at_test_gmr, at_test_gmr, NULL, at_exe_gmr},
    {"AT+VERSION", at_test_gmr, at_test_gmr, NULL, at_exe_gmr},
    {"AT+RESTORE", at_test_restore, at_test_restore, NULL, at_exe_restore},
    {"AT+CONFIG", at_test_config, at_test_config, NULL, at_exe_config},
    {"AT+CMDLIST", at_test_cmdlist, at_test_cmdlist, NULL, at_exe_cmdlist},
#ifdef AT_USE_USART
//    {"AT+UART_CUR", at_test_uart_cur, at_query_uart_cur, at_setup_uart_cur, NULL},
    {"AT+UART_DEF", at_test_uart_def, at_query_uart_def, at_setup_uart_def, NULL},
#endif /* AT_USE_USART */

    {"AT+IFCONFIG", at_test_ifconfig, at_test_ifconfig, NULL, at_exe_ifconfig},
    {"AT+NETSTAT", at_test_netstat, at_exe_netstat, NULL, at_exe_netstat},
//    {"AT+CWDHCP_CUR", at_test_dhcp_cur, at_query_dhcp_cur, at_setup_dhcp_cur, NULL},
    {"AT+CWDHCP_DEF", at_test_dhcp_def, at_query_dhcp_def, at_setup_dhcp_def, NULL},
//    {"AT+CIPSTAMAC_CUR", at_test_mac_cur, at_query_mac_cur, at_setup_mac_cur, NULL},
    {"AT+CIPSTAMAC_DEF", at_test_mac_def, at_query_mac_def, at_setup_mac_def, NULL},

//    {"AT+CIPSTA_CUR", at_test_ip_cur, at_query_ip_cur, at_setup_ip_cur, NULL},
    {"AT+CIPSTA_DEF", at_test_ip_def, at_query_ip_def, at_setup_ip_def, NULL},

//    {"AT+CIPSTADNS_CUR", at_test_dns_cur, at_query_dns_cur, at_setup_dns_cur, NULL},
    {"AT+CIPSTADNS_DEF", at_test_dns_def, at_query_dns_def, at_setup_dns_def, NULL},

    {"AT+CIPSTATUS", NULL, at_query_ipstatus, NULL, NULL},
    {"AT+CIPDOMAIN", at_test_domain, at_test_domain, at_setup_domain, NULL},
    {"AT+CIPSTART", NULL, NULL, at_setup_start, NULL},
    {"AT+ARP", at_test_arp, at_query_arp, at_setup_arp, at_query_arp},
    //{"AT+CIPSSLSIZE",NULL,NULL,NULL,NULL},
    {"AT+CIPSEND", NULL, NULL, at_setup_send, NULL},
    {"AT+CIPSENDEX", NULL, NULL, at_setup_sendex, NULL},
    {"AT+CIPSENDBUF", NULL, NULL, at_setup_sendbuf, NULL},
    {"AT+CIPBUFRESET", NULL, NULL, at_setup_bufreset, NULL},
    {"AT+CIPBUFSTATUS", NULL, at_query_bufstatus, NULL, NULL},
    {"AT+CIPCHECKSEQ", NULL, at_query_ckeckseq, NULL, NULL},
    {"AT+CIPCLOSE", NULL, NULL, at_setup_close, NULL},
    {"AT+CIFSR", at_test_ifsr, at_query_ifsr, NULL, at_query_ifsr},
    {"AT+CIPMUX", NULL, NULL, at_setup_ipmux, NULL},
    {"AT+CIPSERVER", NULL, NULL, at_setup_server, NULL},
    {"AT+CIPMODE", NULL, NULL, at_setup_mode, NULL},
    {"AT+SAVETRANSLINK", NULL, NULL, at_setup_savetranslink, NULL},
    {"AT+CIPSTO", NULL, NULL, at_setup_sto, NULL},
    {"AT+PING", at_test_ping, at_test_ping, at_setup_ping, NULL},
    {"AT+CIPDINFO", NULL, NULL, at_setup_ipdinfo, NULL},
    {"AT+LWIPSTAT", at_test_lwipstats, at_query_lwipstats, NULL, at_query_lwipstats},

    {"AT+OSRUNTIME", at_test_osruntime, at_query_osruntime, NULL, at_query_osruntime},
    {"AT+OSTASKLIST", at_test_ostasklist, at_query_ostasklist, NULL, at_query_ostasklist},
    {"AT+OSHEAP", at_test_heap, at_query_heap, NULL, at_query_heap},

    {"AT+ETHTRANSPSWITCH", NULL, at_query_eth_transpswitch, at_setup_eth_transpswitch, NULL},
    {"AT+PLC", at_test_plc, NULL, at_setup_plc, NULL},
    {"AT+PLCCMDCHK", at_test_plc_cmd_check, at_query_plc_cmd_check, at_setup_plc_cmd_check, NULL},
    {"AT+PLCLINKSEND", at_exe_plc_linksend, NULL, NULL, at_exe_plc_linksend},
    {"AT+PLCTML", at_test_plc_tml, at_query_plc_tml, NULL, at_query_plc_tml},
#ifdef CR600_ADHOC_MODE
    {"AT+PLCPHYSWITCH", NULL, at_query_plc_physwitch, at_setup_plc_physwitch, NULL},
    {"AT+PLCHOPSWITCH", NULL, at_query_plc_hopswitch, at_setup_plc_hopswitch, NULL},
    {"AT+PLCTRANSPSWITCH", NULL, at_query_plc_transpswitch, at_setup_plc_transpswitch, NULL},
    {"AT+PLCPHYSEND", NULL, NULL, at_setup_plc_physend, NULL},
    {"AT+PLCHOPSEND", NULL, NULL, at_setup_plc_hopsend, NULL},
    {"AT+PLCHOPDEBUG", NULL, NULL, at_setup_plc_hopdebug, at_query_plc_hopdebug},
    {"AT+PLCHOPROLE", NULL, NULL, at_setup_plc_hoprole, at_query_plc_hoprole},
    {"AT+PLCHOPLIST", NULL, NULL, NULL, at_query_plc_flooding_opt_list},
#endif /* CR600_ADHOC_MODE */
    {"AT+PLCSTAT", at_test_plc_stat, at_exe_plc_stat, at_setup_plc_stat, at_exe_plc_stat},
    {"AT+MEMW", at_test_mem32, at_test_mem32, at_setup_mem32, NULL},
    {"AT+MEMH", at_test_mem16, at_test_mem16, at_setup_mem16, NULL},
    {"AT+MEMB", at_test_mem8, at_test_mem8, at_setup_mem8, NULL},
    {"AT+WW", at_test_w4, at_test_w4, at_setup_w4, NULL},
    {"AT+WH", at_test_w2, at_test_w2, at_setup_w2, NULL},
    {"AT+WB", at_test_w1, at_test_w1, at_setup_w1, NULL},
    {"AT+PARAM", at_test_param, at_test_param, at_setup_param, NULL},
//    {"AT+UPDATE_CUR", at_test_update_cur, at_query_update_cur, at_setup_update_cur, at_exe_update_cur},
    {"AT+UPDATE_DEF", at_test_update_def, at_query_update_def, at_setup_update_def, at_exe_update_def},
    {"AT+UPDATE_SERIAL", at_test_update_serial, NULL, at_setup_update_serial, NULL},
    {"AT+LEDBEACON", at_test_led_beacon, at_query_led_beacon, at_setup_led_beacon, NULL}
};

static at_cmd_list_t cmd_list = {&at_fun[0], NULL};
extern void AT_SoftTimeout(void *Param);

/*!
 * @brief at module printf function,route to usart or udp
 */
int at_printf(const char *fmt, ...)
{
    AT_ASSERT(at_printf_mutex.mutex);

    at_printf_mutex.take(&at_printf_mutex);
    {
        va_list args;
        va_start(args, fmt);
        vsprintf(at_printf_buf, fmt, args);
        va_end(args);
    }

    switch (current_cmd_route)
    {
    case CMD_ROUTE_USART:
        os_usart_send_block((uint8_t *)at_printf_buf, strlen(at_printf_buf));
        break;
#ifdef AT_USE_UDP
    case CMD_ROUTE_UDP:
        udp_send_block((uint8_t *)at_printf_buf, strlen(at_printf_buf));
        break;
#endif
    case CMD_ROUTE_RTT:
        rtt_send_block((uint8_t *)at_printf_buf, strlen(at_printf_buf));
        break;
    default:
        os_printf("at_printf@can not find cmd_route:%d\r\n", current_cmd_route);
        break;
    }
    at_printf_mutex.give(&at_printf_mutex);
    return 1;
}

/*!
 * @brief at module send block function,route to usart or udp
 */
void at_send_block(const uint8_t *data, uint32_t len)
{
    AT_ASSERT(at_printf_mutex.mutex);

    at_printf_mutex.take(&at_printf_mutex);
    {
        switch (current_cmd_route)
        {
        case CMD_ROUTE_USART:
            os_usart_send_block(data, len);
            break;
#ifdef AT_USE_UDP
        case CMD_ROUTE_UDP:
            udp_send_block(data, len);
            break;
#endif
        default:
            os_printf("at_printf@can not find cmd_route:%d\r\n", current_cmd_route);
            break;
        }
    }
    at_printf_mutex.give(&at_printf_mutex);
}

/*!
 * @brief display all the cmd list registered
 */
void at_cmdlist_display(void)
{
    at_cmd_list_t *p_cmd_list = &cmd_list;

    for (p_cmd_list = &cmd_list; p_cmd_list != NULL; p_cmd_list = p_cmd_list->next)
    {
        at_printf((char *)p_cmd_list->at_fun->at_cmd_name);
        at_printf(":\r\n");
        if (p_cmd_list->at_fun->at_test_cmd)
        {
            at_printf("*******************\r\n");
            p_cmd_list->at_fun->at_test_cmd();
            at_printf("*******************\r\n");
        }
    }
}

/*!
 * @brief register a cmd to at module
 */
uint8_t at_register(at_fun_t *fun)
{
    at_cmd_list_t *new_item = NULL;
    at_cmd_list_t *lp_cmd_list = &cmd_list;
    static at_cmd_list_t *p_cmd_list = &cmd_list;

    if (fun == NULL)
    {
        at_printf("at_register failed,fun can not be null\r\n");
        return 0;
    }

    for (lp_cmd_list = &cmd_list; lp_cmd_list != NULL; lp_cmd_list = p_cmd_list->next)
    {
        if (strcmp((char *)lp_cmd_list->at_fun->at_cmd_name, (char *)fun->at_cmd_name) == 0)
        {
            at_printf("command: is already in the cmd_list\r\n", fun->at_cmd_name);
            return 0;
        }
    }
    new_item = (at_cmd_list_t *)pvPortMalloc(sizeof(at_cmd_list_t));

    if (new_item == NULL)
    {
        at_printf("malloc failed\r\n");
        return 0;
    }

    vPortEnterCritical();

    new_item->at_fun = fun;
    new_item->next = NULL;
    p_cmd_list->next = new_item;
    p_cmd_list = new_item;

    vPortExitCritical();

    return 1;
}

/*!
 * @brief register cmd array to at module
 */
uint8_t at_register_array(at_fun_t *fun, uint32_t array_size)
{
    at_fun_t *p_fun = fun;
    for (uint32_t i = 0; i < array_size; i++, p_fun++)
    {
        if (!at_register(p_fun))
        {
            /* register failed */
            return 0;
        }
    }
    return 1;
}

static uint8_t at_getlen(uint8_t *cmd)
{
    uint8_t n, i;

    n = 0;
    i = 128;

    while (i-- && (*cmd))
    {
        if ((*cmd == '\r') || (*cmd == '=') || (*cmd == '?') || ((*cmd >= '0') && (*cmd <= '9')))
        {
            return n;
        }
        else
        {
            cmd++;
            n++;
        }
    }
    return 0;
}

void at_print_ok(void)
{
    at_printf("\r\nOK\r\n");
}

void at_print_error(void)
{
    at_printf("\r\nERROR\r\n");
}

void at_print_error_param(void)
{
    at_printf("\r\nparameter error\r\n");
}

/*!
 * @brief init at module
 */
void at_init(void)
{
    /* can not init at module twice */
    if (g_at_inited)
    {
        at_printf("at module inited\r\n");
        return;
    }
    g_at_inited = true;
    g_echo_flag = true;
    
    mutex_init(&at_printf_mutex);
    queue_init(&queue_cmd, AT_RECV_QUEUE_LEN, sizeof(at_queue_msg_t));
    at_register_array(&at_fun[1], sizeof(at_fun) / sizeof(at_fun[0]) - 1);

    /* create at soft timer that will call AT_SoftTimeout after 3000ms */
    AT_SoftTimerHandle = xTimerCreate( "AT_SoftTimer", AT_PARAM_TIMEOUT/portTICK_RATE_MS, pdFALSE, (void *)0, AT_SoftTimeout);
    if(AT_SoftTimerHandle == NULL)
    {
        at_print_error();
        at_printf("create at soft timer error\r\n");
    }
    
#ifdef AT_USE_USART
    /* init uart according cr600 param */
    at_uart_init(cr600_param);
#endif

    /* create task to process at commands */
    AT_ASSERT(xTaskCreate(task_at_parse, (char const *)"AT_ParseTask", 512, NULL, AT_PRI_TASK_MAIN, &at_parse_handler) == pdPASS);

#ifdef AT_USE_UDP
    /* create task to recv&send data from/to UDP */
    AT_ASSERT(xTaskCreate(task_cmd_udp_recv, (char const *)"AT_UDPTask", 512, NULL, AT_PRI_TASK_UDP, &at_udp_handler) == pdPASS);
#endif
    AT_ASSERT(xTaskCreate(task_cmd_rtt_recv, (char const *)"AT_RTTTask", 512, NULL, AT_PRI_TASK_RTT, &at_rtt_handler) == pdPASS);
    
    printf("create AT task OK.\r\n");
}

/*!
 * @brief process a cmd string
 */
static void at_process(uint8_t *data)
{
    at_cmd_list_t *p_cmd_list = NULL;
    uint8_t fun_choose = 0;
    uint8_t *p_data = data;
    uint32_t str_len = 0;
    uint8_t cmd_len = 0;

    cmd_len = at_getlen(data);

    if (!cmd_len)
    {
        at_print_error();
        return;
    }

    p_data += cmd_len;
    if (*p_data == '\r')
        fun_choose = 0;
    else if (*p_data == '?' && (p_data[1] == '\r'))
        fun_choose = 1;
    else if ((*p_data == '=') && (p_data[1] == '?') && (p_data[2] == '\r'))
        fun_choose = 2;
    else if ((*p_data >= '0') && (*p_data <= '9') || (*p_data == '='))
        fun_choose = 3;
    else
    {
        at_print_error();
        return;
    }

    if (g_echo_flag)
    {
        if(!strchr((char *)data, '%'))
            at_printf((char *)(data));
    }
    for (p_cmd_list = &cmd_list; p_cmd_list != NULL; p_cmd_list = p_cmd_list->next)
    {
        str_len = strlen((char const *)p_cmd_list->at_fun->at_cmd_name);
        if (cmd_len == str_len)
        {
            if (memcmp(data, p_cmd_list->at_fun->at_cmd_name, cmd_len) == 0)
            {
                switch (fun_choose)
                {
                case 0:
                    if (p_cmd_list->at_fun->at_exe_cmd)
                        p_cmd_list->at_fun->at_exe_cmd();
                    else
                        at_print_error();
                    break;
                case 1:
                    if (p_cmd_list->at_fun->at_query_cmd)
                        p_cmd_list->at_fun->at_query_cmd();
                    else
                        at_print_error();
                    break;
                case 2:
                    if (p_cmd_list->at_fun->at_test_cmd)
                        p_cmd_list->at_fun->at_test_cmd();
                    else
                        at_print_error();
                    break;
                case 3:
                    if (p_cmd_list->at_fun->at_setup_cmd)
                        p_cmd_list->at_fun->at_setup_cmd(p_data);
                    else
                        at_print_error();
                    break;
                default:
                    break;
                }
                return;
            }
        }
    }
    at_print_error();
}

static void task_at_parse(void *unused)
{
    at_queue_msg_t msg;
    cs_buffer_t *pbuf;
    
    uint32_t crc32 = 0, crc32_cal = 0;
    for (;;)
    {
        if (queue_cmd.rcv(&queue_cmd, (void *)&msg) == AT_TRUE)
        {
            /* when in param update mode, data is "THIS IS PARAM UPDATE" */
            if(strcmp((char const *)msg.cmd_data, at_param_prompt))
            {
                current_cmd_route = msg.cmd_route;
                at_process(msg.cmd_data);
            }
            else
            {
                taskENTER_CRITICAL();
                if(msg.cmd_route == CMD_ROUTE_USART)
                {
                    pbuf = &usart_buffer;
                }
                else if(msg.cmd_route == CMD_ROUTE_UDP)
                {
                    pbuf = &udp_buffer;
                }
                else
                {
                    at_print_error();
                    at_printf("task_at_parse@route error\r\n");
                    at_param_update = at_param_size = at_param_addr = 0;
                    continue;
                }
                
                memcpy(&crc32, (uint8_t *)pbuf->buffer + at_param_size + 8, 4);
                /* crc32 check */
                crc32_cal = cr600_crc32_block(pbuf->buffer, at_param_size + 8);
                if(crc32 != crc32_cal)
                {
                    at_print_error();
                    at_printf("task_at_parse@crc32 error\r\n");
                    pbuf->read_index = pbuf->save_index = 0;
                    at_param_update = at_param_size = at_param_addr = 0;
                    continue;
                }
                memcpy((uint8_t *)cr600_param + at_param_addr - CR600_PARA_ADDR, pbuf->buffer + 8, at_param_size);
                cr600_write_sys_param();
                
                pbuf->read_index = pbuf->save_index = 0;
                at_param_update = at_param_size = at_param_addr = 0;
                
                taskEXIT_CRITICAL();
                
                at_print_ok();
            }
        }
    }
    /* can not get here */
    //vTaskDelete( NULL );
}
