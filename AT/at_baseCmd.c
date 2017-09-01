/*
 * at_baseCmd.c
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

#include <string.h>
#include <stdio.h>

#include "clouder600.h"
#include "clouder600_plc.h"

#include "at.h"
#include "at_baseCmd.h"
#include "at_config.h"
#include "at_uart.h"

#include "FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "FreeRTOS/timers.h"

#include "cr600_tools.h"
#include "cr600_partition.h"

#include "lwip/inet.h"

#include "KSZ8041.h"

extern void at_uart_init(cr600_param_t *config);
extern void at_cmdlist_display(void);

extern bool g_echo_flag;

/* default cr600 parameter */
extern cr600_param_t cr600_param_default;

/* cr600 parameter pointer */
extern cr600_param_t *cr600_param;

typedef struct
{
    int8_t flag;
    char reserve[3];
} update_flag_t;

uint32_t at_param_size = 0, at_param_addr = 0, at_param_update = 0;
const char at_param_prompt[] = "THIS IS PARAM UPDATE";
const char at_param_waitting[] = "WAITTING FOR BINARY...\r\n";
void at_exe_at(void)
{
    at_printf("\r\nAT\r\n");
}

void at_setup_echo(uint8_t *pPara)
{
    //  at_printf("%c\n",*pPara);
    if (*pPara == '0')
    {
        g_echo_flag = false;
    }
    else if (*pPara == '1')
    {
        g_echo_flag = true;
    }
    else
    {
        at_print_error_param();
        return;
    }
    at_print_ok();
}

void at_test_rst(void)
{
    at_printf("info. reboot module\r\n");
}
void at_exe_rst(void)
{
    at_print_ok();
    at_printf("rebooting...\r\n");
    cr600_reset();
}

void at_test_gmr(void)
{
    at_printf("info. get firmware version information\r\n");
}

void at_exe_gmr(void)
{
    char boot_version[5] = {0};
    char boot_date[21] = {0};
    
    memcpy(boot_version, (const char *)0x200000CCUL, 4);
    memcpy(boot_date, (const char *)0x200000D4UL, 20);
    at_printf("bootloader version:  %s\r\n", boot_version);
    at_printf("bootloader date:     %s\r\n", boot_date);
    
    at_printf("firmware version:    %s\r\n", firmware_version);
    at_printf("firmware date:       %s\r\n", firmware_date);

    at_printf("library version:     %s\r\n", PLC_GetLibVersion());
    at_printf("ucore version:       %s\r\n", PLC_GetUcoreVersion());
    at_printf("library compiled @   %s\r\n", PLC_GetCompileTime());
}

void at_test_restore(void)
{
    at_printf("info. restore to the factory setting\r\n");
}

void at_exe_restore(void)
{
    at_print_ok();
    cr600_write_default_param();
    //cr600_echo_sys_param();

    at_printf("now restart system\r\n");
    NVIC_SystemReset();
}

void at_test_config(void)
{
    at_printf("info. display all the setting store in flash\r\n");
}

void at_exe_config(void)
{
    /* update sys_param first, plc will change cr600_param */
    cr600_update_sys_param_checksum();
    
    /* send sys_param data to usart/udp */
    at_send_block((uint8_t *)cr600_param, sizeof(cr600_param_t));
}

#ifdef AT_USE_USART
static uint8_t setup_uart(uint8_t *pPara, cr600_param_t *config)
{
    uint32_t baudrate;
    uint32_t databits;
    uint32_t stopbits;
    uint32_t parity;
    uint32_t flowcontrol;

    uint8_t ret = sscanf((char *)pPara, "=%d,%d,%d,%d,%d\r\n",
                         &baudrate,
                         &databits,
                         &stopbits,
                         &parity,
                         &flowcontrol);

    if (ret != 5)
        at_print_error_param();
    else
    {
        config->uart.baudrate = baudrate;
        config->uart.databits = databits;
        config->uart.stopbits = stopbits;
        config->uart.parity = parity;
        config->uart.flowcontrol = flowcontrol;

        //cr600_display_sys_param();
        at_print_ok();

        return AT_TRUE;
    }
    return AT_FALSE;
}

void at_setup_uart_def(uint8_t *pPara)
{
    if (setup_uart(pPara, cr600_param) == AT_TRUE)
    {
        cr600_write_sys_param();
        at_uart_init(cr600_param);
    }
}

void at_test_uart_cur(void)
{
    at_printf("info. AT+UART_CUR=<baudrate>,<databits>,<stopbits>,<parity>,<flow control>\r\n");
    at_printf("exp. AT+UART_CUR=115200,8,1,0,0\r\n");
}

void at_test_uart_def(void)
{
    at_printf("info. AT+UART_DEF=<baudrate>,<databits>,<stopbits>,<parity>,<flow control>\r\n");
    at_printf("exp. AT+UART_DEF=115200,8,1,0,0\r\n");
}

void at_query_uart_def(void)
{
    at_printf("AT+UART_DEF=%d,%d,%d,%d,%d\r\n",
              cr600_param->uart.baudrate,
              cr600_param->uart.databits,
              cr600_param->uart.stopbits,
              cr600_param->uart.parity,
              cr600_param->uart.flowcontrol);
}

#endif /* AT_USE_USART */

void at_test_cmdlist(void)
{
    at_printf("info. list all the register commands\r\n");
}
void at_exe_cmdlist(void)
{
    at_print_ok();
    at_cmdlist_display();
}


void at_test_osruntime(void)
{
    at_printf("info. AT+OSRUNTIME=? echo freertos runtime state\r\n");
}

void at_query_osruntime(void)
{
    //os_echo_runtime_state();
#if (configGENERATE_RUN_TIME_STATS == 1 && configUSE_STATS_FORMATTING_FUNCTIONS == 1)
    char *task_status_buf = NULL;

    task_status_buf = (char *)pvPortMalloc(20 * 40);
    if (task_status_buf == NULL)
    {
        at_printf("at_query_osruntime@pvPortMalloc error\r\n");
        return;
    }
    memset(task_status_buf, 0, 40 * 20);

    int no_task = uxTaskGetNumberOfTasks();
    if (no_task >= 20)
    {
        at_printf("current task number is %d, max for array is %d\r\n", no_task, 20);
        return;
    }

    vTaskGetRunTimeStats(task_status_buf);

    at_printf("*************************************\r\n");
    at_printf(task_status_buf);
    at_printf("*************************************\r\n\r\n");

    vPortFree(task_status_buf);
#else
    at_printf("please open macro:configGENERATE_RUN_TIME_STATS & configUSE_STATS_FORMATTING_FUNCTIONS\r\n");
#endif /* (configGENERATE_RUN_TIME_STATS == 1 && configUSE_STATS_FORMATTING_FUNCTIONS == 1) */
}

void at_test_ostasklist(void)
{
    at_printf("info. AT+OSTASKLIST=? echo freertos task status\r\n");
}

void at_query_ostasklist(void)
{
#if (configUSE_TRACE_FACILITY == 1 && configUSE_STATS_FORMATTING_FUNCTIONS > 0)
    char *task_status_buf = NULL;

    task_status_buf = (char *)pvPortMalloc(20 * 40);
    if (task_status_buf == NULL)
    {
        at_printf("at_query_ostasklist@pvPortMalloc error\r\n");
        return;
    }
    memset(task_status_buf, 0, 40 * 20);

    printf("*************************************\r\n");
    printf("task      status prio   free_stack num\r\n");
    vTaskList((char *)task_status_buf);
    
    at_send_block(task_status_buf, strlen(task_status_buf));
    //at_printf("%s\r\n", task_status_buf);

#else
    at_printf("please open macro:configUSE_TRACE_FACILITY & configUSE_STATS_FORMATTING_FUNCTIONS\r\n");
#endif /* configUSE_TRACE_FACILITY == 1 && configUSE_STATS_FORMATTING_FUNCTIONS > 0 */
}

void at_test_heap(void)
{
    at_printf("info. AT+OSHEAP=? show freertos heap used\r\n");
}
#if defined(__ICCARM__)
extern int __ICFEDIT_size_heap__;
extern int __ICFEDIT_size_cstack__;
#elif defined(__CC_ARM)
//extern unsigned int __current_sp(void);
#else
		#error "not supported compiler"
#endif
void at_query_heap(void)
{
#if defined(__ICCARM__)
    uint32_t stack = (uint32_t)&__ICFEDIT_size_cstack__;
    uint32_t heap = (uint32_t)&__ICFEDIT_size_heap__;
#elif defined(__CC_ARM)
		//uint32_t stack = __current_sp();
		uint32_t stack = 0;
		uint32_t heap = 0;
#else
		#error "not supported compiler"
#endif
    uint32_t used = os_get_heap_used();
    uint32_t hwm  = os_get_heap_high_watermark();
    double per = (double)(heap - used) / heap;

    at_printf("stack all:           %d(bytes)\r\n", stack);
    at_printf("heap  all:           %d(bytes)\r\n", heap);
    at_printf("heap used:           %d(bytes)\r\n", used);
    at_printf("heap free:           %d(bytes) %.2f%\r\n", heap - used, per * 100);
    at_printf("heap high watermark: %d(bytes)\r\n", hwm);

    return;
}

void at_test_plc(void)
{
    at_printf("info. AT+PLC=<plc cmd> send <plc cmd> to plc module\r\n");
}

void at_setup_plc(uint8_t *pPara)
{
    ErrorStatus status;

    pPara++;
    status = PLC_CmdProc(PLC_CMDTYPE_AT, pPara, strlen((char const *)pPara) - 1);
    if(status == ERROR)
        at_printf("PLC_CmdProc error");
}

void at_test_plc_cmd_check(void)
{
    at_printf("info. AT+PLCCMDCHK=<0|1> set checking cmd data in ETH&PLC IRQ\r\n");
    at_printf("exp.  AT+PLCCMDCHK=1\r\n");
}

void at_query_plc_cmd_check(void)
{
    at_printf("AT+PLCCMDCHK=%d\r\n", PLC_GetCheckPlcCommands());
}

void at_exe_plc_linksend(void)
{
    uint8_t data[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x6c, 0x0b, 0x84, 0x08, 0xd9, 0x57, 0x08, 0x06, 0x00, 0x01,
                      0x08, 0x00, 0x06, 0x04, 0x00, 0x01, 0x6c, 0x0b, 0x84, 0x08, 0xd9, 0x57, 0xc0, 0xa8, 0x00, 0xca,
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xa8, 0x00, 0x01
                     };
    PLC_LinkSend(data, sizeof(data));
}

void at_setup_plc_cmd_check(uint8_t *pPara)
{
    uint32_t status = 0;

    uint32_t ret = sscanf((char *)pPara, "=%d", &status);

    if (ret == 1)
    {
        if (status != 0 && status != 1)
        {
            at_printf("parameter must be 0 or 1\r\n");
            at_print_error();
            return;
        }
        PLC_SetCheckPlcCommands((FunctionalState)status);
        at_print_ok();
        return;
    }
    at_print_error();
}

void at_test_plc_tml(void)
{
    at_printf("info. AT+PLCTML show plc tml(terminal manage list)\r\n");
}
void at_query_plc_tml(void)
{
    char *tml_buf = NULL;
    uint8_t empty_cnt = 0;
    
    tml_buf = (char *)pvPortMalloc(20 * 40);
    if (tml_buf == NULL)
    {
        at_printf("at_query_plc_tml@pvPortMalloc error\r\n");
        return;
    }
    
    memset(tml_buf, 0, 40 * 20);

    for(uint8_t i = 0; i < 20; i ++)
    {
        if(PLC_GetTml(i, tml_buf) == SUCCESS)
        {
            at_send_block((uint8_t *)tml_buf, strlen(tml_buf));
        }
        else{
            empty_cnt++;
        }
    }
    
    if(empty_cnt == 20)
        at_printf("PLC TML(terminal manage table) is empty\r\n");
    
    vPortFree(tml_buf);
}

#define AT_CR600_STATISTICS_PRINT(stat, padding)    do{ at_printf(#stat""padding"%8d %12ld %4d %8d\r\n", \
                                                    sta->stat.pkg, \
                                                    sta->stat.size, \
                                                    sta->stat.pkgSpeed, \
                                                    sta->stat.sizeSpeed<<3); \
                                        }while(0)

#define AT_UCORE_STATISTICS_PRINT(stat, padding)    do{ at_printf(#stat""padding"%8d            - %4ld\r\n", \
                                                    sta->stat.pkg, \
                                                    sta->stat.pkgSpeed); \
                                        }while(0)
void at_test_plc_stat(void)
{
    at_printf("info. AT+PLCSTAT show plc&eth statistics\r\n");
    at_printf("info. AT+PLCSTAT=1 clear plc&eth statistics\r\n");
}

void at_setup_plc_stat(uint8_t *pPara)
{
    uint32_t state = 0;
    uint32_t ret = 0;

    ret = sscanf((char *)pPara, "=%d", &state);

    if (ret == 1)
    {
        if (state == 1)
        {
            CR600_StatisticsClr();
            at_print_ok();
        }
        //at_printf("please enter AT+PLCSTAT=1 to clear staticstics\r\n");
    }
    else
    {
        at_print_error();
    }
}

void at_exe_plc_stat(void)
{
    CR600_Statistics_T *sta = (CR600_Statistics_T *)pvPortMalloc(sizeof(CR600_Statistics_T));
    BUB_Statistics_T *bub = (BUB_Statistics_T *)pvPortMalloc(sizeof(BUB_Statistics_T));
    
    *sta = *CR600_GetStatistics();
    *bub = *BUB_GetStats();
    
    at_printf("/*********************************************\r\n");
    at_printf("  name           pkg  size(bytes) pkg/s  bps\r\n");
    
    AT_CR600_STATISTICS_PRINT(ethTxAll,"    ");
    AT_CR600_STATISTICS_PRINT(ethRxAll,"    ");
//    AT_CR600_STATISTICS_PRINT(ethRxPlc,"    ");
    
    AT_CR600_STATISTICS_PRINT(ethMissFrame,"");
    AT_CR600_STATISTICS_PRINT(ethRxErr,"    ");
    
    AT_CR600_STATISTICS_PRINT(plcTxAll,"    ");
//    AT_CR600_STATISTICS_PRINT(plcTxLink,"   ");
//    AT_CR600_STATISTICS_PRINT(plcTxHop,"    ");
//    AT_CR600_STATISTICS_PRINT(plcTxPhy,"    ");
    AT_CR600_STATISTICS_PRINT(plcTxCnf,"    ");
    
    AT_CR600_STATISTICS_PRINT(plcRxAll,"    ");
    AT_CR600_STATISTICS_PRINT(bubTxAll,"    ");
    AT_CR600_STATISTICS_PRINT(bubTxCnf,"    ");
    AT_CR600_STATISTICS_PRINT(bubTxNoBuf,"  ");
    AT_CR600_STATISTICS_PRINT(bubTxDropAll,"");
    AT_CR600_STATISTICS_PRINT(bubTxDropAge,"");
    AT_CR600_STATISTICS_PRINT(bubTxErr,"    ");
//    AT_CR600_STATISTICS_PRINT(plcRxEth,"    ");
//    AT_CR600_STATISTICS_PRINT(plcRxMng,"    ");
    
//    AT_UCORE_STATISTICS_PRINT(ucFC,"        ");
    AT_UCORE_STATISTICS_PRINT(ucTx,"        ");
    AT_UCORE_STATISTICS_PRINT(ucTxErr,"     ");
    AT_UCORE_STATISTICS_PRINT(ucTxDrop,"    ");
    AT_UCORE_STATISTICS_PRINT(ucRx,"        ");
    AT_UCORE_STATISTICS_PRINT(ucRxErr,"     ");
    AT_UCORE_STATISTICS_PRINT(ucRxSeqErr,"  ");
    AT_UCORE_STATISTICS_PRINT(ucRxDup,"     ");
    AT_UCORE_STATISTICS_PRINT(ucRxHdrErr,"  ");
    AT_UCORE_STATISTICS_PRINT(ucRxNoBuf,"   ");
    AT_UCORE_STATISTICS_PRINT(ucRxBdyErr,"  ");
    AT_UCORE_STATISTICS_PRINT(ucRxChkErr,"  ");
    at_printf("phyRxErr           %d\r\n", KSZ8041_GetRxErrCounter());
    at_printf("****************BUB statistics****************/\r\n");
    at_printf("curr: %02d %02d %02d %02d %02d %02d %02d %02d\r\n", bub->priCurNum[0], bub->priCurNum[1], 
              bub->priCurNum[2], bub->priCurNum[3], bub->priCurNum[4], bub->priCurNum[5], bub->priCurNum[6], bub->priCurNum[7]);
    at_printf("curr cnt: %ld %ld %ld %ld %ld %ld %ld %ld\r\n", bub->priCurCnt[0], bub->priCurCnt[1], 
              bub->priCurCnt[2], bub->priCurCnt[3], bub->priCurCnt[4], bub->priCurCnt[5], bub->priCurCnt[6], bub->priCurCnt[7]);
    at_printf("drop: %ld %ld %ld %ld %ld %ld %ld %ld\r\n", bub->priDropCnt[0], bub->priDropCnt[1], 
              bub->priDropCnt[2], bub->priDropCnt[3], bub->priDropCnt[4], bub->priDropCnt[5], bub->priDropCnt[6], bub->priDropCnt[7]);
    at_printf("**********************************************/\r\n");
    
    vPortFree(sta);
    vPortFree(bub);
}

extern FunctionalState g_ETHTranspSwitch;
void at_query_eth_transpswitch(void)
{
    at_printf("AT+ETHTRANSPSWITCH=%d\r\n", g_ETHTranspSwitch == ENABLE ? 1 : 0);
}

void at_setup_eth_transpswitch(uint8_t *pPara)
{
    uint32_t state = 0;

    uint32_t ret = sscanf((char *)pPara, "=%d", &state);

    if (ret == 1)
    {
        if (state == 0 || state == 1)
        {
            g_ETHTranspSwitch = state == 1 ? ENABLE : DISABLE;
            at_print_ok();
        }
    }
    else
        at_print_error_param();
}

#ifdef CR600_ADHOC_MODE

typedef struct
{
    uint8_t     len;                    //if len==0, means this relay is not built or not valid
    uint8_t    	*data;               	
	uint16_t	lifetime;
} at_relayList_T;

void at_query_plc_flooding_opt_list(void)
{
    at_relayList_T *list;
    uint8_t i, j, k;
    uint8_t *data = NULL;
    uint8_t	param[10];
/*--------------------------------	
	param[0] -------- master/slave
	param[1] -------- relay num
	param[2] -------- macAddrLen
	param[3] -------- macAddrBase
	param[4]-[9] ---- macAddrHigh[6] 
--------------------------------*/	
    /* get list pointer & list number */
    PLC_GetRelayList((void *)&list, param);
 
	/* hop mode check */
    if(list == NULL)
    {
        at_printf("not in flooding optimized hop mode\r\n");
        return;
    }
    at_printf("relay list:\r\n");	

    for(i = 0; i < param[1]; i++)
    {
        at_printf("Node %d\r\n",i+1);
        for(j = 0; j < list->len; j++)
        {
            /* get data pointer */
            data = (uint8_t *)(list->data + j * param[2]);
			if(param[3]){					//addrBase not from 0
				for(k = 0; k < param[2]; k++)		//print High from relay
					at_printf("%02X:", data[param[2] - 1 - k]);
				for(k = 0; k < (6 - param[2]); k++){	//print low form macAddrHigh[6]
					at_printf("%02X", param[4 + param[2] + k]);
					if(k != 6 - param[2] - 1)
						at_printf(":");
					else at_printf("%s\r\n", (j == (list->len - 1)) ? (param[0] ? " <-- Destnation" : " <-- Master") : "");
				}
			}
			else{							//addrBase == 0
				for(k = 0; k < (6 - param[2]); k++)			//print High from macAddrHigh[6]
					at_printf("%02X:", param[4 + k]);
				for(k = 0; k < param[2]; k++){				//print low form relay
					at_printf("%02X", data[param[2] - 1 - k]);
					if(k != (param[2] - 1))
						at_printf(":");
					else at_printf("%s\r\n", (j == (list->len - 1)) ? (param[0] ? " <-- Destnation" : " <-- Master") : "");
				}
			}
            
        }
        list++;
    }
}

extern FunctionalState g_PLCHopSwitch;
void at_query_plc_hopswitch(void)
{
    at_printf("AT+PLCHOPSWITCH=%d\r\n", g_PLCHopSwitch == ENABLE ? 1 : 0);
}

void at_setup_plc_hopswitch(uint8_t *pPara)
{
    uint32_t state = 0;
    uint32_t ret = 0;

    ret = sscanf((char *)pPara, "=%d", &state);

    if (ret == 1)
    {
        if (state == 0 || state == 1)
        {
            g_PLCHopSwitch = state == 1 ? ENABLE : DISABLE;
            at_print_ok();
        }
    }
    else
    {
        at_print_error_param();
    }
}


extern FunctionalState g_PLCPhySwitch;
void at_query_plc_physwitch(void)
{
    at_printf("AT+PLCPHYSWITCH=%d\r\n", g_PLCPhySwitch == ENABLE ? 1 : 0);
}

void at_setup_plc_physwitch(uint8_t *pPara)
{
    uint32_t state = 0;

    uint32_t ret = sscanf((char *)pPara, "=%d", &state);

    if (ret == 1)
    {
        if (state == 0 || state == 1)
        {
            g_PLCPhySwitch = state == 1 ? ENABLE : DISABLE;
            at_print_ok();
        }
    }
    else
        at_print_error_param();
}

extern FunctionalState g_PLCTranspSwitch;
void at_query_plc_transpswitch(void)
{
    at_printf("AT+PLCTRANSPSWITCH=%d\r\n", g_PLCTranspSwitch == ENABLE ? 1 : 0);
}

void at_setup_plc_transpswitch(uint8_t *pPara)
{
    uint32_t state = 0;

    uint32_t ret = sscanf((char *)pPara, "=%d", &state);

    if (ret == 1)
    {
        if (state == 0 || state == 1)
        {
            g_PLCTranspSwitch = state == 1 ? ENABLE : DISABLE;
            at_print_ok();
        }
    }
    else
        at_print_error_param();
}

void at_setup_plc_physend(uint8_t *pPara)
{
    uint32_t ret = 0;
    uint32_t i = 0;
    MacAddr dstmac = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint32_t mac[6];
    char data[50] = {0};
    uint32_t len = 0;

    ret = sscanf((char *)pPara, "=\"%x:%x:%x:%x:%x:%x\",\"%s @\",\"%d\"\r\n",
                 &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5],
                 data, &len);
    for (i = 0; i < 6; i++)
    {
        dstmac.mac[i] = mac[i];
    }
    data[len++] = '\0';
    if (ret == 8)
    {
        PLC_PhySend(dstmac, (uint8_t *)data, len);
        at_print_ok();
    }
    else
    {
        at_print_error_param();
    }
}

void at_query_plc_hopdebug(void)
{
    at_printf("AT+PLCHOPDEBUG=%d\r\n", cr600_param->plc.hop.debug);
}

void at_setup_plc_hopdebug(uint8_t *pPara)
{
    uint32_t state = 0;

    uint32_t ret = sscanf((char *)pPara, "=%d", &state);

    if (ret == 1)
    {
        if (state == 0 || state == 1)
        {
            cr600_param->plc.hop.debug = state;
            cr600_write_sys_param();
            PLC_HopDebugConfig(state==1?ENABLE:DISABLE);
            at_print_ok();
        }
        else
            at_print_error_param();
    }
    else
        at_print_error();
}

void at_query_plc_hoprole(void)
{
  at_printf("AT+PLCHOPROLE=%s\r\n", cr600_param->plc.hop.coordinator ? "Master" : "Slave");
}

void at_setup_plc_hoprole(uint8_t *pPara)
{
    uint32_t state = 0;
    uint32_t tmp = cr600_param->plc.hop.coordinator;
    uint32_t ret = sscanf((char *)pPara, "=%d", &state);

    if (ret == 1)
    {
        if (state == 0 || state == 1)
        {
          if(state == tmp){
            at_print_ok();
            printf("no changed, still hold %s\r\n", state ? "Master" : "Slave");
          }
          else{
            /* slave|master : 0|1 */
            cr600_param->plc.hop.coordinator = state;
            cr600_write_sys_param();
            
            at_print_ok();
            printf("Board will be changed from %s to %s, and system reboot right now!\r\n", state ? "Slave" : "Master", state ? "Master" : "Slave");
            cr600_reset();
          }
        }
        else
            at_print_error_param();
    }
    else
        at_print_error();
}

void at_setup_plc_hopsend(uint8_t *pPara)
{
    uint32_t ret = 0;
    uint32_t i = 0;
    MacAddr dstmac = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    uint32_t mac[6];
    char data[50] = {0};
    uint32_t len = 0;

    ret = sscanf((char *)pPara, "=\"%x:%x:%x:%x:%x:%x\",\"%s @\",\"%d\"\r\n",
                 &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5],
                 data, &len);
    for (i = 0; i < 6; i++)
    {
        dstmac.mac[i] = mac[i];
    }
    data[len++] = '\0';
    if (ret == 8)
    {
        PLC_HopSend(dstmac, (uint8_t *)data, len);
        at_print_ok();
    }
    else
    {
        at_print_error_param();
    }
}

#endif /* CR600_ADHOC_MODE */

void at_test_mem32(void)
{
    at_printf("info. get data(4bytes) from address\r\n");
    at_printf("exp. AT+MEMW=0x20001002,16\r\n");
}

void at_test_mem16(void)
{
    at_printf("info. get data(2bytes) from address\r\n");
    at_printf("exp. AT+MEMH=0x20001002,16\r\n");
}

void at_test_mem8(void)
{
    at_printf("info. get data(1bytes) from address\r\n");
    at_printf("exp. AT+MEMB=0x20001002,16\r\n");
}

void at_setup_mem32(uint8_t *pPara)
{
    uint32_t size = 0, i, j;
    uint32_t data = 0;
    uint32_t *addr = NULL;
    uint32_t ret = sscanf((char *)pPara, "=%p,%d", &addr, &size);

    if (ret == 2)
    {
        if (((int)addr > 0x10000000UL && (int)addr < 0x10040000UL) ||
                ((int)addr > 0x20000000UL && (int)addr < 0x20080000UL) ||
                ((int)addr > 0x30000000UL && (int)addr < 0x30010000UL))
        {
            addr = (uint32_t *)((((uint32_t)addr) / 4) * 4);
            for (i = 0; i < size / 4 * 4; i += 4)
            {
                at_printf("0x%p: ", addr + i);
                for (j = 0; j < 4; j++)
                {
                    data = *(addr + i + j);
                    if (j == 3)
                        at_printf("%08x", data);
                    else
                        at_printf("%08x ", data);
                }
                at_printf("\r\n");
            }
            if (size % 4)
            {
                at_printf("0x%p: ", addr + size / 4 * 4);
                for (i = size / 4 * 4; i < size; i++)
                {
                    data = *(addr + i);
                    if (i == size - 1)
                        at_printf("%08x", data);
                    else
                        at_printf("%08x ", data);
                }
                at_printf("\r\n");
            }
        }
        else
        {
            at_print_error_param();
        }
    }
    else
    {
        at_print_error_param();
    }
}

void at_setup_mem16(uint8_t *pPara)
{
    uint32_t size = 0, i, j;
    uint32_t data = 0;
    uint16_t *addr = NULL;
    uint32_t ret = sscanf((char *)pPara, "=%p,%d", &addr, &size);

    if (ret == 2)
    {
        if (((int)addr > 0x10000000UL && (int)addr < 0x10040000UL) ||
                ((int)addr > 0x20000000UL && (int)addr < 0x20080000UL) ||
                ((int)addr > 0x30000000UL && (int)addr < 0x30010000UL))
        {
            addr = (uint16_t *)((((uint32_t)addr) / 2) * 2);
            for (i = 0; i < size / 8 * 8; i += 8)
            {
                at_printf("0x%p: ", addr + i);
                for (j = 0; j < 8; j++)
                {
                    data = *(addr + i + j);
                    if (j == 7)
                        at_printf("%04x", data);
                    else
                        at_printf("%04x ", data);
                }
                at_printf("\r\n");
            }
            if (size % 8)
            {
                at_printf("0x%p: ", addr + size / 8 * 8);
                for (i = size / 8 * 8; i < size; i++)
                {
                    data = *(addr + i);
                    if (i == size - 1)
                        at_printf("%04x", data);
                    else
                        at_printf("%04x ", data);
                }
                at_printf("\r\n");
            }
        }
        else
        {
            at_print_error_param();
        }
    }
    else
    {
        at_print_error_param();
    }
}

void at_setup_mem8(uint8_t *pPara)
{
    uint32_t size = 0, i, j;
    uint32_t data = 0;
    uint8_t *addr = NULL;
    uint32_t ret = sscanf((char *)pPara, "=%p,%d", &addr, &size);

    if (ret == 2)
    {
        if (((int)addr > 0x10000000UL && (int)addr < 0x10040000UL) ||
                ((int)addr > 0x20000000UL && (int)addr < 0x20080000UL) ||
                ((int)addr > 0x30000000UL && (int)addr < 0x30010000UL))
        {
            //addr = (uint8_t *)((((uint32_t)addr)/4)*4);
            for (i = 0; i < size / 16 * 16; i += 16)
            {
                at_printf("0x%p: ", addr + i);
                for (j = 0; j < 16; j++)
                {
                    data = *(addr + i + j);
                    if (j == 15)
                        at_printf("%02x", data);
                    else
                        at_printf("%02x ", data);
                }
                at_printf("\r\n");
            }
            if (size % 16)
            {
                at_printf("0x%p: ", addr + size / 16 * 16);
                for (i = size / 16 * 16; i < size; i++)
                {
                    data = *(addr + i);
                    if (i == size - 1)
                        at_printf("%02x", data);
                    else
                        at_printf("%02x ", data);
                }
                at_printf("\r\n");
            }
        }
        else
        {
            at_print_error_param();
        }
    }
    else
    {
        at_print_error_param();
    }
}

void at_test_w4(void)
{
    at_printf("info. set data(4bytes) at address\r\n");
    at_printf("exp. AT+WW:0x20001002,0x12345678\r\n");
}

void at_test_w2(void)
{
    at_printf("info. set data(2bytes) at address\r\n");
    at_printf("exp. AT+WH:0x20001002,0x1234\r\n");
}

void at_test_w1(void)
{
    at_printf("info. set data(1bytes) at address\r\n");
    at_printf("exp. AT+WB:0x20001002,0x12\r\n");
}

void setup_write_data(uint8_t *pPara, uint8_t bytes)
{
//    uint8_t *addr_n = NULL, *data_n = NULL;
//    uint32_t ret = sscanf((char *)pPara, "=%p,%p",&addr_n, &data_n);
//
//    if(ret == 2)
//    {
//        if(bytes == 4)
//        {
//            uint32_t *addr = (uint32_t *)addr_n;
//            uint32_t data = (uint32_t)data_n;
//        }
//        else if(bytes == 2)
//        {
//            uint16_t *addr = (uint16_t *)addr_n;
//            uint16_t data = (uint16_t)data_n;
//        }
//        else if(bytes == 1)
//        {
//            uint8_t *addr = (uint8_t *)addr_n;
//            uint8_t data = (uint8_t)data_n;
//        }
//        else
//        {
//            at_printf("wrong\r\n");
//            return;
//        }
//        if((int)addr > 0x10000000UL && (int)addr < 0x10040000UL)
//        {
//            /* ram data */
//            *(addr) = data;
//        }
//        else if((int)addr > 0x20000000UL && (int)addr < 0x20080000UL)
//        {
//            /* flash data */
//            at_printf("not support flash data write\r\n");
//        }
//        else if((int)addr > 0x40040000UL && (int)addr < 0x40100000UL)
//        {
//            /* APBPERIPH_BASE register */
//            *(addr) = data;
//        }
//        else if((int)addr > 0x30000000UL && (int)addr < 0x30100000UL)
//        {
//            /* PLC register */
//            *(addr) = data;
//        }
//        else if((int)addr > 0xE000E000UL && (int)addr < 0xE0010000UL)
//        {
//            /* SYS register */
//            *(addr) = data;
//        }
//        else
//        {
//            at_printf("address is wrong\r\n");
//            return;
//        }
//    }
//    else
//    {
//        at_printf("parameter error\r\n");
//    }
}
void at_setup_w4(uint8_t *pPara)
{
    setup_write_data(pPara, 4);
}

void at_setup_w2(uint8_t *pPara)
{
    setup_write_data(pPara, 2);
}

void at_setup_w1(uint8_t *pPara)
{
    setup_write_data(pPara, 1);
}

void at_test_param(void)
{
    at_printf("info. set param data address and length, then send param data by binary\r\n");
    at_printf("exp. AT+PARAM=0x200012000,18\r\n");
}

extern cs_buffer_t usart_buffer;
extern cmd_route_t current_cmd_route;
extern TimerHandle_t AT_SoftTimerHandle;
/* this will call after 3000ms */
void AT_SoftTimeout(void *param)
{
    if(at_param_update == 0)
        return;
    
    taskENTER_CRITICAL();
    
    if(current_cmd_route == CMD_ROUTE_USART)
    {
        usart_buffer.read_index = usart_buffer.save_index = 0;
    }
    else
    {
        /* udp not need to reset buffer */
    }
    at_param_update = at_param_size = at_param_addr = 0;
    
    taskEXIT_CRITICAL();
    
    at_print_error();
    at_printf("receive param data timeout\r\n");
}

void at_setup_param(uint8_t *pPara)
{
    uint32_t size = 0;
    uint8_t *addr = NULL;
    uint32_t ret = sscanf((char *)pPara, "=%p,%d", &addr, &size);

    if (ret == 2)
    {
        at_param_size = size;
        at_param_addr = (uint32_t)addr;
        at_param_update = 1;
        
        if(xTimerIsTimerActive(AT_SoftTimerHandle) == pdFALSE)
        {
            if(pdPASS != (xTimerStart(AT_SoftTimerHandle, 1)))
            {
                at_print_error();
                at_printf("at soft timer start failure\n");
            }
        }
        else
        {
            xTimerReset(AT_SoftTimerHandle, 1);
        }
            
        at_printf(at_param_waitting);
    }
    else
    {
        at_print_error_param();
    }
}

void at_test_update_cur(void)
{
    at_printf("info. AT+UPDATE_CUR=<ip>,<port> remote update according <ip>&<port>\r\n");
    at_printf("exp.  AT+UPDATE_CUR=\"192.168.0.202\",\"5510\"\r\n");
}

void at_test_update_def(void)
{
    at_printf("info. AT+UPDATE_DEF=<ip>,<port> remote update according <ip>&<port>\r\n");
    at_printf("exp.  AT+UPDATE_DEF=\"192.168.0.202\",\"5510\"\r\n");
}

void at_test_update_serial(void)
{
    at_printf("info. AT+UPDATE_SERIAL=<usart_base> serial update according <usart_base>\r\n");
    at_printf("exp.  AT+UPDATE_SERIAL=\"usart2\"\r\n");
}

void at_query_update_def(void)
{
    at_printf("AT+UPDATE_DEF=\"%s\",\"%d\"\r\n",
              inet_ntoa(cr600_param->upgrade.server_ip),
              cr600_param->upgrade.server_port);
}

void at_exe_update_def(void)
{
    os_upgrade_remote(cr600_param->upgrade.server_ip, cr600_param->upgrade.server_port);
}

static uint8_t setup_update(uint8_t *pPara, cr600_param_t *config)
{
    uint32_t ip[4] = {0};
    uint32_t port = 0;
    uint32_t ret = sscanf((char *)pPara, "=\"%d.%d.%d.%d\",\"%d\"",
                          &ip[0], &ip[1], &ip[2], &ip[3], &port);
    if (ret != 5)
    {
        at_printf("parameter num is wrong\r\n");
        return AT_FALSE;
    }
    uint32_t ip_addr = ip[0] + (ip[1] << 8) + (ip[2] << 16) + (ip[3] << 24);

    config->upgrade.server_ip   = ip_addr;
    config->upgrade.server_port = port;

    return AT_TRUE;
}

void at_setup_update_def(uint8_t *pPara)
{
    if (setup_update(pPara, cr600_param) == AT_TRUE)
    {
        cr600_write_sys_param();
        os_upgrade_remote(cr600_param->upgrade.server_ip, cr600_param->upgrade.server_port);
    }
}

void at_setup_update_serial(uint8_t *pPara)
{
    uint8_t usart_str[32] = {0};

    if (strlen((char *)pPara) > 11)
    {
        at_printf("parameter length is wrong\r\n");
        return;
    }
    uint32_t ret = sscanf((char *)pPara, "=\"%s\"", usart_str);
    if (ret != 1)
    {
        at_printf("parameter num is wrong\r\n");
        return;
    }

    if (0 == strcmp((char *)usart_str, "usart0\""))
    {
        os_upgrade_usart(USART0);
    }
    else if (0 == strcmp((char *)usart_str, "usart1\""))
    {
        os_upgrade_usart(USART1);
    }
    else if (0 == strcmp((char *)usart_str, "usart2\""))
    {
        os_upgrade_usart(USART2);
    }
    else if (0 == strcmp((char *)usart_str, "usart3\""))
    {
        os_upgrade_usart(USART3);
    }
    else if (0 == strcmp((char *)usart_str, "usart4\""))
    {
        os_upgrade_usart(USART4);
    }
    else
    {
        at_print_error();
    }
}

void at_test_led_beacon(void)
{
    at_printf("info. AT+LEDBEACON=0|1 diable/enable led beacon\r\n");
    at_printf("exp.  AT+LEDBEACON=1\r\n");
}
void at_query_led_beacon(void)
{
    at_printf("AT+LEDBEACON=%d\r\n", cr600_param->net_ethplc.led_beacon == DISABLE ? 0 : 1);
    at_print_ok();
}

void at_setup_led_beacon(uint8_t *pPara)
{
    uint32_t state = 0;
    uint32_t ret = 0;

    ret = sscanf((char *)pPara, "=%d", &state);

    if (ret == 1)
    {
        if (state == 0 || state == 1)
        {
            cr600_param->net_ethplc.led_beacon = state == 1 ? ENABLE : DISABLE;
            cr600_write_sys_param();
            at_print_ok();
            cr600_reset();
        }
    }
    else
    {
        at_print_error_param();
    }
}