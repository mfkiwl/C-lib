/*
 * at_baseCmd.h
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

#ifndef __AT_BASECMD_H
#define __AT_BASECMD_H

void at_exe_at(void);
void at_setup_echo(uint8_t *pPara);

void at_test_rst(void);
void at_exe_rst(void);
void at_test_gmr(void);
void at_exe_gmr(void);
void at_setup_gslp(uint8_t *pPara);

void at_test_restore(void);
void at_exe_restore(void);

void at_test_config(void);
void at_exe_config(void);

void at_test_cmdlist(void);
void at_exe_cmdlist(void);

#ifdef AT_USE_USART
void at_test_uart_cur(void);
void at_query_uart_cur(void);
void at_setup_uart_cur(uint8_t *pPara);
void at_test_uart_def(void);
void at_query_uart_def(void);
void at_setup_uart_def(uint8_t *pPara);
#endif

void at_test_osruntime(void);
void at_query_osruntime(void);
void at_test_ostasklist(void);
void at_query_ostasklist(void);
void at_test_heap(void);
void at_query_heap(void);
void at_setup_osruntime(uint8_t *pPara);

void at_test_plc(void);
void at_setup_plc(uint8_t *pPara);
void at_test_plc_cmd_check(void);
void at_query_plc_cmd_check(void);
void at_setup_plc_cmd_check(uint8_t *pPara);
void at_test_plc_tml(void);
void at_query_plc_tml(void);
void at_exe_plc_linksend(void);
void at_test_plc_stat(void);
void at_setup_plc_stat(uint8_t *pPara);
void at_exe_plc_stat(void);
void at_test_bub_stat(void);
void at_setup_bub_stat(uint8_t *pPara);
void at_exe_bub_stat(void);
void at_query_plc_flooding_opt_list(void);
void at_query_plc_physwitch(void);
void at_setup_plc_physwitch(uint8_t *pParam);
void at_query_eth_transpswitch(void);
void at_setup_eth_transpswitch(uint8_t *pPara);
void at_query_plc_transpswitch(void);
void at_setup_plc_transpswitch(uint8_t *pPara);
void at_query_plc_hopdebug(void);
void at_setup_plc_hopdebug(uint8_t *pParam);
void at_query_plc_hoprole(void);
void at_setup_plc_hoprole(uint8_t *pPara);
void at_test_mem32(void);
void at_test_mem16(void);
void at_test_mem8(void);
void at_setup_mem32(uint8_t *pPara);
void at_setup_mem16(uint8_t *pPara);
void at_setup_mem8(uint8_t *pPara);
void at_test_w4(void);
void at_test_w2(void);
void at_test_w1(void);
void at_setup_w4(uint8_t *pPara);
void at_setup_w2(uint8_t *pPara);
void at_setup_w1(uint8_t *pPara);

void at_test_param(void);
void at_setup_param(uint8_t *pPara);

void at_test_update_cur(void);
void at_query_update_cur(void);
void at_setup_update_cur(uint8_t *pPara);
void at_exe_update_cur(void);
void at_test_update_def(void);
void at_query_update_def(void);
void at_setup_update_def(uint8_t *pPara);
void at_exe_update_def(void);

void at_test_update_serial(void);
void at_setup_update_serial(uint8_t *pPara);
void at_query_plc_hopswitch(void);
void at_setup_plc_hopswitch(uint8_t *pPara);
void at_setup_plc_physend(uint8_t *pPara);
void at_setup_plc_hopsend(uint8_t *pPara);

void at_test_led_beacon(void);
void at_query_led_beacon(void);
void at_setup_led_beacon(uint8_t *pPara);

#endif
