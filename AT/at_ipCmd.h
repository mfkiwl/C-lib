/*
 * at_ipCmd.h
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

#ifndef __AT_IPCMD_H
#define __AT_IPCMD_H

#include "at.h"

void at_test_ip_cur(void);
void at_query_ip_cur(void);
void at_setup_ip_cur(uint8_t *pPara);
void at_test_ip_def(void);
void at_query_ip_def(void);
void at_setup_ip_def(uint8_t *pPara);

void at_test_dns_cur(void);
void at_query_dns_cur(void);
void at_setup_dns_cur(uint8_t *pPara);
void at_test_dns_def(void);
void at_query_dns_def(void);
void at_setup_dns_def(uint8_t *pPara);

void at_test_dhcp_cur(void);
void at_query_dhcp_cur(void);
void at_setup_dhcp_cur(uint8_t *pPara);
void at_test_dhcp_def(void);
void at_query_dhcp_def(void);
void at_setup_dhcp_def(uint8_t *pPara);

void at_test_mac_cur(void);
void at_query_mac_cur(void);
void at_setup_mac_cur(uint8_t *pPara);
void at_test_mac_def(void);
void at_query_mac_def(void);
void at_setup_mac_def(uint8_t *pPara);

void at_query_ipstatus(void);

void at_setup_start(uint8_t *pPara);

void at_setup_ipmux(uint8_t *pPara);

void at_setup_send(uint8_t *pPara);
void at_setup_sendex(uint8_t *pPara);
void at_setup_sendbuf(uint8_t *pPara);

void at_setup_bufreset(uint8_t *pPara);

void at_query_bufstatus(void);

void at_query_bufstatus(void);

void at_setup_close(uint8_t *pPara);

void at_query_ckeckseq(void);

void at_test_ifsr(void);
void at_query_ifsr(void);

void at_setup_server(uint8_t *pPara);

void at_setup_mode(uint8_t *pPara);

void at_setup_savetranslink(uint8_t *pPara);

void at_setup_sto(uint8_t *pPara);
void at_setup_ipdinfo(uint8_t *pPara);

void at_test_domain(void);
void at_setup_domain(uint8_t *pPara);

void at_test_ifconfig(void);
void at_exe_ifconfig(void);

void at_test_netstat(void);
void at_exe_netstat(void);

void at_test_arp(void);
void at_query_arp(void);
void at_setup_arp(uint8_t *pPara);

void at_test_ping(void);
void at_setup_ping(uint8_t *pPara);

void at_test_lwipstats(void);
void at_query_lwipstats(void);

#endif
