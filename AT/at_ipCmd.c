/*
 * at_ipCmd.c
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

#include "at_ipCmd.h"
#include "at_uart.h"
#include "cr600_tools.h"

#include "lwip\inet.h"
#include "lwip\netif.h"
#include "lwip\opt.h"
#include "lwip\tcp.h"
#include "lwip\tcp_impl.h"
#include "lwip\sys.h"
#include "lwip\raw.h"
#include "lwip\inet_chksum.h"
#include "lwip\timers.h"
#include "netif/etharp.h"

struct IPCMD_CONFIG
{
    uint8_t ipmux;
    uint8_t send;
    uint8_t sendex;
    uint8_t sendbuf;
    uint32_t buf_length;
    uint8_t server;
    uint8_t server_port;
    uint8_t tranfer_mode;
    uint32_t timeout;
    uint8_t ipd_info;
};

extern struct netif *netif_list;

static struct IPCMD_CONFIG ipcmd_config;

extern dhcp_mode_t lwip_get_dhcp(void);
extern void at_print_ok(void);
/* ip setting */
static uint8_t setup_ip(uint8_t *pPara, cr600_param_t *config)
{
    uint32_t ip[4], gate[4], mask[4];
    uint32_t ret = 0;
    uint32_t ip_addr, gate_addr, mask_addr;

    /* check if dhcp start */
#if 0 != LWIP_DHCP
    if (lwip_get_dhcp() == DHCP_ENABLE)
    {
        at_printf("please disable dhcp mode\r\n");
        at_print_error();
        return AT_FALSE;
    }
#endif
    ret = sscanf((char *)pPara, "=\"%d.%d.%d.%d\",\"%d.%d.%d.%d\",\"%d.%d.%d.%d\"\r\n",
                 &ip[0], &ip[1], &ip[2], &ip[3],
                 &gate[0], &gate[1], &gate[2], &gate[3],
                 &mask[0], &mask[1], &mask[2], &mask[3]);
    if (ret != 12)
    {
        ret = sscanf((char *)pPara, "=\"%d.%d.%d.%d\"\r\n",
                     &ip[0], &ip[1], &ip[2], &ip[3]);
        if (ret != 4)
        {
            at_print_error_param();
            return AT_FALSE;
        }
    }

    ip_addr = ip[0] + (ip[1] << 8) + (ip[2] << 16) + (ip[3] << 24);
    if (ret == 12)
    {
        gate_addr = gate[0] + (gate[1] << 8) + (gate[2] << 16) + (gate[3] << 24);
        mask_addr = mask[0] + (mask[1] << 8) + (mask[2] << 16) + (mask[3] << 24);
    }
    else if (ret == 4)
    {
        gate_addr = config->net_ethplc.gate;
        mask_addr = config->net_ethplc.mask;
    }
	else
	{
		return AT_FALSE;
	}
	config->net_ethplc.ip   = ip_addr;
	config->net_ethplc.gate = gate_addr;
	config->net_ethplc.mask = mask_addr;
	at_print_ok();
	vTaskDelay(100);
	lwip_set_addr(ip_addr, mask_addr, gate_addr);
    return AT_TRUE;
}

//void at_test_ip_cur(void)
//{
//    at_printf("info. AT+CIPSTA_CUR=<ip>[,<gateway>,<netmask>]\r\n");
//    at_printf("exp. AT+CIPSTA_CUR=\"192.168.1.100\",\"192.168.1.1\",\"255.255.255.0\"\r\n");
//    at_printf("exp. AT+CIPSTA_CUR=\"192.168.1.100\"\r\n");
//}

void at_test_ip_def(void)
{
    at_printf("info. AT+CIPSTA_DEF=<ip>[,<gateway>,<netmask>]\r\n");
    at_printf("exp. AT+CIPSTA_DEF=\"192.168.1.100\",\"192.168.1.1\",\"255.255.255.0\"\r\n");
    at_printf("exp. AT+CIPSTA_DEF=\"192.168.1.100\"\r\n");
}

//void at_query_ip_cur(void)
//{
//    at_printf("AT+CIPSTA_CUR=\"%s\",", inet_ntoa(cr600_param_cur->net_ethplc.ip));
//    at_printf("\"%s\",", inet_ntoa(cr600_param_cur->net_ethplc.gate));
//    at_printf("\"%s\"\r\n", inet_ntoa(cr600_param_cur->net_ethplc.mask));
//}

void at_query_ip_def(void)
{
    at_printf("AT+CIPSTA_DEF=\"%s\",", inet_ntoa(cr600_param->net_ethplc.ip));
    at_printf("\"%s\",", inet_ntoa(cr600_param->net_ethplc.gate));
    at_printf("\"%s\"\r\n", inet_ntoa(cr600_param->net_ethplc.mask));
}

//void at_setup_ip_cur(uint8_t *pPara)
//{
//    setup_ip(pPara, cr600_param_cur);
//}

void at_setup_ip_def(uint8_t *pPara)
{
    if (setup_ip(pPara, cr600_param) == AT_TRUE)
    {
        cr600_write_sys_param();
    }
}

/* ip setting */
static uint8_t setup_dns(uint8_t *pPara, cr600_param_t *config)
{
    uint32_t dns[4];
    uint32_t ret = 0;
    uint32_t dns_addr;

    ret = sscanf((char *)pPara, "=\"%d.%d.%d.%d\"\r\n",
                 &dns[0], &dns[1], &dns[2], &dns[3]);
    if (ret != 4)
    {
        at_print_error_param();
        return AT_FALSE;
    }

    dns_addr = dns[0] + (dns[1] << 8) + (dns[2] << 16) + (dns[3] << 24);

    config->net_ethplc.dns   = dns_addr;
    at_print_ok();
    return AT_TRUE;
}

//void at_test_dns_cur(void)
//{
//    at_printf("info. AT+CIPSTA_CUR=<dns>\r\n");
//    at_printf("exp. AT+CIPSTA_CUR=\"192.168.1.1\"\r\n");
//}

void at_test_dns_def(void)
{
    at_printf("info. AT+CIPSTA_DEF=<dns>\r\n");
    at_printf("exp. AT+CIPSTA_DEF=\"192.168.1.1\"\r\n");
}

//void at_query_dns_cur(void)
//{
//    at_printf("AT+CIPSTA_CUR=\"%s\",", inet_ntoa(cr600_param_cur->net_ethplc.dns));
//}

void at_query_dns_def(void)
{
    at_printf("AT+CIPSTA_DEF=\"%s\",", inet_ntoa(cr600_param->net_ethplc.dns));
}

//void at_setup_dns_cur(uint8_t *pPara)
//{
//    setup_dns(pPara, cr600_param_cur);
//}

void at_setup_dns_def(uint8_t *pPara)
{
    if (setup_dns(pPara, cr600_param) == AT_TRUE)
    {
        cr600_write_sys_param();
    }
}


extern uint8_t lwip_set_dhcp(dhcp_mode_t mode);
/* dhcp setting */
static uint8_t setup_dhcp(uint8_t *pPara, cr600_param_t *config)
{
#if (LWIP_DHCP == 1)
    uint32_t mode, status;
    uint32_t ret = 0;

    ret = sscanf((char *)pPara, "=%d,%d\r\n", &mode, &status);
    if (ret != 2)
    {
        at_print_error_param();
    }
    else
    {
        if (lwip_set_dhcp((dhcp_mode_t)status) == AT_TRUE)
        {
            config->net_ethplc.dhcp = (status == 1) ? DHCP_ENABLE : DHCP_DISABLE;
            at_print_ok();
            return AT_TRUE;
        }
        else
        {
            at_print_error();
        }
    }
#else
    at_printf("please open macro:LWIP_DHCP\r\n");
#endif

    return AT_FALSE;
}
//void at_test_dhcp_cur(void)
//{
//    at_printf("info. AT+CWDHCP_CUR=<mode>,<status>\r\n");
//    at_printf("more. mode:0|1|2  status:0|1\r\n");
//}

void at_test_dhcp_def(void)
{
    at_printf("info. AT+CWDHCP_DEF=<mode>,<status>\r\n");
    at_printf("more. mode:0|1|2  status:0|1\r\n");
}

//void at_query_dhcp_cur(void)
//{
//    at_printf("AT+CWDHCP_CUR=1,%d\r\n", (cr600_param_cur->net_ethplc.dhcp == DHCP_ENABLE) ? 1 : 0);
//}

void at_query_dhcp_def(void)
{
    at_printf("AT+CWDHCP_DEF=1,%d\r\n", (cr600_param->net_ethplc.dhcp == DHCP_ENABLE) ? 1 : 0);
}

//void at_setup_dhcp_cur(uint8_t *pPara)
//{
//    setup_dhcp(pPara, cr600_param_cur);
//}

void at_setup_dhcp_def(uint8_t *pPara)
{
    if (setup_dhcp(pPara, cr600_param) == AT_TRUE)
    {
        cr600_write_sys_param();
    }
}

/* mac setting */
static uint8_t setup_mac(uint8_t *pPara, cr600_param_t *config)
{
    uint32_t mac[6] = {0};
    MacAddr  mac_n = {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
    uint32_t ret = 0;

    ret = sscanf((char *)pPara, "=\"%02x:%02x:%02x:%02x:%02x:%02x\"\r\n",
                 &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]);
    if (ret != 6)
    {
        at_print_error_param();
    }
    else
    {
        for(uint32_t i = 0; i < 6; i++)
            mac_n.mac[i] = (uint8_t)mac[i];
        
        if (lwip_set_mac(mac_n) == 1)
        {
            memcpy(&cr600_param->plc.mac_addr, mac_n.mac, 6);
            at_print_ok();
            return AT_TRUE;
        }
    }

    return AT_FALSE;
}

//void at_setup_mac_cur(uint8_t *pPara)
//{
//    setup_mac(pPara, cr600_param_cur);
//}

void at_setup_mac_def(uint8_t *pPara)
{
    if (setup_mac(pPara, cr600_param) == AT_TRUE)
    {
        cr600_write_sys_param();
    }
    at_printf("please rebooting to active mac address setting\r\n");
}

//void at_test_mac_cur(void)
//{
//    at_printf("info. AT+CIPSTAMAC_CUR=<mac>\r\n");
//    at_printf("exp. AT+CIPSTAMAC_CUR=\"18:fe:35:98:d3:7b\"\r\n");
//}

void at_test_mac_def(void)
{
    at_printf("info. AT+CIPSTAMAC_DEF=<mac>\r\n");
    at_printf("exp. AT+CIPSTAMAC_DEF=\"18:fe:35:98:d3:7b\"\r\n");
}

//void at_query_mac_cur(void)
//{
//    at_printf("AT+CIPSTAMAC_CUR=\"%02X:%02X:%02X:%02X:%02X:%02X\"\r\n", cr600_param_cur->plc.mac_addr[0],
//              cr600_param_cur->plc.mac_addr[1],
//              cr600_param_cur->plc.mac_addr[2],
//              cr600_param_cur->plc.mac_addr[3],
//              cr600_param_cur->plc.mac_addr[4],
//              cr600_param_cur->plc.mac_addr[5]);
//}

void at_query_mac_def(void)
{
    at_printf("AT+CIPSTAMAC_DEF=\"%02X:%02X:%02X:%02X:%02X:%02X\"\r\n", cr600_param->plc.mac_addr[0],
              cr600_param->plc.mac_addr[1],
              cr600_param->plc.mac_addr[2],
              cr600_param->plc.mac_addr[3],
              cr600_param->plc.mac_addr[4],
              cr600_param->plc.mac_addr[5]);
}

void at_setup_ipmux(uint8_t *pPara)
{
    uint32_t flag = 0;
    uint32_t ret = sscanf((char *)pPara, "=%d\r\n", &flag);

    if (ret != 1)
    {
        at_print_error_param();
    }
    else if (flag != 0 && flag != 1)
    {
        at_print_error_param();
    }
    else
    {
        ipcmd_config.ipmux = flag;
        at_print_ok();
    }
}

void at_query_ipstatus(void)
{

}
void at_setup_start(uint8_t *pPara)
{
    uint32_t type, link_id;
    //uint32_t ret = 0;

    pPara++;
    if (ipcmd_config.ipmux)
    {
        link_id = atoi((char *)pPara);
        pPara++;
        pPara = (uint8_t *)strchr((char *)pPara, '\"');
    }
    else
    {
        link_id = 0;
    }
    if (link_id > 4)
    {
        at_print_error();
        return;
    }
    if (strstr((char *)pPara, "UDP"))
    {
        type = 1;
    }
    else if (strstr((char *)pPara, "TCP"))
    {
        type = 0;
    }
    else
    {
        at_print_error();
        return;
    }
    (void)type;
    at_print_ok();
    return;
}
void at_setup_send(uint8_t *pPara)
{
    uint32_t link_id = 0, length;
    if (ipcmd_config.ipmux)
    {
        uint32_t ret = sscanf((char *)pPara, "=%d,%d", &link_id, &length);
        if (ret != 2)
        {
            at_print_error_param();
            return;
        }
    }
    else
    {
        uint32_t ret = sscanf((char *)pPara, "=%d", &length);
        if (ret != 1)
        {
            at_print_error_param();
            return;
        }
    }

    if (link_id > 4)
    {
        at_print_error();
        return;
    }
    ipcmd_config.send = 1;
    at_print_ok();
    return;
}
void at_setup_sendex(uint8_t *pPara)
{
    uint32_t link_id = 0, length;
    if (ipcmd_config.ipmux)
    {
        uint32_t ret = sscanf((char *)pPara, "=%d,%d", &link_id, &length);
        if (ret != 2)
        {
            at_print_error_param();
            return;
        }
    }
    else
    {
        uint32_t ret = sscanf((char *)pPara, "=%d", &length);
        if (ret != 1)
        {
            at_print_error_param();
            return;
        }
    }

    if (link_id > 4)
    {
        at_print_error();
        return;
    }
    ipcmd_config.sendex = 1;
    at_print_ok();
    return;
}
void at_setup_sendbuf(uint8_t *pPara)
{
    uint32_t link_id = 0, length;
    if (ipcmd_config.ipmux)
    {
        uint32_t ret = sscanf((char *)pPara, "=%d,%d", &link_id, &length);
        if (ret != 2)
        {
            at_print_error_param();
            return;
        }
    }
    else
    {
        uint32_t ret = sscanf((char *)pPara, "=%d", &length);
        if (ret != 1)
        {
            at_print_error_param();
            return;
        }
    }

    if (link_id > 4)
    {
        at_print_error();
        return;
    }
    ipcmd_config.sendbuf = 1;
    ipcmd_config.buf_length = length;
    at_print_ok();
    return;
}

void at_setup_bufreset(uint8_t *pPara)
{
    uint32_t link_id = 0;
    if (ipcmd_config.ipmux)
    {
        uint32_t ret = sscanf((char *)pPara, "=%d", &link_id);
        if (ret != 1)
        {
            at_print_error_param();
            return;
        }
    }


    if (link_id > 4)
    {
        at_print_error();
        return;
    }
    ipcmd_config.buf_length = 0;
    return;
}
void at_query_bufstatus(void)
{
    at_printf("BUF:%d\r\n", ipcmd_config.buf_length);
    return;
}

void at_query_ckeckseq(void)
{
    if (ipcmd_config.buf_length == 0)
        at_printf("buf empty\r\n");
    else
        at_printf("buf is not empty\r\n");
    return;
}

void at_setup_close(uint8_t *pPara)
{
    uint32_t link_id = 0;
    if (ipcmd_config.ipmux)
    {
        uint32_t ret = sscanf((char *)pPara, "=%d", &link_id);
        if (ret != 1)
        {
            at_print_error_param();
            return;
        }
    }


    if (link_id > 4)
    {
        at_print_error();
        return;
    }

    at_printf("closed\r\n");
    return;
}

void at_test_ifsr(void)
{
    at_printf("info.get ip,gateway,netmask address\r\n");
    at_printf("exp. AT+CIFSR\r\n");
}

void at_query_ifsr(void)
{
    uint32_t ip, mask, gw;

    lwip_get_addr(&ip, &gw, &mask);

    at_printf("AT+CIFSR=\"%s\",", inet_ntoa(ip));
    at_printf("\"%s\",", inet_ntoa(gw));
    at_printf("\"%s\"\r\n", inet_ntoa(mask));

    return;
}

void at_setup_server(uint8_t *pPara)
{
    uint32_t mode, port;

    uint32_t ret;

    ret = sscanf((char *)pPara, "=%d,%d", &mode, &port);

    if (ret == 2)
    {
        ipcmd_config.server = mode;
        ipcmd_config.server_port = port;
    }
    if (ret == 1)
    {
        ipcmd_config.server = mode;
    }

    at_print_ok();
}

void at_setup_mode(uint8_t *pPara)
{
    uint32_t mode;

    uint32_t ret;

    ret = sscanf((char *)pPara, "=%d", &mode);

    if (ret == 1 && (mode == 0 || mode == 1))
    {
        ipcmd_config.tranfer_mode = mode;
    }

    at_print_ok();
}

void at_setup_savetranslink(uint8_t *pPara)
{
    uint32_t mode;

    uint32_t ret;

    ret = sscanf((char *)pPara, "=%d", &mode);

    if (ret == 1 && (mode == 0 || mode == 1))
    {
        ipcmd_config.tranfer_mode = mode;
    }

    at_print_ok();
}

void at_setup_sto(uint8_t *pPara)
{
    uint32_t timeout;

    uint32_t ret;

    ret = sscanf((char *)pPara, "=%d", &timeout);

    if (ret == 1)
    {
        ipcmd_config.timeout = timeout;
    }

    at_print_ok();
}

void at_setup_ipdinfo(uint8_t *pPara)
{
    uint32_t info;

    uint32_t ret;

    ret = sscanf((char *)pPara, "=%d", &info);

    if (ret == 1 && (info == 1 || info == 0))
    {
        ipcmd_config.ipd_info = info;
        at_print_ok();
    }
    else
        at_print_error();
    return;

}

extern uint32_t lwip_get_host_by_name(char *hostname);

void at_test_domain(void)
{
    at_printf("info. get ip address according domain\r\n");
    at_printf("exp. AT+CIPDOMAIN=\"www.baidu.com\"\r\n");
}
void at_setup_domain(uint8_t *pPara)
{
#if LWIP_DNS & LWIP_SOCKET
    char buf[128] = {0};
    char *token_first = strchr((char *)pPara, '\"');

    if (token_first == NULL)
    {
        at_print_error_param();
        return;
    }
    char *token_second = strchr(token_first + 1, '\"');
    if (token_second == NULL)
    {
        at_print_error_param();
        return;
    }
    //memset(at_stdout_buf, 0, sizeof(at_stdout_buf));
    memcpy(buf, token_first + 1, token_second - token_first - 1);

    uint32_t addr = lwip_get_host_by_name(buf);

    at_printf("%s:%s\r\n", buf, inet_ntoa(addr));
#else
    at_printf("please open macro: LWIP_DNS & LWIP_SOCKET\r\n");
#endif
}


void at_test_ifconfig(void)
{
    at_printf("info. display all the netif(network interface) information\r\n");
}
void at_exe_ifconfig(void)
{
    struct netif *dev = netif_list;

    if (netif_list == NULL)
    {
        at_printf("no netif, IS lwip init?\r\n");
        return;
    }
    for (dev = netif_list; dev != NULL; dev = dev->next)
    {
        at_printf("%c%c%d%s ", dev->name[0], dev->name[1], dev->num, (dev == netif_default) ? "(Default)" : "   ");
        at_printf("hostname:%s  ", dev->hostname);
        at_printf("hwaddr:%02X:%02X:%02X:%02X:%02X:%02X\r\n", dev->hwaddr[0], dev->hwaddr[1],
                  dev->hwaddr[2], dev->hwaddr[3], dev->hwaddr[4], dev->hwaddr[5]);
        at_printf("        ip addr:%s  ", inet_ntoa(dev->ip_addr));
        at_printf("gateway:%s  ", inet_ntoa(dev->gw));
        at_printf("mask:%s\r\n", inet_ntoa(dev->netmask));
        at_printf("        FLAGS:");
        if (dev->flags & NETIF_FLAG_UP) at_printf(" UP");
        else at_printf(" DOWN");
        if (dev->flags & NETIF_FLAG_LINK_UP) at_printf(" LINK_UP");
        else at_printf(" LINK_DOWN");
        if (dev->flags & NETIF_FLAG_DHCP) at_printf(" DHCP");
        if (dev->flags & NETIF_FLAG_POINTTOPOINT) at_printf(" PPP");
        if (dev->flags & NETIF_FLAG_ETHARP) at_printf(" ETHARP");
        if (dev->flags & NETIF_FLAG_IGMP) at_printf(" IGMP");
        at_printf("\r\n");

        at_printf("        MTU:%d(Bytes)\r\n", dev->mtu);
    }
}
void at_test_netstat(void)
{
    at_printf("info. display all the tcp/udp connections information\r\n");
}
void at_exe_netstat(void)
{
#if LWIP_TCP
    uint32_t num = 0;
    struct tcp_pcb *pcb;
    char local_ip_str[16];
    char remote_ip_str[16];

    extern struct tcp_pcb *tcp_active_pcbs;
    extern union tcp_listen_pcbs_t tcp_listen_pcbs;
    extern struct tcp_pcb *tcp_tw_pcbs;
    extern const char *tcp_state_str[];

    at_printf("Active PCB states:\r\n");
    for (pcb = tcp_active_pcbs; pcb != NULL; pcb = pcb->next)
    {
        strcpy(local_ip_str, ipaddr_ntoa(&(pcb->local_ip)));
        strcpy(remote_ip_str, ipaddr_ntoa(&(pcb->remote_ip)));

        at_printf("#%d %s:%d <==> %s:%d snd_nxt 0x%08X rcv_nxt 0x%08X ",
                  num++,
                  local_ip_str,
                  pcb->local_port,
                  remote_ip_str,
                  pcb->remote_port,
                  pcb->snd_nxt,
                  pcb->rcv_nxt);
        at_printf("state: %s\r\n", tcp_state_str[pcb->state]);
    }

    at_printf("Listen PCB states:\r\n");
    num = 0;
    for (pcb = (struct tcp_pcb *)tcp_listen_pcbs.pcbs; pcb != NULL; pcb = pcb->next)
    {
        at_printf("#%d local port %d ", num++, pcb->local_port);
        at_printf("state: %s\r\n", tcp_state_str[pcb->state]);
    }

    at_printf("TIME-WAIT PCB states:\r\n");
    num = 0;
    for (pcb = tcp_tw_pcbs; pcb != NULL; pcb = pcb->next)
    {
        strcpy(local_ip_str, ipaddr_ntoa(&(pcb->local_ip)));
        strcpy(remote_ip_str, ipaddr_ntoa(&(pcb->remote_ip)));

        at_printf("#%d %s:%d <==> %s:%d snd_nxt 0x%08X rcv_nxt 0x%08X ",
                  num++,
                  local_ip_str,
                  pcb->local_port,
                  remote_ip_str,
                  pcb->remote_port,
                  pcb->snd_nxt,
                  pcb->rcv_nxt);
        at_printf("state: %s\r\n", tcp_state_str[pcb->state]);
    }
    at_printf("netstat end.\r\n");
#else
    at_printf("please open macro: LWIP_TCP\r\n");
#endif
}

#if LWIP_ARP || LWIP_ETHERNET
void at_test_arp(void)
{
    at_printf("info. display/clear arp information\r\n");
    at_printf("exp. AT+CIPARP     display arp list\r\n");
    at_printf("exp. AT+CIPARP=1   clear arp list\r\n");
}

void at_query_arp(void)
{
    struct etharp_entry *entry = etharp_get_list();
    struct netif *dev = netif_list;
    char state[15] = {0};
    
    if (netif_list == NULL)
    {
        at_printf("no netif, IS lwip init?\r\n");
        return;
    }
    for (dev = netif_list; dev != NULL; dev = dev->next)
    {
        at_printf("interface:%s\r\n", inet_ntoa(dev->ip_addr));
        at_printf("   IP addr           MAC addr         type\r\n");
        for(uint32_t i = 0; i < ARP_TABLE_SIZE; i++)
        {
            memset(state, 0, 15);
            switch(entry->state)
            {
            case ETHARP_STATE_EMPTY:
                break;
            case ETHARP_STATE_PENDING:
                strcpy(state, "PENDING");
                break;
            case ETHARP_STATE_STABLE:
                strcpy(state, "STABLE");
                break;
            case ETHARP_STATE_STABLE_REREQUESTING:
                strcpy(state, "REQEQUESTING");
                break;
    #if ETHARP_SUPPORT_STATIC_ENTRIES
            case ETHARP_STATE_STATIC:
                strcpy(state, "STATIC");
                break;
    #endif /* ETHARP_SUPPORT_STATIC_ENTRIES */
            default:
                at_printf("arp list state error\r\n");
                break;
            }
            
            if(state[0] != 0)
            {
                at_printf("%15s  %02X:%02X:%02X:%02X:%02X:%02X  %s\r\n", inet_ntoa(entry->ipaddr),
                          entry->ethaddr.addr[0], entry->ethaddr.addr[1],entry->ethaddr.addr[2],
                          entry->ethaddr.addr[3],entry->ethaddr.addr[4], entry->ethaddr.addr[5], state);
            }
            
            entry++;
        }
    }
}

void at_setup_arp(uint8_t *pPara)
{
    struct netif *dev = netif_list;
    uint32_t status = 0, ret = 0;
    
    ret = sscanf((char *)pPara, "=%d", &status);

    if (ret == 1 && status == 1)
    {
        etharp_cleanup_netif(dev);
        at_print_ok();
    }
    else
        at_print_error_param();
    
    return;
}
#endif /* LWIP_ARP || LWIP_ETHERNET */

#if LWIP_TCP
/* this is ping */
#define PING_DELAY  1000
#define PING_CNT    4
#define PING_ID     0xAFAF
#define PING_DATA_SIZE  32

static uint8_t doing_ping = 0;
static u16_t ping_seq_num = 0;
static ip_addr_t ping_dst;
static u32_t ping_time_snd[PING_CNT] = {0};
static u32_t ping_time_rcv[PING_CNT] = {0};
static struct raw_pcb *ping_pcb = NULL;
extern const struct ip_hdr *current_header;

static u8_t ping_recv(void *arg, struct raw_pcb *pcb, struct pbuf *p,  ip_addr_t *addr)
{
    struct icmp_echo_hdr *iecho;
    u16_t pkg_len = PBUF_IP_HLEN + sizeof(struct icmp_echo_hdr);
    if (p->tot_len >= pkg_len)
    {
        iecho = (struct icmp_echo_hdr *)((u8_t *)p->payload + PBUF_IP_HLEN);

        if ((iecho->type == ICMP_ER) && (iecho->id == PING_ID) && (iecho->seqno == htons(ping_seq_num)))
        {
            ping_time_rcv[ping_seq_num - 1] = 0x80000000UL + sys_now() - ping_time_snd[ping_seq_num - 1];
            
            at_printf("32 bytes from %s: icmp_seq:%d ttl=%d time=%dms\r\n",
                      inet_ntoa(ping_dst.addr),
                      ping_seq_num,
                      current_header->_ttl,
                      ping_time_rcv[ping_seq_num - 1] - 0x80000000UL);

            pbuf_free(p);
            return 1;
        }
    }

    return 0;
}

static void ping_prepare_echo(struct icmp_echo_hdr *iecho, u16_t len)
{
    size_t i;
    size_t data_len = len - sizeof(struct icmp_echo_hdr);

    ICMPH_TYPE_SET(iecho, ICMP_ECHO);
    ICMPH_CODE_SET(iecho, 0);
    iecho->chksum   = 0;
    iecho->id       = PING_ID;
    iecho->seqno    = htons(++ping_seq_num);

    for (i = 0; i < data_len; i++)
        ((char *)iecho)[sizeof(struct icmp_echo_hdr) + i] = (char)i;

    iecho->chksum = inet_chksum(iecho, len);
}


static void ping_send(struct raw_pcb *raw, ip_addr_t *addr)
{
    struct pbuf *p;
    struct icmp_echo_hdr *iecho;
    size_t ping_size = sizeof(struct icmp_echo_hdr) + PING_DATA_SIZE;

    p = pbuf_alloc(PBUF_IP, (u16_t)ping_size, PBUF_RAM);
    if (!p)
        return;

    if ((p->len == p->tot_len) && (p->next == NULL))
    {
        iecho = (struct icmp_echo_hdr *)p->payload;
        ping_prepare_echo(iecho, (u16_t)ping_size);
        raw_sendto(raw, p, addr);

        ping_time_snd[ping_seq_num - 1] = sys_now();
    }

    pbuf_free(p);
}


static void ping_timeout(void *arg)
{
    if(ping_time_rcv[ping_seq_num - 1] == 0)
        at_printf("Timeout\r\n");

    /* ping ending */
    if (ping_seq_num > PING_CNT - 1)
    {
        at_printf("--- %s ping done ---\r\n", inet_ntoa(ping_dst.addr));
        doing_ping = 0;
        ping_seq_num = 0;
        memset(ping_time_snd, 0, sizeof(ping_time_snd));
        memset(ping_time_rcv, 0, sizeof(ping_time_rcv));
        raw_remove(ping_pcb);
        return;
    }
    
    struct raw_pcb *pcb = (struct raw_pcb *)arg;
    ping_send(pcb, &ping_dst);

    sys_timeout(PING_DELAY, ping_timeout, pcb);
}
#endif /* LWIP_TCP */

void at_test_ping(void)
{
    at_printf("info. AT+PING=<ip/domain>\r\n");
}

void at_setup_ping(uint8_t *pPara)
{
#if LWIP_TCP
    uint32_t ip[4];
    uint32_t ret = 0;

    /* can not get here again until doing ping end */
    if (doing_ping)
    {
        at_printf("please wait for ping end\r\n");
        return;
    }
    doing_ping = 1;
    ret = sscanf((char *)pPara, "=\"%d.%d.%d.%d\"\r\n",
                 &ip[0], &ip[1], &ip[2], &ip[3]);
    if (ret == 4)
    {
        ping_dst.addr = ip[0] + (ip[1] << 8) + (ip[2] << 16) + (ip[3] << 24);

        at_printf("ping %s 32 bytes of data.\r\n", inet_ntoa(ping_dst.addr));
    }
    else
    {
#if LWIP_DNS && LWIP_SOCKET
        char buf[128] = {0};
        char *token_first = strchr((char *)pPara, '\"');

        if (token_first == NULL)
        {
            at_print_error_param();
            return;
        }
        char *token_second = strchr(token_first + 1, '\"');
        if (token_second == NULL)
        {
            at_print_error_param();
            return;
        }
        //memset(at_stdout_buf, 0, sizeof(at_stdout_buf));
        memcpy(buf, token_first + 1, token_second - token_first - 1);

        ping_dst.addr = lwip_get_host_by_name(buf);

        at_printf("ping %s(%s) 32 bytes of data.\r\n", buf, inet_ntoa(ping_dst.addr));
#else /* LWIP_DNS && LWIP_SOCKET */
        doing_ping = 0;
        
        at_printf("please open macro:LWIP_DNS & LWIP_SOCKET to get ip by domain\r\n");
        return;
#endif
    }

    /* create ping task */
    ping_pcb = (struct raw_pcb *)raw_new(IP_PROTO_ICMP);
    raw_recv(ping_pcb, ping_recv, NULL);
    raw_bind(ping_pcb, IP_ADDR_ANY);
    ping_send(ping_pcb, &ping_dst);
    sys_timeout(PING_DELAY, ping_timeout, ping_pcb);
#else /* LWIP_TCP */
    at_printf("please open macro: LWIP_TCP\r\n");
#endif
}

void at_test_lwipstats(void)
{
    at_printf("info. AT+LWIPSTAT=? echo lwip run-stats\r\n");
}

void at_query_lwipstats(void)
{
#if (LWIP_STATS == 1)
    lwip_echo_stats();
#else
    at_printf("please open macro:LWIP_STATS\r\n");
#endif /* (LWIP_STATS == 1) */
}
