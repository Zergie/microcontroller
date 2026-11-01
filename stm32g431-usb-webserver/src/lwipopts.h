#ifndef LWIPOPTS_H
#define LWIPOPTS_H
#define NO_SYS 1
#define SYS_LIGHTWEIGHT_PROT 0
#ifndef MEM_ALIGNMENT
#define MEM_ALIGNMENT 4
#endif
#define MEM_SIZE 4096
#define MEMP_NUM_PBUF 4
#define PBUF_POOL_SIZE 4
#define PBUF_POOL_BUFSIZE 1536
#define MEMP_NUM_TCP_PCB 2
#define MEMP_NUM_TCP_PCB_LISTEN 1
#define MEMP_NUM_TCP_SEG 8
#define MEMP_NUM_UDP_PCB 1
#define MEMP_NUM_ARP_QUEUE 2
#define ARP_TABLE_SIZE 4
#define TCP_MSS 536
#define TCP_WND (2 * TCP_MSS)
#define TCP_SND_BUF (2 * TCP_MSS)
#define TCP_SND_QUEUELEN 8
#define TCP_QUEUE_OOSEQ 0
#define LWIP_IPV4 1
#define LWIP_IPV6 0
#define LWIP_ARP 1
#define LWIP_ETHERNET 1
#define LWIP_ICMP 1
#define LWIP_UDP 1
#define LWIP_TCP 1
#define LWIP_DHCP 0
#define LWIP_DNS 0
#define LWIP_RAW 0
#define LWIP_NETCONN 0
#define LWIP_SOCKET 0
#define LWIP_STATS 0
#define LWIP_SINGLE_NETIF 1
#define IP_REASSEMBLY 0
#define IP_FRAG 0
#define LWIP_NETIF_TX_SINGLE_PBUF 1
/* DHCP requests arrive from hosts without a configured IP address. */
#define LWIP_IP_ACCEPT_UDP_PORT(p) ((p) == PP_NTOHS(67))
#define LWIP_HTTPD_CUSTOM_FILES 1
#define HTTPD_USE_CUSTOM_FSDATA 1
#define HTTPD_FSDATA_FILE "empty_fsdata.h"
#define LWIP_HTTPD_DYNAMIC_HEADERS 0
#define LWIP_HTTPD_CGI 0
#define LWIP_HTTPD_SSI 0
#define LWIP_HTTPD_SUPPORT_POST 1
#define LWIP_HTTPD_FILE_EXTENSION 1
#define HTTP_IS_DATA_VOLATILE(hs) (((hs)->handle && (hs)->handle->pextension) ? TCP_WRITE_FLAG_COPY : 0)
#define LWIP_HTTPD_MAX_REQUEST_URI_LEN 64
#endif
