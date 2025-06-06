#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__


#define TCPIP_THREAD_PRIO   30		//Move to top priority bellow Timer
#define TCPIP_THREAD_STACKSIZE 2048 //1024
#define DEFAULT_THREAD_STACKSIZE 1024
#define DEFAULT_RAW_RECVMBOX_SIZE 8
#define TCPIP_MBOX_SIZE 			4 //8 Reduced as don't think used
#define LWIP_TIMEVAL_PRIVATE 0

#define LWIP_SOCKET                 1
#define LWIP_SO_RCVBUF				1
#define RECV_BUFSIZE_DEFAULT		512 //256

#define MEM_LIBC_MALLOC             0

#define MEM_ALIGNMENT               4
#define MEM_SIZE                    10000 //12000 //4000
#define MEMP_NUM_TCP_SEG            32 // 64 //  32
#define MEMP_NUM_ARP_QUEUE          10
#define PBUF_POOL_SIZE              32 //32 //24
#define LWIP_ARP                    1
#define LWIP_ETHERNET               1
#define LWIP_ICMP                   1
#define LWIP_RAW                    1
#define TCP_WND                     (8 * TCP_MSS)
#define TCP_MSS                     1460
#define TCP_SND_BUF                 (8 * TCP_MSS)
#define TCP_SND_QUEUELEN            ((4 * (TCP_SND_BUF) + (TCP_MSS - 1)) / (TCP_MSS))
#define LWIP_NETIF_STATUS_CALLBACK  1
#define LWIP_NETIF_LINK_CALLBACK    1
#define LWIP_NETIF_HOSTNAME         1
#define LWIP_NETCONN                1
#define SYS_STATS                   0
#define LINK_STATS                  0
//#define ETH_PAD_SIZE                2  //Was turned off JMD
#define LWIP_CHKSUM_ALGORITHM       3
#define LWIP_DHCP                   1
#define LWIP_IPV4                   1
#define LWIP_TCP                    1
#define LWIP_UDP                    1
#define LWIP_DNS                    1
#define LWIP_TCP_KEEPALIVE          1
#define LWIP_NETIF_TX_SINGLE_PBUF   1
#define DHCP_DOES_ARP_CHECK         1
#define LWIP_DHCP_DOES_ACD_CHECK    0

//DNS settings
#define DNS_TABLE_SIZE      6
#define DNS_MAX_REQUESTS    4

//UDP settings
#define MEMP_NUM_UDP_PCB         4   // DNS and SNTP
#define MEMP_NUM_TCP_PCB         8   // MQTT/TCP clients
#define MEMP_NUM_TCP_PCB_LISTEN  2   // Only if you're listening
#define MEMP_NUM_NETBUF          8   // For netconn buffers
#define MEMP_NUM_NETCONN         8   // Needed for sockets/netconn


#ifndef NDEBUG
#define LWIP_DEBUG                  0
#define LWIP_STATS                  0
#define LWIP_STATS_DISPLAY	0
#endif


//LWIP Tuning configuration
#ifndef LWIP_TUNE
#define LWIP_STATS                  0
#define LWIP_STATS_DISPLAY          0
#define MEM_STATS                   0
#define MEMP_STATS                  0
#else
#define LWIP_STATS                  1
#define LWIP_STATS_DISPLAY          1
#define MEM_STATS                   1
#define MEMP_STATS                  1
#endif //LWIP_TUNE

#define ETHARP_DEBUG                LWIP_DBG_OFF
#define NETIF_DEBUG                 LWIP_DBG_OFF
#define PBUF_DEBUG                  LWIP_DBG_OFF
#define API_LIB_DEBUG               LWIP_DBG_OFF
#define API_MSG_DEBUG               LWIP_DBG_OFF
#define SOCKETS_DEBUG               LWIP_DBG_OFF
#define ICMP_DEBUG                  LWIP_DBG_OFF
#define INET_DEBUG                  LWIP_DBG_OFF
#define IP_DEBUG                    LWIP_DBG_OFF
#define IP_REASS_DEBUG              LWIP_DBG_OFF
#define RAW_DEBUG                   LWIP_DBG_OFF
#define MEM_DEBUG                   LWIP_DBG_OFF
#define MEMP_DEBUG                  LWIP_DBG_OFF
#define SYS_DEBUG                   LWIP_DBG_OFF
#define TCP_DEBUG                   LWIP_DBG_OFF
#define TCP_INPUT_DEBUG             LWIP_DBG_OFF
#define TCP_OUTPUT_DEBUG            LWIP_DBG_OFF
#define TCP_RTO_DEBUG               LWIP_DBG_OFF
#define TCP_CWND_DEBUG              LWIP_DBG_OFF
#define TCP_WND_DEBUG               LWIP_DBG_OFF
#define TCP_FR_DEBUG                LWIP_DBG_OFF
#define TCP_QLEN_DEBUG              LWIP_DBG_OFF
#define TCP_RST_DEBUG               LWIP_DBG_OFF
#define UDP_DEBUG                   LWIP_DBG_OFF
#define TCPIP_DEBUG                 LWIP_DBG_OFF
#define PPP_DEBUG                   LWIP_DBG_OFF
#define SLIP_DEBUG                  LWIP_DBG_OFF
#define DHCP_DEBUG                  LWIP_DBG_OFF


// #define ETHARP_DEBUG                LWIP_DBG_ON//LWIP_DBG_OFF
// #define NETIF_DEBUG                 LWIP_DBG_ON//LWIP_DBG_OFF
// #define PBUF_DEBUG                 LWIP_DBG_ON// LWIP_DBG_OFF
// #define API_LIB_DEBUG              LWIP_DBG_ON// LWIP_DBG_ON//LWIP_DBG_OFF
// #define API_MSG_DEBUG              LWIP_DBG_ON// LWIP_DBG_OFF
// #define SOCKETS_DEBUG              LWIP_DBG_ON// LWIP_DBG_OFF
// #define ICMP_DEBUG                 LWIP_DBG_ON// LWIP_DBG_OFF
// #define INET_DEBUG                 LWIP_DBG_ON// LWIP_DBG_OFF
// #define IP_DEBUG                   LWIP_DBG_ON// LWIP_DBG_OFF
// #define IP_REASS_DEBUG             LWIP_DBG_ON// LWIP_DBG_OFF
// #define RAW_DEBUG                  LWIP_DBG_ON// LWIP_DBG_OFF
// #define MEM_DEBUG                  LWIP_DBG_ON// LWIP_DBG_OFF
// #define MEMP_DEBUG                 LWIP_DBG_ON// LWIP_DBG_OFF
// #define SYS_DEBUG                  LWIP_DBG_ON// LWIP_DBG_OFF
// #define TCP_DEBUG                   LWIP_DBG_ON//LWIP_DBG_OFF
// #define TCP_INPUT_DEBUG            LWIP_DBG_ON// LWIP_DBG_OFF
// #define TCP_OUTPUT_DEBUG           LWIP_DBG_ON// LWIP_DBG_OFF
// #define TCP_RTO_DEBUG              LWIP_DBG_ON// LWIP_DBG_OFF
// #define TCP_CWND_DEBUG             LWIP_DBG_ON// LWIP_DBG_OFF
// #define TCP_WND_DEBUG              LWIP_DBG_ON// LWIP_DBG_OFF
// #define TCP_FR_DEBUG               LWIP_DBG_ON// LWIP_DBG_OFF
// #define TCP_QLEN_DEBUG             LWIP_DBG_ON// LWIP_DBG_OFF
// #define TCP_RST_DEBUG              LWIP_DBG_ON// LWIP_DBG_OFF
// #define UDP_DEBUG                  LWIP_DBG_ON// LWIP_DBG_OFF
// #define TCPIP_DEBUG                LWIP_DBG_ON// LWIP_DBG_OFF
// #define PPP_DEBUG                  LWIP_DBG_ON// LWIP_DBG_OFF
// #define SLIP_DEBUG                 LWIP_DBG_ON// LWIP_DBG_OFF
// #define DHCP_DEBUG                 LWIP_DBG_ON// LWIP_DBG_OFF


#define DEFAULT_TCP_RECVMBOX_SIZE 128

//SNTP NETWORK TIME
#include "pico/stdlib.h"
#define SNTP_SUPPORT      1
#define SNTP_SERVER_DNS   1
void sntpSetTimeSec(uint32_t sec);
#define SNTP_SET_SYSTEM_TIME(sec) sntpSetTimeSec(sec)
//MEMP_NUM_SYS_TIMEOUTS Needs to be one larger than default for SNTP
#define MEMP_NUM_SYS_TIMEOUT            (LWIP_NUM_SYS_TIMEOUT_INTERNAL + 20)

//Once an hour
#define SNTP_UPDATE_DELAY 60000*60


#define MEM_USE_POOLS 0 //1
#define MEMP_USE_CUSTOM_POOLS 0 //1

//MQTT
#define MQTT_OUTPUT_RINGBUF_SIZE   2048  // Or larger, depending on your messages



#endif /* __LWIPOPTS_H__ */
