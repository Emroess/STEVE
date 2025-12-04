/**
  * @file    net_init.h
  * @author  STEVE firmware team
  * @brief   Network initialization and Ethernet processing
  */

#ifndef NET_INIT_H
#define NET_INIT_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "lwip/netif.h"
#include "config/network.h"

/* DHCP states are now in config/network.h */

void ethernet_link_status_updated(struct netif *netif);
void ethernet_init(void);
void ethernet_process(void);

#ifdef __cplusplus
}
#endif

#endif /* NET_INIT_H */




