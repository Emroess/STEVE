/**
  * @file    app_ethernet.h
  * @author  STEVE firmware team
  * @brief   Header for app_ethernet.c module
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_ETHERNET_H
#define __APP_ETHERNET_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "lwip/netif.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* DHCP process states */
#define DHCP_OFF                   (uint8_t) 0
#define DHCP_START                 (uint8_t) 1
#define DHCP_WAIT_ADDRESS          (uint8_t) 2
#define DHCP_ADDRESS_ASSIGNED      (uint8_t) 3
#define DHCP_TIMEOUT               (uint8_t) 4
#define DHCP_LINK_DOWN             (uint8_t) 5

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void ethernet_link_status_updated(struct netif *netif);
void ethernet_init(void);
void ethernet_process(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_ETHERNET_H */




