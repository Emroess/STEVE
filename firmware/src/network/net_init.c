/**
  * @file    app_ethernet.c
  * @author  STEVE firmware team
  * @brief   Ethernet initialization for bare-metal STM32H7
  */

/* Includes ------------------------------------------------------------------*/
#include "lwip/opt.h"
#include "lwip/mem.h"
#include "lwip/memp.h"
#include "lwip/tcp.h"
#include "lwip/udp.h"
#include "lwip/init.h"
#include "netif/etharp.h"
#include "ethernetif.h"
#include "network/net_init.h"
#include "network/http.h"
#include "board.h"
#include "drivers/uart.h"
#include "network/nvm.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
struct netif gnetif;

/* Private function prototypes -----------------------------------------------*/
void ethernet_link_status_updated(struct netif *netif);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Notify the User about the network interface config status
  * @param  netif: the network interface
  * @retval None
  */
void ethernet_link_status_updated(struct netif *netif)
{
  if (netif_is_up(netif))
  {
    uart_write_string(NULL, "\r\nEthernet link up\r\n", 100);
  }
  else
  {
    uart_write_string(NULL, "\r\nEthernet link down\r\n", 100);
  }
}

/**
  * @brief  Ethernet link thread (simplified for bare-metal)
  * @param  argument: not used
  * @retval None
  */
void ethernet_link_thread(void const * argument)
{
  (void)argument;  /* Suppress unused parameter warning */
  ip4_addr_t ipaddr, netmask, gw;

  /* Try loading from NVM first */
  struct network_nvm_config config;
  bool loaded = network_nvm_load(&config);
  if (loaded && config.magic == NETWORK_NVM_MAGIC && config.version == NETWORK_NVM_VERSION) {
    /* Validate checksum */
    if (network_nvm_calculate_checksum(&config) == config.checksum) {
      /* Use NVM values */
      ip4addr_aton(config.ip_addr, &ipaddr);
      ip4addr_aton(config.netmask, &netmask);
      ip4addr_aton(config.gateway, &gw);
      /* Log success */
      struct uart_handle *uart = uart_get_handle();
      if (uart) {
        uart_write_string(uart, "\r\nLoaded network config from NVM\r\n", 100);
      }
    } else {
      loaded = false;  /* Checksum mismatch */
      struct uart_handle *uart = uart_get_handle();
      if (uart) {
        uart_write_string(uart, "\r\nNVM checksum mismatch\r\n", 100);
      }
    }
  } else {
    loaded = false;
    struct uart_handle *uart = uart_get_handle();
    if (uart) {
      if (!loaded) {
        uart_write_string(uart, "\r\nNVM load failed\r\n", 100);
      } else if (config.magic != NETWORK_NVM_MAGIC) {
        uart_write_string(uart, "\r\nNVM magic invalid\r\n", 100);
      } else if (config.version != NETWORK_NVM_VERSION) {
        uart_write_string(uart, "\r\nNVM version invalid\r\n", 100);
      }
    }
  }

  if (!loaded) {
    /* Fallback to hardcoded defaults */
    IP4_ADDR(&ipaddr, 10, 0, 1, 17);
    IP4_ADDR(&netmask, 255, 255, 255, 0);
    IP4_ADDR(&gw, 10, 0, 1, 1);
    /* Log fallback */
    struct uart_handle *uart = uart_get_handle();
    if (uart) {
      uart_write_string(uart, "\r\nUsing default network config\r\n", 100);
    }
  }

  /* Add the network interface */
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &ethernet_input);

  /* Ensure IP is set correctly (in case init function overwrites it) */
  netif_set_addr(&gnetif, &ipaddr, &netmask, &gw);

  /* Registers the default network interface */
  netif_set_default(&gnetif);

  if (netif_is_link_up(&gnetif))
  {
    /* When the netif is fully configured this function must be called */
    netif_set_up(&gnetif);
    ethernet_link_status_updated(&gnetif);
  }
  else
  {
    /* When the netif link is down this function must be called */
    netif_set_down(&gnetif);
    uart_write_string(NULL, "\r\nEthernet link down (PHY not connected)\r\n", 100);
  }

  /* Set the link callback function, this function is called on change of link status */
  netif_set_link_callback(&gnetif, ethernet_link_status_updated);
}

/**
  * @brief  Initialize the Ethernet
  * @param  None
  * @retval None
  */
void ethernet_init(void)
{
  /* Initialize the LwIP stack */
  lwip_init();

  /* Configure the Network interface */
  ethernet_link_thread(NULL);

  /* Initialize HTTP server */
  ethernet_http_init();
}

/**
  * @brief  Ethernet process (called periodically)
  * @param  None
  * @retval None
  */
void ethernet_process(void)
{
  static uint32_t last_link_check_ms = 0;
  uint32_t now = board_get_systick_ms();

  /* Process any received Ethernet packets */
  ethernetif_input(&gnetif);

  /* Periodically poll PHY/link status to bring the interface up/down */
  if ((now - last_link_check_ms) >= 500U) {
    ethernet_link_check_state(&gnetif);
    last_link_check_ms = now;
  }
}
