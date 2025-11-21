#ifndef LWIP_PING_H
#define LWIP_PING_H

#include "lwip/ip_addr.h"
#include <stdint.h>

struct uart_handle;

/**
 * PING_USE_SOCKETS: Set to 1 to use sockets, otherwise the raw api is used
 */
#ifndef PING_USE_SOCKETS
#define PING_USE_SOCKETS    LWIP_SOCKET
#endif

void ping_init(const ip_addr_t* ping_addr);
void ping_stop(void);
void ping_set_uart(struct uart_handle *uart);
void ping_set_count(u16_t count);
uint16_t ping_get_sent_count(void);
uint16_t ping_get_recv_count(void);
uint8_t ping_is_active(void);

#if !PING_USE_SOCKETS
void ping_send_now(void);
#endif /* !PING_USE_SOCKETS */

#endif /* LWIP_PING_H */
