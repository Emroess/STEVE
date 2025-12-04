/**
  * @file    network/http.h
  * @author  STEVE firmware team
  * @brief   HTTP server infrastructure for valve configuration and control
  */

#ifndef NETWORK_HTTP_H
#define NETWORK_HTTP_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialize the HTTP server on port 8080
 * @return true if successful, false otherwise
 */
bool ethernet_http_init(void);

/**
 * @brief Stop the HTTP server
 */
void ethernet_http_stop(void);

/**
 * @brief Enable or disable HTTP logging to UART
 * @param enable true to enable logging, false to disable
 */
void ethernet_http_enable_logging(bool enable);

/**
 * @brief Get current HTTP logging state
 * @return true if logging is enabled, false otherwise
 */
bool ethernet_http_get_logging_enabled(void);

/**
 * @brief Get HTTP server status information
 * @param[out] running true if server is running
 * @param[out] active_connections number of active connections
 * @param[out] logging_enabled true if logging is enabled
 */
void ethernet_http_get_status(bool *running, uint32_t *active_connections, bool *logging_enabled);

/**
 * @brief Process HTTP server tasks (if any)
 *        Note: lwIP callbacks handle most work, this is for periodic tasks
 */
void ethernet_http_process(void);

#endif /* NETWORK_HTTP_H */