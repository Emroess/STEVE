/**
  * @file    ethernet_stream.h
  * @author  STEVE firmware team
  * @brief   Header for ethernet_stream.c
  */

#ifndef __ETHERNET_STREAM_H
#define __ETHERNET_STREAM_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
 extern "C" {
#endif

#define ETHERNET_STREAM_MIN_INTERVAL_MS      10U
#define ETHERNET_STREAM_MAX_INTERVAL_MS      1000U
#define ETHERNET_STREAM_DEFAULT_INTERVAL_MS  100U

typedef struct {
  uint8_t active;
  uint8_t connected_clients;
  uint32_t default_interval_ms;
  uint32_t messages_sent;
  uint32_t send_errors;
  uint32_t invalid_samples;
  uint32_t last_send_tick_ms;
} ethernet_stream_stats_t;

/* Exported functions ------------------------------------------------------- */
bool ethernet_stream_init(void);
void ethernet_stream_stop(void);
void ethernet_stream_process(void);
uint32_t ethernet_stream_set_default_interval(uint32_t interval_ms);
void ethernet_stream_get_stats(ethernet_stream_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif /* __ETHERNET_STREAM_H */
