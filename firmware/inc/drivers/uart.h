/*
 * uart.h - UART driver API for USART3 (ST-LINK VCP)
 *
 * Per IMPROVE_FIRMWARE_PLAN.md:
 * - Opaque handle pattern
 * - status_t returns for all operations
 * - No blocking forever loops
 * - Proper volatile ISR synchronization
 *
 * Compliant with:
 * - MISRA-C:2012
 * - OpenBSD style guide (no typedefs for structs)
 * - DO-178C Level A requirements
 */

#ifndef UART_H
#define UART_H

#include <stdint.h>
#include <stddef.h>
#include "status.h"

/*
 * UART configuration structure
 */
struct uart_config {
	uint32_t baudrate;       /* Baud rate (e.g., 115200) */
	uint32_t kernel_hz;      /* Kernel clock frequency */
};

/*
 * UART handle (opaque to application)
 */
struct uart_handle;

/*
 * uart_init - Initialize UART peripheral
 *
 * @h: Pointer to UART handle
 * @cfg: Pointer to configuration structure
 *
 * Returns: STATUS_OK on success, error code on failure
 */
status_t uart_init(struct uart_handle *h, const struct uart_config *cfg);

/*
 * uart_deinit - Deinitialize UART peripheral
 *
 * @h: Pointer to UART handle
 *
 * Returns: STATUS_OK on success, error code on failure
 */
status_t uart_deinit(struct uart_handle *h);

/*
 * uart_write - Write data to UART
 *
 * This function is blocking but with timeout protection.
 *
 * @h: Pointer to UART handle
 * @data: Pointer to data buffer
 * @len: Number of bytes to write
 * @timeout_ms: Timeout in milliseconds (0 = non-blocking check only)
 *
 * Returns: STATUS_OK on success, error code on failure
 */
status_t uart_write(struct uart_handle *h, const uint8_t *data, size_t len,
                    uint32_t timeout_ms);

/*
 * uart_read - Read data from UART
 *
 * This function is non-blocking. It returns immediately with available data.
 *
 * @h: Pointer to UART handle
 * @data: Pointer to buffer to receive data
 * @len: Maximum number of bytes to read
 * @bytes_read: Pointer to receive actual number of bytes read
 *
 * Returns: STATUS_OK on success, error code on failure
 */
status_t uart_read(struct uart_handle *h, uint8_t *data, size_t len,
                   size_t *bytes_read);

/*
 * uart_write_string - Write null-terminated string to UART
 *
 * Convenience function for writing strings.
 *
 * @h: Pointer to UART handle
 * @str: Pointer to null-terminated string
 * @timeout_ms: Timeout in milliseconds
 *
 * Returns: STATUS_OK on success, error code on failure
 */
status_t uart_write_string(struct uart_handle *h, const char *str,
                           uint32_t timeout_ms);

/*
 * uart_printf - Formatted output to UART
 *
 * Simple printf implementation for UART. Maximum output: 128 bytes.
 *
 * @h: Pointer to UART handle
 * @fmt: Format string
 * @...: Variable arguments
 *
 * Returns: STATUS_OK on success, error code on failure
 */
status_t uart_printf(struct uart_handle *h, const char *fmt, ...);

/*
 * uart_get_rx_available - Get number of bytes available in RX buffer
 *
 * @h: Pointer to UART handle
 *
 * Returns: Number of bytes available
 */
size_t uart_get_rx_available(const struct uart_handle *h);

/*
 * uart_get_handle - Get UART3 handle
 *
 * Returns: Pointer to UART3 handle
 */
struct uart_handle *uart_get_handle(void);

/*
 * uart_get_rx_overflows - Get RX buffer overflow count
 *
 * @h: Pointer to UART handle
 *
 * Returns: Number of bytes dropped due to buffer full
 */
uint32_t uart_get_rx_overflows(const struct uart_handle *h);

#endif /* UART_H */
