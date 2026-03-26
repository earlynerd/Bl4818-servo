/*
 * UART Driver — Interrupt-driven TX/RX with ring buffers
 */
#ifndef UART_H
#define UART_H

#include <stdint.h>

#define UART_TX_BUF_SIZE    64
#define UART_RX_BUF_SIZE    64

/* Initialize UART0 at specified baud rate */
void uart_init(uint32_t baudrate);

/* Send a single byte (blocks if buffer full) */
void uart_putc(uint8_t c);

/* Send a null-terminated string */
void uart_puts(const char *s);

/* Send a decimal number as ASCII */
void uart_put_int(int32_t val);

/* Check if data available in receive buffer */
uint8_t uart_available(void);

/* Read a byte from receive buffer (-1 if empty) */
int16_t uart_getc(void);

#endif /* UART_H */
