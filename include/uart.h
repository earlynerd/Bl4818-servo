/*
 * UART1 Driver
 *
 * RX uses an interrupt-backed ring buffer when global interrupts are enabled.
 * If interrupts are disabled, the driver falls back to direct polling so early
 * boot code and bench utilities still work.
 */
#ifndef UART_H
#define UART_H

#include <stdint.h>

#define UART_RX_BUF_SIZE  128u
#define UART_TX_BUF_SIZE  128u

/* Initialize UART1 at specified baud rate */
void uart_init(uint32_t baudrate);

/* Queue a single byte for transmission, blocking only if the TX buffer is full */
void uart_putc(uint8_t c);

/* Try to queue one TX byte; returns 1 on success, 0 if the buffer is full */
uint8_t uart_try_putc(uint8_t c);

/* Send a null-terminated string */
void uart_puts(const char *s);

/* Send a decimal number as ASCII */
void uart_put_int(int32_t val);

/* Return the number of bytes ready to read */
uint8_t uart_available(void);

/* Read a byte from receive buffer (-1 if empty) */
int16_t uart_getc(void);

/* Drop all buffered RX data and clear any pending hardware RX flag */
void uart_rx_flush(void);

/* Returns nonzero if the RX ring buffer has overflowed since the last clear */
uint8_t uart_rx_overflowed(void);

/* Clear the RX overflow indicator */
void uart_rx_clear_overflow(void);

/* Returns nonzero while bytes are still queued or actively transmitting */
uint8_t uart_tx_busy(void);

/* Block until all queued TX bytes have reached the UART shifter */
void uart_tx_flush(void);

#endif /* UART_H */
