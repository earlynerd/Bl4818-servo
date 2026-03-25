/*
 * UART Driver — Interrupt-driven with ring buffers
 *
 * UART0 on MS51FB9AE:
 *   TX = P0.6 (shared with ADC CH6 — current sense)
 *   RX = P0.7 (shared with ADC CH7 — voltage sense)
 *
 * When UART is enabled, ADC channels 6/7 are unavailable.
 * The firmware must choose: UART or full ADC.
 * In servo mode, UART is typically more important; current sensing
 * can use a single ADC channel with time-multiplexing.
 */

#include "ms51_reg.h"
#include "ms51_config.h"
#include "uart.h"

/* Ring buffers */
static volatile uint8_t tx_buf[UART_TX_BUF_SIZE];
static volatile uint8_t tx_head, tx_tail;
static volatile uint8_t tx_busy;

static volatile uint8_t rx_buf[UART_RX_BUF_SIZE];
static volatile uint8_t rx_head, rx_tail;

void uart_init(uint32_t baudrate)
{
    /*
     * UART0 in Mode 1 (8-bit, variable baud rate)
     * Baud rate generator: Timer 3
     *
     * Timer 3 reload value:
     *   Reload = 65536 - (FSYS / 4 / baudrate)
     *
     * For 115200 baud at 24MHz:
     *   Reload = 65536 - (24000000 / 4 / 115200) = 65536 - 52 = 65484
     */
    uint16_t reload = (uint16_t)(65536UL - (FSYS / 4UL / baudrate));

    /* Configure UART0 pins */
    /* P0.6 (TX) as push-pull output */
    P0M1 &= ~0x40;
    P0M2 |=  0x40;
    /* P0.7 (RX) as input */
    P0M1 |=  0x80;
    P0M2 &= ~0x80;

    /* SCON: Mode 1, REN=1 (receive enable) */
    SCON = 0x50;  /* SM0=0, SM1=1, REN=1 */

    /* Timer 3 as baud rate generator */
    T3CON = 0x08;  /* BRCK=1: Timer3 as UART0 baud rate clock */
    RH3 = (uint8_t)(reload >> 8);
    RL3 = (uint8_t)(reload & 0xFF);
    T3CON |= 0x04;  /* TR3=1: start Timer 3 */

    /* Clear buffers */
    tx_head = tx_tail = 0;
    rx_head = rx_tail = 0;
    tx_busy = 0;

    /* Enable UART0 interrupt */
    ES = 1;
}

void uart_putc(uint8_t c)
{
    uint8_t next = (tx_head + 1) % UART_TX_BUF_SIZE;

    /* Wait if buffer full */
    while (next == tx_tail)
        ;

    tx_buf[tx_head] = c;
    tx_head = next;

    /* Kick off transmission if idle */
    if (!tx_busy) {
        tx_busy = 1;
        SBUF = tx_buf[tx_tail];
        tx_tail = (tx_tail + 1) % UART_TX_BUF_SIZE;
    }
}

void uart_puts(const char *s)
{
    while (*s)
        uart_putc(*s++);
}

void uart_put_int(int32_t val)
{
    char buf[12];
    uint8_t i = 0;
    uint8_t neg = 0;

    if (val < 0) {
        neg = 1;
        val = -val;
    }

    if (val == 0) {
        uart_putc('0');
        return;
    }

    while (val > 0 && i < sizeof(buf)) {
        buf[i++] = '0' + (val % 10);
        val /= 10;
    }

    if (neg)
        uart_putc('-');

    while (i > 0)
        uart_putc(buf[--i]);
}

uint8_t uart_available(void)
{
    return (rx_head != rx_tail) ? 1 : 0;
}

int16_t uart_getc(void)
{
    if (rx_head == rx_tail)
        return -1;

    uint8_t c = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) % UART_RX_BUF_SIZE;
    return c;
}

/* UART0 ISR */
void uart_isr(void) __interrupt(INT_UART0)
{
    if (RI) {
        RI = 0;
        uint8_t next = (rx_head + 1) % UART_RX_BUF_SIZE;
        if (next != rx_tail) {  /* Not full */
            rx_buf[rx_head] = SBUF;
            rx_head = next;
        }
        /* else: overflow — drop byte */
    }

    if (TI) {
        TI = 0;
        if (tx_head != tx_tail) {
            SBUF = tx_buf[tx_tail];
            tx_tail = (tx_tail + 1) % UART_TX_BUF_SIZE;
        } else {
            tx_busy = 0;
        }
    }
}
