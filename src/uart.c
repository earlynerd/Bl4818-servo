/*
 * UART1 Driver for MS51FB9AE
 *
 * Uses UART1 (not UART0, which shares pins with ADC):
 *   TX = P1.6 (pin 8, prog header "P" / ICE_DAT)
 *   RX = P0.2 (pin 18, prog header "S" / ICE_CLK)
 *
 * Baud rate from Timer 3 (UART1's only clock source).
 * Currently polled TX for bring-up simplicity.
 */

#include "ms51_reg.h"
#include "ms51_config.h"
#include "uart.h"

void uart_init(uint32_t baudrate)
{
    /*
     * UART1 Mode 1, Timer 3 as baud rate generator.
     * Following TRM Section 6.8.2.2 and sample code (page 269).
     *
     * Baud = FSYS / (16 × Pre-scale × (65536 - [RH3:RL3]))
     *   with SMOD_1=1 and Pre-scale=1 (T3PS=000).
     *
     * For 115200 @ 24MHz: reload = 65536 - 13 = 65523 = 0xFFF3
     * Actual = 24000000 / (16 × 13) = 115384.6 (0.16% error)
     *
     * T3CON bit layout (C4H, page 0):
     *   [7] SMOD_1  [6] SMOD0_1  [5] BRCK  [4] TF3
     *   [3] TR3     [2:0] T3PS
     */
    /* Round to nearest integer instead of truncating */
    uint16_t reload = (uint16_t)(65536UL - ((FSYS / 16UL + baudrate / 2) / baudrate));

    /* Configure UART1 pins */
    P1M1 &= ~0x40;  /* P1.6 (TX) push-pull output */
    P1M2 |=  0x40;
    P0M1 |=  0x04;  /* P0.2 (RX) input */
    P0M2 &= ~0x04;

    /* UART1 Mode 1, REN=1, TI_1=1 (per Nuvoton sample) */
    SCON_1 = 0x52;

    /* Timer 3: SMOD_1=1 (double baud rate), T3PS=000 (prescale=1) */
    T3CON = 0x80;
    RH3 = (uint8_t)(reload >> 8);
    RL3 = (uint8_t)(reload & 0xFF);
    T3CON |= 0x08;  /* TR3=1 (bit 3): start Timer 3 */
}

void uart_putc(uint8_t c)
{
    /* Polled TX: wait for transmitter ready, then send */
    while (!TI_1)
        ;
    TI_1 = 0;
    SBUF_1 = c;
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
    return RI_1 ? 1 : 0;
}

int16_t uart_getc(void)
{
    if (!RI_1)
        return -1;
    RI_1 = 0;
    return SBUF_1;
}
