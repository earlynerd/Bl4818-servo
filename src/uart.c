/*
 * UART1 Driver for MS51FB9AE
 *
 * Uses UART1 (not UART0, which shares pins with ADC):
 *   TX = P1.6 (pin 8, prog header "P" / ICE_DAT)
 *   RX = P0.2 (pin 18, prog header "S" / ICE_CLK)
 *
 * Baud rate from Timer 3 (UART1's only clock source).
 *
 * RX uses an interrupt-backed ring buffer to avoid dropping bytes while the
 * control loop runs. TX also uses a ring buffer when interrupts are enabled,
 * so callers do not busy-wait on each transmitted byte. If global interrupts
 * are disabled, the driver falls back to direct polled TX/RX so the bench
 * image and early boot code still work.
 */

#include "ms51_reg.h"
#include "ms51_config.h"
#include "uart.h"

#if (UART_RX_BUF_SIZE < 2u) || (UART_RX_BUF_SIZE > 256u) || ((UART_RX_BUF_SIZE & (UART_RX_BUF_SIZE - 1u)) != 0u)
#error "UART_RX_BUF_SIZE must be a power of two in the range 2..256."
#endif

#if (UART_TX_BUF_SIZE < 2u) || (UART_TX_BUF_SIZE > 256u) || ((UART_TX_BUF_SIZE & (UART_TX_BUF_SIZE - 1u)) != 0u)
#error "UART_TX_BUF_SIZE must be a power of two in the range 2..256."
#endif

#define UART1_INT_ENABLE_MASK  0x01u
#define UART_RX_BUF_MASK       (UART_RX_BUF_SIZE - 1u)
#define UART_TX_BUF_MASK       (UART_TX_BUF_SIZE - 1u)

static volatile uint8_t __xdata rx_buf[UART_RX_BUF_SIZE];
static volatile uint8_t rx_head;
static volatile uint8_t rx_tail;
static volatile uint8_t rx_overflow;

static volatile uint8_t __xdata tx_buf[UART_TX_BUF_SIZE];
static volatile uint8_t tx_head;
static volatile uint8_t tx_tail;
static volatile uint8_t tx_running;

static uint8_t uart_irq_mode_active(void)
{
    return (EA && (EIE1 & UART1_INT_ENABLE_MASK)) ? 1u : 0u;
}

static void uart_tx_start_next_locked(void)
{
    if (tx_tail != tx_head) {
        uint8_t c = tx_buf[tx_tail];
        tx_tail = (uint8_t)((tx_tail + 1u) & UART_TX_BUF_MASK);
        tx_running = 1;
        SBUF_1 = c;
    } else {
        tx_running = 0;
    }
}

static void uart_tx_service_poll(void)
{
    if (tx_running && TI_1) {
        TI_1 = 0;
        uart_tx_start_next_locked();
    }
}

void uart_init(uint32_t baudrate)
{
    /*
     * UART1 Mode 1, Timer 3 as baud rate generator.
     * Following TRM Section 6.8.2.2 and sample code (page 269).
     *
     * Baud = FSYS / (16 × Pre-scale × (65536 - [RH3:RL3]))
     *   with SMOD_1=1 and Pre-scale=1 (T3PS=000).
     *
     * For 250000 @ 24MHz: reload = 65536 - 6 = 65530 = 0xFFFA
     * Actual = 24000000 / (16 × 6) = 250000.0 (0.00% error)
     *
     * T3CON bit layout (C4H, page 0):
     *   [7] SMOD_1  [6] SMOD0_1  [5] BRCK  [4] TF3
     *   [3] TR3     [2:0] T3PS
     */
    uint16_t reload = (uint16_t)(65536UL - ((FSYS / 16UL + baudrate / 2) / baudrate));

    rx_head = 0;
    rx_tail = 0;
    rx_overflow = 0;
    tx_head = 0;
    tx_tail = 0;
    tx_running = 0;

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

    /* Enable UART1 interrupt source; actual delivery still depends on EA. */
    EIE1 |= UART1_INT_ENABLE_MASK;
}

uint8_t uart_try_putc(uint8_t c)
{
    if (!uart_irq_mode_active()) {
        uart_tx_service_poll();
        if (tx_running)
            return 0;

        tx_running = 1;
        TI_1 = 0;
        SBUF_1 = c;
        return 1;
    }

    {
        uint8_t saved_ea = EA;
        uint8_t next;

        EA = 0;

        next = (uint8_t)((tx_head + 1u) & UART_TX_BUF_MASK);
        if (next == tx_tail) {
            EA = saved_ea;
            return 0;
        }

        tx_buf[tx_head] = c;
        tx_head = next;

        if (!tx_running) {
            TI_1 = 0;
            uart_tx_start_next_locked();
        }

        EA = saved_ea;
    }

    return 1;
}

void uart_putc(uint8_t c)
{
    while (!uart_try_putc(c)) {
        if (!uart_irq_mode_active())
            uart_tx_service_poll();
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
    uint8_t head = rx_head;
    uint8_t tail = rx_tail;
    uint8_t count = (uint8_t)((head - tail) & UART_RX_BUF_MASK);

    if (count != 0u)
        return count;

    return (!uart_irq_mode_active() && RI_1) ? 1u : 0u;
}

int16_t uart_getc(void)
{
    uint8_t tail = rx_tail;

    if (tail != rx_head) {
        uint8_t c = rx_buf[tail];
        rx_tail = (uint8_t)((tail + 1u) & UART_RX_BUF_MASK);
        return c;
    }

    if (!uart_irq_mode_active() && RI_1) {
        RI_1 = 0;
        return SBUF_1;
    }

    return -1;
}

void uart_rx_flush(void)
{
    uint8_t saved_ea = EA;

    EA = 0;
    rx_head = 0;
    rx_tail = 0;
    rx_overflow = 0;
    RI_1 = 0;
    EA = saved_ea;
}

uint8_t uart_rx_overflowed(void)
{
    return rx_overflow;
}

void uart_rx_clear_overflow(void)
{
    rx_overflow = 0;
}

uint8_t uart_tx_busy(void)
{
    return (tx_running || (tx_head != tx_tail)) ? 1u : 0u;
}

void uart_tx_flush(void)
{
    while (uart_tx_busy()) {
        if (!uart_irq_mode_active())
            uart_tx_service_poll();
    }
}

void uart1_isr(void) __interrupt(INT_UART1)
{
    if (RI_1) {
        uint8_t c = SBUF_1;
        uint8_t next = (uint8_t)((rx_head + 1u) & UART_RX_BUF_MASK);

        RI_1 = 0;

        if (next != rx_tail) {
            rx_buf[rx_head] = c;
            rx_head = next;
        } else {
            rx_overflow = 1;
        }
    }

    if (TI_1) {
        TI_1 = 0;
        uart_tx_start_next_locked();
    }
}
