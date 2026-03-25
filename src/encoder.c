/*
 * Quadrature Encoder Interface
 *
 * Reads incremental encoder signals on:
 *   Channel A: P3.0
 *   Channel B: P0.1 (replaces PWM input in servo mode)
 *
 * Uses pin-change interrupt on Channel A (rising + falling edges).
 * Channel B is read at each edge to determine direction.
 * This gives 2x decoding (both edges of A, single edge of B).
 * For 4x decoding, add interrupt on Channel B as well.
 */

#include "ms51_reg.h"
#include "ms51_config.h"
#include "encoder.h"

static volatile int32_t enc_position;
static volatile int16_t enc_velocity;
static int32_t enc_prev_position;

void encoder_init(void)
{
    /* Configure P3.0 (Encoder A) as input */
    P3M1 |=  0x01;
    P3M2 &= ~0x01;

    /* Configure P0.1 (Encoder B) as input */
    P0M1 |=  0x02;
    P0M2 &= ~0x02;

    enc_position = 0;
    enc_velocity = 0;
    enc_prev_position = 0;

    /*
     * Pin interrupt on P3.0 for encoder A.
     * The MS51 supports pin interrupts via PICON/PINEN/PIPEN registers.
     * We configure for both-edge trigger on the encoder A pin.
     *
     * For polling-based approach (simpler, works from control loop):
     * The hall ISR or timer ISR can call encoder_isr() to sample.
     */
}

int32_t encoder_get_position(void)
{
    int32_t pos;
    EA = 0;  /* Disable interrupts for atomic read */
    pos = enc_position;
    EA = 1;
    return pos;
}

void encoder_set_position(int32_t counts)
{
    EA = 0;
    enc_position = counts;
    EA = 1;
}

int16_t encoder_get_velocity(void)
{
    return enc_velocity;
}

/*
 * Called from timer ISR or pin-change ISR.
 * Implements 2x quadrature decoding on channel A edges.
 */
static uint8_t prev_a;

void encoder_isr(void)
{
    uint8_t a = (P3 & 0x01);       /* P3.0 = Encoder A */
    uint8_t b = (P0 & 0x02) >> 1;  /* P0.1 = Encoder B */

    if (a != prev_a) {
        /* Edge detected on A */
        if (a ^ b) {
            enc_position++;
        } else {
            enc_position--;
        }
        prev_a = a;
    }
}

/*
 * Called once per control period to compute velocity.
 * Velocity = position change since last call.
 */
void encoder_update_velocity(void)
{
    int32_t pos;
    EA = 0;
    pos = enc_position;
    EA = 1;

    enc_velocity = (int16_t)(pos - enc_prev_position);
    enc_prev_position = pos;
}
