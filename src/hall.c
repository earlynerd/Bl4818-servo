/*
 * Hall Sensor Interface
 *
 * Reads three hall sensors and tracks rotor position via state transitions.
 *
 * Hall encoding (3 bits: CBA):
 *   State 1: 001  (0° electrical)
 *   State 3: 011  (60°)
 *   State 2: 010  (120°)
 *   State 6: 110  (180°)
 *   State 4: 100  (240°)
 *   State 5: 101  (300°)
 *
 * States 0 and 7 are invalid (all same = sensor fault).
 *
 * Pin assignments are provisional — update HALL_x_PIN in ms51_config.h
 * after probing the board. Halls may be on different ports (not all on P1).
 */

#include "ms51_reg.h"
#include "ms51_config.h"
#include "hall.h"

/* Forward rotation sequence: 1→3→2→6→4→5→1 */
static const uint8_t __code hall_forward_seq[8] = {
    0xFF, /* 0: invalid */
    3,    /* 1 → next is 3 */
    6,    /* 2 → next is 6 */
    2,    /* 3 → next is 2 */
    5,    /* 4 → next is 5 */
    1,    /* 5 → next is 1 */
    4,    /* 6 → next is 4 */
    0xFF  /* 7: invalid */
};

/* Hall state to electrical sector (0-5) mapping */
static const uint8_t __code hall_to_sector[8] = {
    0xFF, /* 0: invalid */
    0,    /* 1: 0° */
    2,    /* 2: 120° */
    1,    /* 3: 60° */
    4,    /* 4: 240° */
    5,    /* 5: 300° */
    3,    /* 6: 180° */
    0xFF  /* 7: invalid */
};

static uint8_t prev_hall;
static int8_t  detected_direction;
static int32_t hall_position;
static uint16_t transition_period;  /* Timer ticks between transitions */
static uint16_t last_transition_time;

void hall_init(void)
{
    /*
     * Configure hall sensor pins as inputs.
     * Since halls may be on different ports, configure each individually.
     * Pin mode: Px_M1=1, Px_M2=0 → input mode.
     *
     * TODO: Update these when actual hall pin assignments are confirmed.
     */

    /* Hall A = P0.5 (pin 1) — configure as input */
    P0M1 |=  0x20;   /* P0.5 M1=1 */
    P0M2 &= ~0x20;   /* P0.5 M2=0 */

    /* Hall B = P1.7 (pin 6) — configure as input */
    P1M1 |=  0x80;   /* P1.7 M1=1 */
    P1M2 &= ~0x80;   /* P1.7 M2=0 */

    /* Hall C = P3.0 (pin 5) — configure as input */
    P3M1 |=  0x01;   /* P3.0 M1=1 */
    P3M2 &= ~0x01;   /* P3.0 M2=0 */

    prev_hall = hall_read();
    detected_direction = 0;
    hall_position = 0;
    transition_period = 0xFFFF;  /* Unknown speed */
    last_transition_time = 0;
}

uint8_t hall_read(void)
{
    /*
     * Read hall sensors from individual pins (possibly on different ports).
     * Returns 3-bit value: bit2=C, bit1=B, bit0=A.
     */
    uint8_t state = 0;

    if (HALL_A_PIN) state |= 0x01;
    if (HALL_B_PIN) state |= 0x02;
    if (HALL_C_PIN) state |= 0x04;

    return state;
}

uint8_t hall_sector(void)
{
    uint8_t h = hall_read();
    return hall_to_sector[h];
}

int8_t hall_direction(void)
{
    return detected_direction;
}

uint16_t hall_period(void)
{
    return transition_period;
}

int32_t hall_count(void)
{
    return hall_position;
}

void hall_count_reset(void)
{
    hall_position = 0;
}

void hall_isr(void)
{
    uint8_t current = hall_read();

    if (current == prev_hall)
        return;  /* No change (debounce / noise) */

    /* Detect invalid states */
    if (current == 0 || current == 7) {
        /* Hall sensor fault — don't update */
        return;
    }

    /* Determine direction from state transition */
    if (hall_forward_seq[prev_hall] == current) {
        detected_direction = 1;
        hall_position++;
    } else if (hall_forward_seq[current] == prev_hall) {
        detected_direction = -1;
        hall_position--;
    }
    /* else: skipped state (fast transition) — direction unchanged */

    /* Calculate period using Timer 2 free-running counter */
    uint16_t now = ((uint16_t)TH2 << 8) | TL2;
    transition_period = now - last_transition_time;
    last_transition_time = now;

    prev_hall = current;
}
