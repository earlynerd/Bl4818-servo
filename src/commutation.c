/*
 * Six-Step Commutation — Low-Side-Only PWM Chopping
 *
 * Maps hall sensor state + direction to PWM mask registers (PMEN/PMD).
 *
 * Strategy: Only the sink phase low-side FET switches at PWM rate.
 * High-side FETs are held DC ON or OFF via masks, switching only at
 * commutation transitions (~500 Hz at 1000 RPM).
 *
 * This avoids shoot-through caused by the slow high-side P-FET turn-off
 * (1.5k pullup through gate capacitance) which cannot keep up with
 * 20 kHz complementary switching.  Low-side N-FETs turn on/off fast
 * through 100 ohm direct gate drive.
 *
 * For each commutation step:
 *   Source phase: high-side MASKED ON,  low-side MASKED OFF
 *   Sink phase:   high-side MASKED OFF, low-side UNMASKED (follows PWM)
 *   Float phase:  both MASKED OFF
 *
 * Channel bit mapping in PMEN/PMD registers (bits 5:0):
 *   Bit 0: CH0 = Phase U low-side  (P1.2)
 *   Bit 1: CH1 = Phase U high-side (P1.1)
 *   Bit 2: CH2 = Phase V high-side (P1.0)
 *   Bit 3: CH3 = Phase V low-side  (P0.0)
 *   Bit 4: CH4 = Phase W low-side  (P0.1)
 *   Bit 5: CH5 = Phase W high-side (P0.3)
 *
 * PMEN: 1=masked (forced to PMD), 0=follows PWM generator
 * PMD:  output value when masked (0=off, 1=on)
 */

#include "ms51_reg.h"
#include "ms51_config.h"
#include "commutation.h"
#include "hall.h"
#include "pwm.h"

/*
 * Forward commutation tables — low-side-only chopping.
 *
 * Each step has one high-side masked ON (source), one low-side
 * unmasked to follow PWM (sink), everything else masked OFF.
 *
 * Hall  Sector  Source(DC) Sink(PWM) PMEN    PMD
 * ────  ──────  ────────── ───────── ──────  ──────
 *  1     0°     U_HI       V_LO      0x37    0x02
 *  3     60°    U_HI       W_LO      0x2F    0x02
 *  2     120°   V_HI       W_LO      0x2F    0x04
 *  6     180°   V_HI       U_LO      0x3E    0x04
 *  4     240°   W_HI       U_LO      0x3E    0x20
 *  5     300°   W_HI       V_LO      0x37    0x20
 *
 * Hall=1: U_HI on (CH1 masked HIGH), V_LO PWM (CH3 unmasked)
 *   PMEN = 0b110111 = 0x37, PMD = 0b000010 = 0x02
 *
 * Hall=3: U_HI on (CH1 masked HIGH), W_LO PWM (CH4 unmasked)
 *   PMEN = 0b101111 = 0x2F, PMD = 0b000010 = 0x02
 *
 * Hall=2: V_HI on (CH2 masked HIGH), W_LO PWM (CH4 unmasked)
 *   PMEN = 0b101111 = 0x2F, PMD = 0b000100 = 0x04
 *
 * Hall=6: V_HI on (CH2 masked HIGH), U_LO PWM (CH0 unmasked)
 *   PMEN = 0b111110 = 0x3E, PMD = 0b000100 = 0x04
 *
 * Hall=4: W_HI on (CH5 masked HIGH), U_LO PWM (CH0 unmasked)
 *   PMEN = 0b111110 = 0x3E, PMD = 0b100000 = 0x20
 *
 * Hall=5: W_HI on (CH5 masked HIGH), V_LO PWM (CH3 unmasked)
 *   PMEN = 0b110111 = 0x37, PMD = 0b100000 = 0x20
 */
static const uint8_t __code fwd_pmen[8] = {
    0x3F, /* 0: invalid — all masked off */
    0x37, /* 1: V_LO (CH3) follows PWM */
    0x2F, /* 2: W_LO (CH4) follows PWM */
    0x2F, /* 3: W_LO (CH4) follows PWM */
    0x3E, /* 4: U_LO (CH0) follows PWM */
    0x37, /* 5: V_LO (CH3) follows PWM */
    0x3E, /* 6: U_LO (CH0) follows PWM */
    0x3F  /* 7: invalid — all masked off */
};

static const uint8_t __code fwd_pmd[8] = {
    0x00, /* 0: invalid — all off */
    0x02, /* 1: U_HI on (CH1) */
    0x04, /* 2: V_HI on (CH2) */
    0x02, /* 3: U_HI on (CH1) */
    0x20, /* 4: W_HI on (CH5) */
    0x20, /* 5: W_HI on (CH5) */
    0x04, /* 6: V_HI on (CH2) */
    0x00  /* 7: invalid — all off */
};

/*
 * Reverse commutation — swap source/sink roles to reverse torque.
 *
 * Hall  Source(DC) Sink(PWM) PMEN    PMD
 * ────  ────────── ───────── ──────  ──────
 *  1    V_HI       U_LO      0x3E    0x04
 *  3    W_HI       U_LO      0x3E    0x20
 *  2    W_HI       V_LO      0x37    0x20
 *  6    U_HI       V_LO      0x37    0x02
 *  4    U_HI       W_LO      0x2F    0x02
 *  5    V_HI       W_LO      0x2F    0x04
 */
static const uint8_t __code rev_pmen[8] = {
    0x3F, /* 0: invalid */
    0x3E, /* 1: U_LO (CH0) follows PWM */
    0x37, /* 2: V_LO (CH3) follows PWM */
    0x3E, /* 3: U_LO (CH0) follows PWM */
    0x2F, /* 4: W_LO (CH4) follows PWM */
    0x2F, /* 5: W_LO (CH4) follows PWM */
    0x37, /* 6: V_LO (CH3) follows PWM */
    0x3F  /* 7: invalid */
};

static const uint8_t __code rev_pmd[8] = {
    0x00, /* 0: invalid */
    0x04, /* 1: V_HI on (CH2) */
    0x20, /* 2: W_HI on (CH5) */
    0x20, /* 3: W_HI on (CH5) */
    0x02, /* 4: U_HI on (CH1) */
    0x04, /* 5: V_HI on (CH2) */
    0x02, /* 6: U_HI on (CH1) */
    0x00  /* 7: invalid */
};

static uint8_t rotate_hall_state(uint8_t hall_state)
{
    static const uint8_t __code seq[6] = { 1, 3, 2, 6, 4, 5 };
    uint8_t i;

    if (COMMUTATION_OFFSET == 0 || hall_state == 0 || hall_state == 7)
        return hall_state;

    for (i = 0; i < 6; i++) {
        if (seq[i] == hall_state)
            return seq[(i + COMMUTATION_OFFSET) % 6];
    }

    return hall_state;
}

void commutation_init(void)
{
}

void commutation_get_masks(uint8_t hall_state, int8_t direction,
                           uint8_t *pmen, uint8_t *pmd)
{
    hall_state = rotate_hall_state(hall_state);

    if (hall_state == 0 || hall_state == 7) {
        /* Invalid hall state — all off for safety */
        *pmen = 0x3F;
        *pmd  = 0x00;
        return;
    }

    if (direction >= 0) {
        *pmen = fwd_pmen[hall_state];
        *pmd  = fwd_pmd[hall_state];
    } else {
        *pmen = rev_pmen[hall_state];
        *pmd  = rev_pmd[hall_state];
    }
}

void commutation_update(int8_t direction)
{
    uint8_t hall = hall_read();
    uint8_t pmen, pmd;

    commutation_get_masks(hall, direction, &pmen, &pmd);
    pwm_set_commutation(pmen, pmd);
}

void commutation_coast(void)
{
    /* Float all phases — all FETs off, motor coasts freely */
    pwm_set_commutation(0x3F, 0x00);
}
