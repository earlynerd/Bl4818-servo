/*
 * Six-Step Commutation
 *
 * Maps hall sensor state + direction to bridge drive pattern.
 *
 * In six-step (trapezoidal) commutation:
 * - One phase is driven high (PWM-modulated)
 * - One phase is driven low (current return path)
 * - One phase is floating (back-EMF sensing, if used)
 *
 * Pattern encoding (6 bits):
 *   Bit 5: Phase C high   Bit 4: Phase C low
 *   Bit 3: Phase B high   Bit 2: Phase B low
 *   Bit 1: Phase A high   Bit 0: Phase A low
 */

#include "ms51_reg.h"
#include "ms51_config.h"
#include "commutation.h"
#include "hall.h"
#include "pwm.h"

/*
 * Forward commutation table (indexed by hall state 1-6).
 * Each entry: which high-side to PWM, which low-side to keep on.
 *
 * Hall  Sector  High(PWM)  Low(on)   Pattern
 * ────  ──────  ─────────  ────────  ───────
 *  1     0°     A_HI       B_LO     0b00_01_10 = 0x06
 *  3     60°    A_HI       C_LO     0b01_00_10 = 0x12
 *  2     120°   B_HI       C_LO     0b01_10_00 = 0x18
 *  6     180°   B_HI       A_LO     0b00_10_01 = 0x09
 *  4     240°   C_HI       A_LO     0b10_00_01 = 0x21
 *  5     300°   C_HI       B_LO     0b10_01_00 = 0x24
 */
static const uint8_t __code fwd_table[8] = {
    0x00,   /* 0: invalid — all off */
    0x06,   /* 1: A_HI + B_LO */
    0x18,   /* 2: B_HI + C_LO */
    0x12,   /* 3: A_HI + C_LO */
    0x21,   /* 4: C_HI + A_LO */
    0x24,   /* 5: C_HI + B_LO */
    0x09,   /* 6: B_HI + A_LO */
    0x00    /* 7: invalid — all off */
};

/*
 * Reverse commutation table — swaps high and low roles to reverse torque.
 * This is equivalent to advancing the commutation by 180° electrical.
 *
 * Hall  High(PWM)  Low(on)   Pattern
 * ────  ─────────  ────────  ───────
 *  1     B_HI       A_LO     0b00_10_01 = 0x09
 *  3     C_HI       A_LO     0b10_00_01 = 0x21
 *  2     C_HI       B_LO     0b10_01_00 = 0x24
 *  6     A_HI       B_LO     0b00_01_10 = 0x06
 *  4     A_HI       C_LO     0b01_00_10 = 0x12
 *  5     B_HI       C_LO     0b01_10_00 = 0x18
 */
static const uint8_t __code rev_table[8] = {
    0x00,   /* 0: invalid */
    0x09,   /* 1 */
    0x24,   /* 2 */
    0x21,   /* 3 */
    0x12,   /* 4 */
    0x18,   /* 5 */
    0x06,   /* 6 */
    0x00    /* 7: invalid */
};

void commutation_init(void)
{
    /* Nothing to initialize — tables are in code memory */
}

uint8_t commutation_pattern(uint8_t hall_state, int8_t direction)
{
    if (hall_state == 0 || hall_state == 7)
        return 0x00;  /* Invalid hall state — all off */

    if (direction >= 0)
        return fwd_table[hall_state];
    else
        return rev_table[hall_state];
}

void commutation_update(int8_t direction)
{
    uint8_t hall = hall_read();
    uint8_t pattern = commutation_pattern(hall, direction);
    pwm_apply_pattern(pattern);
}

void commutation_brake(void)
{
    /* Active braking: all low-sides ON, all high-sides OFF.
     * This shorts the motor windings through the low-side FETs,
     * converting kinetic energy to heat in the winding resistance. */
    pwm_set_duty(0);
    pwm_apply_pattern(0x15);  /* All three low-sides: 01_01_01 = 0x15 */
}

void commutation_coast(void)
{
    /* Float all phases — motor coasts freely */
    pwm_set_duty(0);
    pwm_apply_pattern(0x00);
}
