/*
 * Six-Step Commutation — PWM Mask-Based
 *
 * Maps hall sensor state + direction to PWM mask registers (PMEN/PMD)
 * for hardware complementary PWM commutation.
 *
 * In six-step (trapezoidal) commutation:
 * - One phase pair is unmasked → complementary PWM switching with dead-time
 * - One phase has its low-side masked ON (current return path)
 * - One phase has both channels masked OFF (floating)
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
 * Forward commutation mask tables (indexed by hall state 1-6).
 *
 * Hall  Sector  HI(PWM)  LO(on)   PMEN    PMD
 * ────  ──────  ───────  ──────   ──────  ──────
 *  1     0°     U_HI     V_LO     0x3C    0x08
 *  3     60°    U_HI     W_LO     0x3C    0x10
 *  2     120°   V_HI     W_LO     0x33    0x10
 *  6     180°   V_HI     U_LO     0x33    0x01
 *  4     240°   W_HI     U_LO     0x0F    0x01
 *  5     300°   W_HI     V_LO     0x0F    0x08
 *
 * Detail for each step:
 *
 * Hall=1: U_HI_PWM + V_LO_ON
 *   U: CH0/CH1 unmasked (complementary PWM) → PMEN bits 1:0 = 00
 *   V: CH2(hi)=masked LOW, CH3(lo)=masked HIGH → PMEN bits 3:2 = 11, PMD 3=1,2=0
 *   W: CH4(lo)=masked LOW, CH5(hi)=masked LOW → PMEN bits 5:4 = 11, PMD 5:4=00
 *   PMEN = 0b111100 = 0x3C, PMD = 0b001000 = 0x08
 *
 * Hall=3: U_HI_PWM + W_LO_ON
 *   U: unmasked
 *   V: all masked LOW
 *   W: CH4(lo)=masked HIGH, CH5(hi)=masked LOW
 *   PMEN = 0x3C, PMD = 0b010000 = 0x10
 *
 * Hall=2: V_HI_PWM + W_LO_ON
 *   U: all masked LOW
 *   V: CH2/CH3 unmasked
 *   W: CH4(lo)=masked HIGH, CH5(hi)=masked LOW
 *   PMEN = 0b110011 = 0x33, PMD = 0b010000 = 0x10
 *
 * Hall=6: V_HI_PWM + U_LO_ON
 *   U: CH0(lo)=masked HIGH, CH1(hi)=masked LOW
 *   V: CH2/CH3 unmasked
 *   W: all masked LOW
 *   PMEN = 0x33, PMD = 0b000001 = 0x01
 *
 * Hall=4: W_HI_PWM + U_LO_ON
 *   U: CH0(lo)=masked HIGH, CH1(hi)=masked LOW
 *   V: all masked LOW
 *   W: CH4/CH5 unmasked
 *   PMEN = 0b001111 = 0x0F, PMD = 0x01
 *
 * Hall=5: W_HI_PWM + V_LO_ON
 *   U: all masked LOW
 *   V: CH2(hi)=masked LOW, CH3(lo)=masked HIGH
 *   W: CH4/CH5 unmasked
 *   PMEN = 0x0F, PMD = 0b001000 = 0x08
 */
static const uint8_t __code fwd_pmen[8] = {
    0x3F, /* 0: invalid — all masked off */
    0x3C, /* 1: U pair active */
    0x33, /* 2: V pair active */
    0x3C, /* 3: U pair active */
    0x0F, /* 4: W pair active */
    0x0F, /* 5: W pair active */
    0x33, /* 6: V pair active */
    0x3F  /* 7: invalid — all masked off */
};

static const uint8_t __code fwd_pmd[8] = {
    0x00, /* 0: invalid — all off */
    0x08, /* 1: V_LO on (CH3) */
    0x10, /* 2: W_LO on (CH4) */
    0x10, /* 3: W_LO on (CH4) */
    0x01, /* 4: U_LO on (CH0) */
    0x08, /* 5: V_LO on (CH3) */
    0x01, /* 6: U_LO on (CH0) */
    0x00  /* 7: invalid — all off */
};

/*
 * Reverse commutation: swap high-side PWM and low-side roles to reverse torque.
 *
 * Hall  HI(PWM)  LO(on)   PMEN    PMD
 * ────  ───────  ──────   ──────  ──────
 *  1    V_HI     U_LO     0x33    0x01
 *  3    W_HI     U_LO     0x0F    0x01
 *  2    W_HI     V_LO     0x0F    0x08
 *  6    U_HI     V_LO     0x3C    0x08
 *  4    U_HI     W_LO     0x3C    0x10
 *  5    V_HI     W_LO     0x33    0x10
 */
static const uint8_t __code rev_pmen[8] = {
    0x3F, /* 0: invalid */
    0x33, /* 1: V pair active */
    0x0F, /* 2: W pair active */
    0x0F, /* 3: W pair active */
    0x3C, /* 4: U pair active */
    0x33, /* 5: V pair active */
    0x3C, /* 6: U pair active */
    0x3F  /* 7: invalid */
};

static const uint8_t __code rev_pmd[8] = {
    0x00, /* 0: invalid */
    0x01, /* 1: U_LO on (CH0) */
    0x08, /* 2: V_LO on (CH3) */
    0x01, /* 3: U_LO on (CH0) */
    0x10, /* 4: W_LO on (CH4) */
    0x10, /* 5: W_LO on (CH4) */
    0x08, /* 6: V_LO on (CH3) */
    0x00  /* 7: invalid */
};

void commutation_init(void)
{
    /* Tables are in code memory — nothing to initialize */
}

void commutation_get_masks(uint8_t hall_state, int8_t direction,
                           uint8_t *pmen, uint8_t *pmd)
{
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

void commutation_brake(void)
{
    /*
     * Active braking: all low-sides ON, all high-sides OFF.
     * This shorts the motor windings through the low-side FETs,
     * converting kinetic energy to heat in winding resistance.
     *
     * All channels masked:
     *   CH0 (U_lo) = HIGH (on)    bit 0 = 1
     *   CH1 (U_hi) = LOW  (off)   bit 1 = 0
     *   CH2 (V_hi) = LOW  (off)   bit 2 = 0
     *   CH3 (V_lo) = HIGH (on)    bit 3 = 1
     *   CH4 (W_lo) = HIGH (on)    bit 4 = 1
     *   CH5 (W_hi) = LOW  (off)   bit 5 = 0
     *
     * PMD = 0b011001 = 0x19
     */
    pwm_set_commutation(0x3F, 0x19);
}

void commutation_coast(void)
{
    /* Float all phases — all FETs off, motor coasts freely */
    pwm_set_commutation(0x3F, 0x00);
}
