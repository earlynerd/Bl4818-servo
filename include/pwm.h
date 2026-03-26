/*
 * PWM Driver — Hardware Complementary PWM with Dead-Time Insertion
 *
 * Uses MS51FB9AE PWM0 module in complementary mode for 3-phase bridge.
 * Hardware dead-time prevents shoot-through even if firmware has bugs.
 */
#ifndef PWM_H
#define PWM_H

#include <stdint.h>

/* Initialize PWM hardware: complementary mode, dead-time, all FETs off */
void pwm_init(void);

/* Set duty cycle (0 to PWM_MAX_DUTY) — applied to all three phase pairs.
 * The commutation mask (PMEN/PMD) determines which phase actually switches. */
void pwm_set_duty(uint16_t duty);

/* Get current duty cycle */
uint16_t pwm_get_duty(void);

/*
 * Set commutation mask for six-step BLDC control.
 *
 * pmen: PWM Mask Enable — bit=1 forces channel to pmd level,
 *       bit=0 lets channel follow PWM generator.
 * pmd:  PWM Mask Data — forced output when masked (0=off, 1=on).
 *
 * Channel bit mapping (bits 5:0):
 *   Bit 0: CH0 = Phase U low-side  (P1.2)
 *   Bit 1: CH1 = Phase U high-side (P1.1)
 *   Bit 2: CH2 = Phase V high-side (P1.0)
 *   Bit 3: CH3 = Phase V low-side  (P0.0)
 *   Bit 4: CH4 = Phase W low-side  (P0.1)
 *   Bit 5: CH5 = Phase W high-side (P0.3)
 */
void pwm_set_commutation(uint8_t pmen, uint8_t pmd);

/* Enable PWM output (start generator) */
void pwm_enable(void);

/* Disable PWM output (stop generator, all FETs off) */
void pwm_disable(void);

/* Emergency fault brake — immediate all-off */
void pwm_fault_brake(void);

#endif /* PWM_H */
