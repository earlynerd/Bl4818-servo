/*
 * PWM Driver — Complementary 3-phase PWM with dead-time insertion
 */
#ifndef PWM_H
#define PWM_H

#include <stdint.h>

/* Initialize PWM hardware for 6-channel complementary output */
void pwm_init(void);

/* Set duty cycle (0 to PWM_MAX_DUTY) for the active phase pair */
void pwm_set_duty(uint16_t duty);

/* Get current duty cycle */
uint16_t pwm_get_duty(void);

/* Enable/disable PWM output (all phases) */
void pwm_enable(void);
void pwm_disable(void);

/* Activate fault brake — all high-sides off, all low-sides off */
void pwm_fault_brake(void);

/* Apply commutation pattern: sets which phases are driven */
void pwm_apply_pattern(uint8_t pattern);

#endif /* PWM_H */
