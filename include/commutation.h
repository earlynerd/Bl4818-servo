/*
 * Six-Step Commutation — PWM Mask-Based
 *
 * Maps hall state to PWM mask registers (PMEN/PMD) for
 * hardware complementary PWM commutation.
 */
#ifndef COMMUTATION_H
#define COMMUTATION_H

#include <stdint.h>

/* Initialize commutation tables */
void commutation_init(void);

/*
 * Get PMEN/PMD mask values for a given hall state and direction.
 * hall_state: 1-6 (from hall sensors)
 * direction:  >= 0 forward, < 0 reverse
 * pmen/pmd:   output mask register values
 */
void commutation_get_masks(uint8_t hall_state, int8_t direction,
                           uint8_t *pmen, uint8_t *pmd);

/* Update commutation: read halls, compute masks, apply to PWM */
void commutation_update(int8_t direction);

/* Apply active braking pattern (all low-sides on, high-sides off) */
void commutation_brake(void);

/* Float all phases (coast) — all FETs off */
void commutation_coast(void);

#endif /* COMMUTATION_H */
