/*
 * Six-Step Commutation — Low-Side-Only PWM Chopping
 *
 * Maps hall state + direction to PWM mask registers (PMEN/PMD).
 * Commutation offset is set at compile time via COMMUTATION_OFFSET.
 */
#ifndef COMMUTATION_H
#define COMMUTATION_H

#include <stdint.h>

void commutation_get_masks(uint8_t hall_state, int8_t direction,
                           uint8_t *pmen, uint8_t *pmd);

/* Update commutation: read halls, compute masks, apply to PWM */
void commutation_update(int8_t direction);

/* Float all phases (coast) — all FETs off */
void commutation_coast(void);

#endif /* COMMUTATION_H */
