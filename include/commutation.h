/*
 * Six-Step Commutation — Maps hall state to bridge drive pattern
 */
#ifndef COMMUTATION_H
#define COMMUTATION_H

#include <stdint.h>

/*
 * Bridge drive pattern encoding (6 bits):
 *   Bit 5: Phase C high-side
 *   Bit 4: Phase C low-side
 *   Bit 3: Phase B high-side
 *   Bit 2: Phase B low-side
 *   Bit 1: Phase A high-side
 *   Bit 0: Phase A low-side
 *
 * For each commutation step, exactly one high-side and one low-side
 * (on different phases) are PWM-modulated. The third phase is floating.
 */

/* Initialize commutation tables */
void commutation_init(void);

/* Get drive pattern for given hall state and direction
 * hall_state: 1-6 (from hall sensors)
 * direction:  1 = forward, -1 = reverse
 * Returns: 6-bit drive pattern
 */
uint8_t commutation_pattern(uint8_t hall_state, int8_t direction);

/* Update commutation: read halls, compute pattern, apply to PWM */
void commutation_update(int8_t direction);

/* Apply active braking pattern (all low-sides on) */
void commutation_brake(void);

/* Float all phases (coast) */
void commutation_coast(void);

#endif /* COMMUTATION_H */
