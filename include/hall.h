/*
 * Hall Sensor Interface — Read hall state and track commutation
 */
#ifndef HALL_H
#define HALL_H

#include <stdint.h>

/* Initialize hall sensor GPIO and interrupt */
void hall_init(void);

/* Read raw hall sensor pins (3-bit value, bit2=Hall3, bit1=Hall2, bit0=Hall1) */
uint8_t hall_read_raw(void);

/* Map a raw 3-bit hall pattern onto the logical commutation state */
uint8_t hall_decode_state(uint8_t raw_state);

/* Read logical hall state after applying the decode table (1-6 valid) */
uint8_t hall_read(void);

/* Get electrical sector from hall state (0-5) */
uint8_t hall_sector(void);

/* Get direction of rotation from hall transitions (+1 or -1, 0 if unknown) */
int8_t hall_direction(void);

/* Get time between last two hall transitions (for speed calculation) */
uint16_t hall_period(void);

/* Get cumulative hall transition count (for coarse position) */
int32_t hall_count(void);

/* Reset hall counter */
void hall_count_reset(void);

/* Poll for hall transitions from a fast path; returns 1 when a valid edge was processed */
uint8_t hall_poll(void);

/* Compatibility wrapper for ISR-style call sites */
void hall_isr(void);

#endif /* HALL_H */
