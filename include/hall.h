/*
 * Hall Sensor Interface — Read hall state and track commutation
 */
#ifndef HALL_H
#define HALL_H

#include <stdint.h>

/* Initialize hall sensor GPIO and interrupt */
void hall_init(void);

/* Read current hall state (3-bit value, 1-6 valid) */
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
