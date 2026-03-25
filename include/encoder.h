/*
 * Quadrature Encoder Interface
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* Initialize encoder GPIO and interrupt */
void encoder_init(void);

/* Get current position in counts */
int32_t encoder_get_position(void);

/* Set / reset position */
void encoder_set_position(int32_t counts);

/* Get velocity in counts per control period */
int16_t encoder_get_velocity(void);

/* Call from pin-change ISR for encoder channel A */
void encoder_isr(void);

#endif /* ENCODER_H */
