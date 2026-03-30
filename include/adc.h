/*
 * ADC Driver — Current sense and voltage measurement
 */
#ifndef ADC_H
#define ADC_H

#include <stdint.h>

/* Initialize ADC for current/voltage sensing */
void adc_init(void);

/* Read a channel (blocking, returns 12-bit value) */
uint16_t adc_read(uint8_t channel);

/* Read motor current in milliamps */
uint16_t adc_read_current_ma(void);

#endif /* ADC_H */
