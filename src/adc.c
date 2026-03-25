/*
 * ADC Driver — Current sense and battery voltage measurement
 *
 * Uses the MS51FB9AE 12-bit SAR ADC.
 * Channel 6 (P0.6) = current shunt amplifier output
 * Channel 7 (P0.7) = battery voltage divider
 */

#include "ms51_reg.h"
#include "ms51_config.h"
#include "adc.h"

void adc_init(void)
{
    /* Configure P0.6 and P0.7 as analog inputs */
    /* Set P0M1 bits (input mode) and clear P0M2 bits */
    P0M1 |=  0xC0;  /* P0.6, P0.7 as input */
    P0M2 &= ~0xC0;

    /*
     * ADCCON0: ADC control register 0
     *   Bit 7: ADCEN = 1 (enable ADC)
     *   Bit 6: ADCI  = 0 (clear interrupt flag)
     *   Bit 5: ADCS  = 0 (don't start conversion yet)
     *   Bit 3-0: CHS[3:0] = channel select
     */
    ADCCON0 = 0x80;  /* Enable ADC, no conversion yet */

    /*
     * ADCCON1: ADC control register 1
     *   Bit 4-3: ADCDIV = clock divider
     *   For 12-bit at 24MHz, use divider of 4 → ~500KSPS max
     */
    ADCCON1 = 0x01;  /* Conversion time: medium speed */

    /* ADCCON2: trigger source = software */
    ADCCON2 = 0x00;
}

uint16_t adc_read(uint8_t channel)
{
    uint16_t result;

    /* Select channel */
    ADCCON0 = 0x80 | (channel & 0x0F);

    /* Start conversion */
    ADCCON0 |= 0x20;  /* ADCS = 1 */

    /* Wait for conversion complete (ADCI flag) */
    while (!(ADCCON0 & 0x40))
        ;

    /* Clear interrupt flag */
    ADCCON0 &= ~0x40;

    /* Read 12-bit result (ADCRH[7:0] = high 8 bits, ADCRL[3:0] = low 4 bits) */
    result = ((uint16_t)ADCRH << 4) | ((ADCRL >> 4) & 0x0F);

    return result;
}

uint16_t adc_read_current_ma(void)
{
    uint16_t raw = adc_read(ADC_CH_CURRENT);

    /*
     * Convert ADC reading to milliamps:
     * I_mA = raw * ADC_TO_MA_NUM / ADC_TO_MA_DEN
     *      = raw * 825 / 512
     *
     * Use 32-bit intermediate to avoid overflow.
     */
    uint32_t ma = (uint32_t)raw * ADC_TO_MA_NUM / ADC_TO_MA_DEN;

    return (uint16_t)ma;
}

uint16_t adc_read_voltage_mv(void)
{
    uint16_t raw = adc_read(ADC_CH_VOLTAGE);

    /*
     * Battery voltage divider assumed to be ~11:1 ratio
     * (10k + 1k divider for up to ~36V range).
     * V_batt_mV = raw * 3300 / 4096 * 11
     *           = raw * 36300 / 4096
     *           ≈ raw * 554 / 64
     */
    uint32_t mv = (uint32_t)raw * 554 / 64;

    return (uint16_t)mv;
}

uint8_t adc_overcurrent(void)
{
    uint16_t ma = adc_read_current_ma();
    return (ma > CURRENT_LIMIT_MA) ? 1 : 0;
}
