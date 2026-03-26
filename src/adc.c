/*
 * ADC Driver — Current sense and battery voltage measurement
 *
 * Uses the MS51FB9AE 12-bit SAR ADC. VDD = 5.0V (confirmed).
 *
 * Confirmed channel assignments:
 *   ADC_CH3 (P0.6, pin 2) = Current shunt — direct, NO amplifier
 *   ADC_CH2 (P0.7, pin 3) = Battery voltage — 10k/10k divider (2:1)
 */

#include "ms51_reg.h"
#include "ms51_config.h"
#include "adc.h"

void adc_init(void)
{
    /* Configure P0.6 (ADC_CH3) and P0.7 (ADC_CH2) as analog inputs */
    P0M1 |=  0xC0;  /* P0.6, P0.7 input mode (M1=1) */
    P0M2 &= ~0xC0;  /* (M2=0) */

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
     *   For 12-bit at 24MHz, use divider for reasonable conversion time.
     */
    ADCCON1 = 0x01;

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
     * Direct shunt, no amplifier. VDD = 5.0V reference.
     *
     * I_mA = raw × ADC_TO_MA_NUM / ADC_TO_MA_DEN
     *      = raw × 625 / 26    (for 50mΩ shunt)
     *
     * At 50mΩ: ~24mA per LSB, 5A = ADC ~205
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
     * 10k/10k voltage divider (2:1 ratio). VDD = 5.0V reference.
     *
     * V_batt_mV = raw / 4096 × 5000 × 2
     *           = raw × 10000 / 4096
     *           ≈ raw × 625 / 256
     *
     * NOTE: With 2:1 divider and 5V ref, max measurable = 10V.
     * At 12V nominal, divider output is 6V → clipped at 5V.
     * Only useful for detecting battery voltages below ~10V.
     */
    uint32_t mv = (uint32_t)raw * 625 / 256;

    return (uint16_t)mv;
}

uint8_t adc_overcurrent(void)
{
    uint16_t ma = adc_read_current_ma();
    return (ma > CURRENT_LIMIT_MA) ? 1 : 0;
}
