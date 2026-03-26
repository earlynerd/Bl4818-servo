/*
 * ADC Driver — Current sense and battery voltage measurement
 *
 * Uses the MS51FB9AE 12-bit SAR ADC. VDD = 5.0V (confirmed).
 *
 * Confirmed channel assignments:
 *   ADC_CH3 (P0.6, pin 2) = Current shunt — direct, NO amplifier
 *   ADC_CH2 (P0.7, pin 3) = Battery voltage — 10k/10k divider (2:1)
 *
 * Register layout (from BSP):
 *   ADCCON0 (E8H): ADCF|ADCS|ETGSEL1:0|ADCHS3:0
 *   ADCCON1 (E1H): -|STADCPX|ADCDIV1:0|ETGTYP1:0|ADCEX|ADCEN
 *   ADCCON2 (E2H): ADFBEN|ADCMPOP|ADCMPEN|ADCMPO|ADCAQT2:0|ADCDLY.8
 *   ADCRH   (C3H): ADC result bits [11:4]
 *   ADCRL   (C2H): ADC result bits [3:0] in upper nibble [7:4]
 */

#include "ms51_reg.h"
#include "ms51_config.h"
#include "adc.h"

void adc_init(void)
{
    /* Configure P0.6 (ADC_CH3) and P0.7 (ADC_CH2) as inputs */
    P0M1 |=  0xC0;  /* P0.6, P0.7 input mode (M1=1) */
    P0M2 &= ~0xC0;  /* (M2=0) */

    /*
     * Disable digital input buffer on ADC pins to reduce noise.
     * AINDIDS: bit=0 enables analog, bit=1 keeps digital.
     * Clear bits for channels we use as analog inputs.
     */
    AINDIDS &= ~0x0C;   /* Clear bits 3,2 for ADC_CH3(P0.6), ADC_CH2(P0.7) */

    /*
     * ADCCON1: ADC configuration
     *   Bit 5:4 = ADCDIV = 01 (FSYS/2 = 12 MHz ADC clock)
     *   Bit 1   = ADCEX = 0 (software trigger only)
     *   Bit 0   = ADCEN = 1 (enable ADC circuit)
     */
    ADCCON1 = 0x11;     /* ADCDIV=01, ADCEN=1 */

    /*
     * ADCCON2: sampling time
     *   Bit 3:1 = ADCAQT = 001 (10 ADC clock acquisition time)
     */
    ADCCON2 = 0x02;
}

uint16_t adc_read(uint8_t channel)
{
    uint16_t result;

    /* Select channel (ADCCON0 bits 3:0 = channel) */
    ADCCON0 = (channel & 0x0F);

    /* Start conversion: set ADCS (bit 6) */
    ADCS = 1;

    /* Wait for conversion complete: ADCF (bit 7) set by hardware */
    while (!ADCF)
        ;

    /* Clear completion flag */
    ADCF = 0;

    /* Read 12-bit result: ADCRH[7:0]=bits[11:4], ADCRL[7:4]=bits[3:0] */
    result = ((uint16_t)ADCRH << 4) | ((ADCRL >> 4) & 0x0F);

    return result;
}

uint16_t adc_read_current_ma(void)
{
    uint16_t raw = adc_read(ADC_CH_CURRENT);

    /*
     * Direct shunt (20mΩ, R020), no amplifier. VDD = 5.0V reference.
     *
     * I_mA = raw × ADC_TO_MA_NUM / ADC_TO_MA_DEN
     *      = raw × 2500 / 41
     *
     * At 20mΩ: ~61mA per LSB, 5A = ADC ~82
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
