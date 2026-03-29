/*
 * MS51FB9AE Register Definitions for SDCC
 *
 * Based on Nuvoton MS51BSP_SDCC (danchouzhou/MS51BSP_SDCC)
 * Reference: MS51FB9AE Technical Reference Manual Rev1.03
 *
 * *** Addresses verified against official BSP MS51_16K.h ***
 */
#ifndef MS51_REG_H
#define MS51_REG_H

#include <stdint.h>

/* ── Core 8051 Registers ─────────────────────────────────────────────────── */
__sfr __at(0x80) P0;
__sfr __at(0x81) SP;
__sfr __at(0x82) DPL;
__sfr __at(0x83) DPH;
__sfr __at(0x84) RCTRIM0;  /* TA protected — HIRC 16 MHz trim [8:1] */
__sfr __at(0x85) RCTRIM1;  /* TA protected — HIRC 16 MHz trim [0] + HIRC24 (bit 4) */
__sfr __at(0x87) PCON;
__sfr __at(0x88) TCON;
__sfr __at(0x89) TMOD;
__sfr __at(0x8A) TL0;
__sfr __at(0x8B) TL1;
__sfr __at(0x8C) TH0;
__sfr __at(0x8D) TH1;
__sfr __at(0x90) P1;
__sfr __at(0x98) SCON;
__sfr __at(0x99) SBUF;
__sfr __at(0xA0) P2;
__sfr __at(0xA8) IE;
__sfr __at(0xB0) P3;
__sfr __at(0xB8) IP;
__sfr __at(0xD0) PSW;
__sfr __at(0xE0) ACC;
__sfr __at(0xF0) B;

/* ── SFR Page Select and Timed Access ────────────────────────────────────── */
__sfr __at(0x91) SFRS;     /* TA protected — SFR page select */
__sfr __at(0x92) CAPCON0;  /* Capture control/flags */
__sfr __at(0x93) CAPCON1;  /* Capture edge selection */
__sfr __at(0x94) CAPCON2;  /* Capture falling-edge enable */
__sfr __at(0xC7) TA;       /* Timed access protection register */

/* ── Clock Configuration ─────────────────────────────────────────────────── */
__sfr __at(0x8E) CKCON;    /* Bit 6: PWMCKS (PWM clock source) */
__sfr __at(0x95) CKDIV;    /* Clock divider */
__sfr __at(0x96) CKSWT;    /* TA protected — clock switch */

/* ── Port Mode Registers ─────────────────────────────────────────────────── */
__sfr __at(0xB1) P0M1;
__sfr __at(0xB2) P0M2;
__sfr __at(0xB3) P1M1;
__sfr __at(0xB4) P1M2;
/* P2M1/P2M2 not available on MS51FB9AE */
__sfr __at(0xAC) P3M1;
__sfr __at(0xAD) P3M2;

/* ── Timer 2 ─────────────────────────────────────────────────────────────── */
__sfr __at(0xC8) T2CON;
__sfr __at(0xC9) T2MOD;
__sfr __at(0xCA) RCMP2L;
__sfr __at(0xCB) RCMP2H;
__sfr __at(0xCC) TL2;      /* Page 0 (shares CC with PWM4L on page 1) */
__sfr __at(0xCD) TH2;      /* Page 0 (shares CD with PWM5L on page 1) */

/* ── Timer 3 ─────────────────────────────────────────────────────────────── */
__sfr __at(0xC4) T3CON;    /* Page 0 (shares C4 with PWM4H on page 1) */
__sfr __at(0xC5) RL3;      /* Page 0 (shares C5 with PWM5H on page 1) */
__sfr __at(0xC6) RH3;      /* Page 0 (shares C6 with PIOCON1 on page 1) */

/* ── Watchdog Timer ──────────────────────────────────────────────────────── */
__sfr __at(0xAA) WDCON;    /* TA protected */

/* ── Interrupt Registers ─────────────────────────────────────────────────── */
/*     IE  at 0xA8 (declared above) */
/*     IP  at 0xB8 (declared above) */
__sfr __at(0xB7) IPH;      /* Page 0 (shares B7 with PWMINTC on page 1) */
__sfr __at(0xA9) SADDR;
__sfr __at(0xB9) SADEN;
__sfr __at(0x97) CKEN;     /* TA protected — clock enable */
__sfr __at(0x9B) EIE;      /* Extended interrupt enable */
__sfr __at(0x9C) EIE1;     /* Extended interrupt enable 1 */
__sfr __at(0xEF) EIP;      /* Extended interrupt priority */
__sfr __at(0xFE) EIP1;     /* Extended interrupt priority 1 */
__sfr __at(0xF7) EIPH;     /* Extended interrupt priority high */
__sfr __at(0xFF) EIPH1;    /* Extended interrupt priority high 1 */

/* ── UART0 ───────────────────────────────────────────────────────────────── */
/*     SCON at 0x98, SBUF at 0x99 (declared above) */

/* ── UART1 ───────────────────────────────────────────────────────────────── */
__sfr __at(0x9A) SBUF_1;
__sfr __at(0xF8) SCON_1;
__sfr __at(0xBA) SADEN_1;
__sfr __at(0xBB) SADDR_1;

/* ── ADC ─────────────────────────────────────────────────────────────────── */
__sfr __at(0xE8) ADCCON0;  /* Bit-addressable: ADCF|ADCS|ETGSEL1:0|ADCHS3:0 */
__sfr __at(0xE1) ADCCON1;  /* STADCPX|ADCDIV1:0|ETGTYP1:0|ADCEX|ADCEN */
__sfr __at(0xE2) ADCCON2;  /* ADFBEN|ADCMPOP|ADCMPEN|ADCMPO|ADCAQT2:0|ADCDLY.8 */
__sfr __at(0xC2) ADCRL;    /* ADC result low byte (page 0) */
__sfr __at(0xC3) ADCRH;    /* ADC result high byte (page 0) */
__sfr __at(0xE4) C0L;      /* Capture 0 low byte */
__sfr __at(0xE5) C0H;      /* Capture 0 high byte */
__sfr __at(0xE6) C1L;      /* Capture 1 low byte */
__sfr __at(0xE7) C1H;      /* Capture 1 high byte */
__sfr __at(0xE3) ADCDLY;   /* ADC trigger delay */
__sfr __at(0xF6) AINDIDS;  /* Analog input digital input disable */
__sfr __at(0xED) C2L;      /* Capture 2 low byte */
__sfr __at(0xEE) C2H;      /* Capture 2 high byte */

/* ── PWM Module — Page 0 (primary) ───────────────────────────────────────── */
__sfr __at(0xD8) PWMCON0;  /* Bit-addressable: PWMRUN|LOAD|PWMF|CLRPWM|0000 */
__sfr __at(0xDF) PWMCON1;  /* PWMMOD1:0|GP|PWMTYP|FBINEN|PWMDIV2:0 */
__sfr __at(0xD1) PWMPH;    /* Period high byte */
__sfr __at(0xD9) PWMPL;    /* Period low byte */
__sfr __at(0xD2) PWM0H;    /* CH0 duty high byte */
__sfr __at(0xD3) PWM1H;    /* CH1 duty high (ignored in complementary mode) */
__sfr __at(0xD4) PWM2H;    /* CH2 duty high byte */
__sfr __at(0xD5) PWM3H;    /* CH3 duty high (ignored in complementary mode) */
__sfr __at(0xDA) PWM0L;    /* CH0 duty low byte */
__sfr __at(0xDB) PWM1L;    /* CH1 duty low (ignored in complementary mode) */
__sfr __at(0xDC) PWM2L;    /* CH2 duty low byte */
__sfr __at(0xDD) PWM3L;    /* CH3 duty low (ignored in complementary mode) */
__sfr __at(0xDE) PIOCON0;  /* PWM I/O control 0 — primary pin enable */
__sfr __at(0xD6) PNP;      /* PWM negative polarity (per channel) */
__sfr __at(0xD7) FBD;      /* Fault brake data */

/* ── PWM Module — Page 1 (shared addresses with Timer 2/3) ──────────────── */
/* Must switch to SFR page 1 before accessing these registers! */
__sfr __at(0xC4) PWM4H;    /* CH4 duty high (page 1, shares with T3CON) */
__sfr __at(0xC5) PWM5H;    /* CH5 duty high (page 1, shares with RL3) */
__sfr __at(0xCC) PWM4L;    /* CH4 duty low  (page 1, shares with TL2) */
__sfr __at(0xCD) PWM5L;    /* CH5 duty low  (page 1, shares with TH2) */
__sfr __at(0xC6) PIOCON1;  /* PWM I/O control 1 (page 1, shares with RH3) */
__sfr __at(0xB7) PWMINTC;  /* PWM interrupt ctrl (page 1, shares with IPH) */

/* ── PWM Dead-Time and Mask ──────────────────────────────────────────────── */
__sfr __at(0xF9) PDTEN;    /* TA protected — dead-time enable + PDTCNT bit 8 */
__sfr __at(0xFA) PDTCNT;   /* TA protected — dead-time counter [7:0] */
__sfr __at(0xFB) PMEN;     /* PWM mask enable (per channel) */
__sfr __at(0xFC) PMD;      /* PWM mask data (per channel) */

/* ── I2C ─────────────────────────────────────────────────────────────────── */
__sfr __at(0xC0) I2CON;
__sfr __at(0xC1) I2ADDR;
__sfr __at(0xBC) I2DAT;
__sfr __at(0xBD) I2STAT;
__sfr __at(0xBE) I2CLK;
__sfr __at(0xBF) I2TOC;

/* ── SPI ─────────────────────────────────────────────────────────────────── */
__sfr __at(0xF3) SPCR;
__sfr __at(0xF4) SPSR;
__sfr __at(0xF5) SPDR;
__sfr __at(0xF1) CAPCON3;  /* Capture input selection for CAP0/CAP1 */
__sfr __at(0xF2) CAPCON4;  /* Capture input selection for CAP2 */

/* ── Brown-out Detect ────────────────────────────────────────────────────── */
__sfr __at(0xA3) BODCON0;  /* TA protected */
__sfr __at(0xAB) BODCON1;  /* TA protected */

/* ── Flash / IAP ─────────────────────────────────────────────────────────── */
__sfr __at(0xA4) IAPTRG;   /* TA protected — IAP trigger */
__sfr __at(0xA5) IAPUEN;   /* TA protected — IAP update enable */
__sfr __at(0xA6) IAPAL;    /* IAP address low */
__sfr __at(0xA7) IAPAH;    /* IAP address high */
__sfr __at(0x9F) CHPCON;   /* TA protected — chip control (bit 0 = IAPEN) */
__sfr __at(0xAE) IAPFD;    /* IAP flash data */
__sfr __at(0xAF) IAPCN;    /* IAP command */

/* ── Pin Interrupt ───────────────────────────────────────────────────────── */
__sfr __at(0xE9) PICON;    /* Pin interrupt control */
__sfr __at(0xEA) PINEN;    /* Pin interrupt negative enable */
__sfr __at(0xEB) PIPEN;    /* Pin interrupt positive enable */
__sfr __at(0xEC) PIF;      /* Pin interrupt flag */

/* ── Bit-Addressable SFR Bits ────────────────────────────────────────────── */

/* P0 bits (base 0x80) */
__sbit __at(0x80) P00;
__sbit __at(0x81) P01;
__sbit __at(0x82) P02;
__sbit __at(0x83) P03;
__sbit __at(0x84) P04;
__sbit __at(0x85) P05;
__sbit __at(0x86) P06;
__sbit __at(0x87) P07;

/* P1 bits (base 0x90) */
__sbit __at(0x90) P10;
__sbit __at(0x91) P11;
__sbit __at(0x92) P12;
__sbit __at(0x93) P13;
__sbit __at(0x94) P14;
__sbit __at(0x95) P15;
__sbit __at(0x96) P16;
__sbit __at(0x97) P17;

/* P3 bits (base 0xB0) */
__sbit __at(0xB0) P30;

/* TCON bits (base 0x88) */
__sbit __at(0x88) IT0;
__sbit __at(0x89) IE0_bit;
__sbit __at(0x8A) IT1;
__sbit __at(0x8B) IE1_bit;
__sbit __at(0x8C) TR0;
__sbit __at(0x8D) TF0;
__sbit __at(0x8E) TR1;
__sbit __at(0x8F) TF1;

/* IE bits (base 0xA8) */
__sbit __at(0xA8) EX0;
__sbit __at(0xA9) ET0;
__sbit __at(0xAA) EX1;
__sbit __at(0xAB) ET1;
__sbit __at(0xAC) ES;
__sbit __at(0xAD) EBOD;
__sbit __at(0xAF) EA;

/* IP bits (base 0xB8) */
__sbit __at(0xB8) PX0;
__sbit __at(0xB9) PT0;

/* SCON bits (base 0x98) */
__sbit __at(0x98) RI;
__sbit __at(0x99) TI;
__sbit __at(0x9C) REN;
__sbit __at(0x9E) SM1;
__sbit __at(0x9F) SM0;

/* SCON_1 bits (base 0xF8) */
__sbit __at(0xF8) RI_1;
__sbit __at(0xF9) TI_1;
__sbit __at(0xFC) REN_1;
__sbit __at(0xFE) SM1_1;
__sbit __at(0xFF) SM0_1;

/* T2CON bits (base 0xC8) */
__sbit __at(0xC8) CM_RL2;
__sbit __at(0xCA) TR2;
__sbit __at(0xCF) TF2;

/* PSW bits (base 0xD0) */
__sbit __at(0xD0) P_bit;
__sbit __at(0xD2) OV;
__sbit __at(0xD3) RS0;
__sbit __at(0xD4) RS1;
__sbit __at(0xD5) F0;
__sbit __at(0xD6) AC;
__sbit __at(0xD7) CY;

/* PWMCON0 bits (base 0xD8) */
__sbit __at(0xDC) CLRPWM;
__sbit __at(0xDD) PWMF;
__sbit __at(0xDE) LOAD;
__sbit __at(0xDF) PWMRUN;

/* ADCCON0 bits (base 0xE8) */
__sbit __at(0xE8) ADCHS0;
__sbit __at(0xE9) ADCHS1;
__sbit __at(0xEA) ADCHS2;
__sbit __at(0xEB) ADCHS3;
__sbit __at(0xEC) ETGSEL0;
__sbit __at(0xED) ETGSEL1;
__sbit __at(0xEE) ADCS;
__sbit __at(0xEF) ADCF;

/* ── Interrupt Vector Numbers ────────────────────────────────────────────── */
#define INT_EXT0     0   /* External interrupt 0 */
#define INT_TIMER0   1   /* Timer 0 overflow */
#define INT_EXT1     2   /* External interrupt 1 */
#define INT_TIMER1   3   /* Timer 1 overflow */
#define INT_UART0    4   /* UART0 */
#define INT_TIMER2   5   /* Timer 2 */
#define INT_I2C      6   /* I2C */
#define INT_PIN      7   /* Pin interrupt */
#define INT_BOD      8   /* Brown-out detect */
#define INT_SPI      9   /* SPI */
#define INT_WDT      10  /* Watchdog timer */
#define INT_ADC      11  /* ADC conversion complete */
#define INT_CAPTURE  12  /* Input capture */
#define INT_PWM      13  /* PWM */
#define INT_FAULT    14  /* PWM fault brake */
#define INT_UART1    15  /* UART1 */
#define INT_TIMER3   16  /* Timer 3 */
#define INT_WKT      17  /* Wake-up timer */

/* ── IAP Commands ────────────────────────────────────────────────────────── */
#define IAP_BYTE_READ     0x00
#define IAP_READ_UID      0x04
#define IAP_BYTE_PROGRAM  0x21
#define IAP_PAGE_ERASE    0x22
#define IAP_AP_SZ_SELECT  0xE1

/* ── Timed Access Macro ──────────────────────────────────────────────────── */
#define TIMED_ACCESS()  do { TA = 0xAA; TA = 0x55; } while(0)

/* ── SFR Page Switch Macros ──────────────────────────────────────────────── */
#define SFR_PAGE0()     do { TIMED_ACCESS(); SFRS = 0x00; } while(0)
#define SFR_PAGE1()     do { TIMED_ACCESS(); SFRS = 0x01; } while(0)

/* ── Clock Configuration ─────────────────────────────────────────────────── */
#define SET_HIRC_24MHZ() do { \
    TIMED_ACCESS(); \
    RCTRIM1 |= 0x10;   /* Select HIRC24 using the current trim setting */ \
    TIMED_ACCESS(); \
    CKSWT = 0x00;  /* HIRC as system clock */ \
    CKDIV = 0x00;  /* No divider */ \
} while(0)

#endif /* MS51_REG_H */
