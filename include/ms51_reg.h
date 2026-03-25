/*
 * MS51FB9AE Register Definitions for SDCC
 *
 * Based on Nuvoton MS51 BSP adapted for SDCC compiler.
 * Reference: MS51FB9AE Technical Reference Manual Rev1.03
 */
#ifndef MS51_REG_H
#define MS51_REG_H

#include <8051.h>

/* ── SFR Registers (directly addressed, 0x80-0xFF) ───────────────────────── */

/* Port registers */
__sfr __at(0x80) P0;
__sfr __at(0x90) P1;
__sfr __at(0xA0) P2;
__sfr __at(0xF0) P3;

/* Port mode registers */
__sfr __at(0x93) P0M1;
__sfr __at(0x94) P0M2;
__sfr __at(0x91) P1M1;
__sfr __at(0x92) P1M2;
__sfr __at(0x95) P2M1;
__sfr __at(0x96) P2M2;
__sfr __at(0xB1) P3M1;
__sfr __at(0xB2) P3M2;

/* System clock */
__sfr __at(0x86) CKCON;
__sfr __at(0x97) CKSWT;
__sfr __at(0x98) CKDIV;

/* Power control */
__sfr __at(0x87) PCON;

/* Timer registers */
__sfr __at(0x88) TCON;
__sfr __at(0x89) TMOD;
__sfr __at(0x8A) TL0;
__sfr __at(0x8C) TH0;
__sfr __at(0x8B) TL1;
__sfr __at(0x8D) TH1;

/* Timer 2 */
__sfr __at(0xC8) T2CON;
__sfr __at(0xC9) T2MOD;
__sfr __at(0xCA) RCMP2L;
__sfr __at(0xCB) RCMP2H;
__sfr __at(0xCC) TL2;
__sfr __at(0xCD) TH2;

/* Timer 3 */
__sfr __at(0xC4) T3CON;
__sfr __at(0xC5) RL3;
__sfr __at(0xC6) RH3;

/* Watchdog timer */
__sfr __at(0xAA) WDCON;

/* Interrupt registers */
__sfr __at(0xA8) IE;
__sfr __at(0xA9) SADDR;
__sfr __at(0xB8) IP;
__sfr __at(0xB9) SADEN;
__sfr __at(0xAF) IE1;
__sfr __at(0xBF) IP1;
__sfr __at(0xEE) EIE0;
__sfr __at(0xEF) EIE1;
__sfr __at(0xFE) EIP0;
__sfr __at(0xFF) EIP1;

/* UART */
__sfr __at(0x98) SCON;
__sfr __at(0x99) SBUF;
__sfr __at(0x9A) SCON_1;
__sfr __at(0x9B) SBUF_1;

/* SPI */
__sfr __at(0xD5) SPDR;
__sfr __at(0xD4) SPSR;
__sfr __at(0xD3) SPCR;

/* I2C */
__sfr __at(0xC0) I2CON;
__sfr __at(0xC1) I2ADDR;
__sfr __at(0xC2) I2DAT;
__sfr __at(0xC3) I2STATUS;
__sfr __at(0xC7) I2CLK;

/* ADC */
__sfr __at(0xD6) ADCCON0;
__sfr __at(0xD7) ADCCON1;
__sfr __at(0xBE) ADCCON2;
__sfr __at(0xCE) ADCRL;
__sfr __at(0xCF) ADCRH;

/* PWM (basic SFR access — extended via page register) */
__sfr __at(0xD8) PWMCON0;
__sfr __at(0xD9) PWMCON1;

/* Brown-out detect */
__sfr __at(0x87) BODCON0;
__sfr __at(0xAB) BODCON1;

/* Flash / IAP */
__sfr __at(0xA6) IAPAL;
__sfr __at(0xA7) IAPAH;
__sfr __at(0xA2) IAPFD;
__sfr __at(0xA4) IAPCN;
__sfr __at(0xA5) IAPTRG;
__sfr __at(0xA3) IAPUEN;

/* Misc */
__sfr __at(0x81) SP;
__sfr __at(0x82) DPL;
__sfr __at(0x83) DPH;
__sfr __at(0xD0) PSW;
__sfr __at(0xE0) ACC;
__sfr __at(0xF0) B;

/* ── SFR Page Registers (accessed via TA protection or page select) ──────── */
__sfr __at(0xC7) TA;  /* Timed-access protection register */

/* SFR page select */
__sfr __at(0x91) SFRS;

/* ── Bit-addressable SFR bits ────────────────────────────────────────────── */

/* IE bits */
__sbit __at(0xAF) EA;
__sbit __at(0xAC) ES;
__sbit __at(0xAB) ET1;
__sbit __at(0xAA) EX1;
__sbit __at(0xA9) ET0;
__sbit __at(0xA8) EX0;

/* TCON bits */
__sbit __at(0x8F) TF1;
__sbit __at(0x8E) TR1;
__sbit __at(0x8D) TF0;
__sbit __at(0x8C) TR0;
__sbit __at(0x8B) IE1_bit;
__sbit __at(0x8A) IT1;
__sbit __at(0x89) IE0_bit;
__sbit __at(0x88) IT0;

/* SCON bits */
__sbit __at(0x9F) SM0;
__sbit __at(0x9E) SM1;
__sbit __at(0x9C) REN;
__sbit __at(0x99) TI;
__sbit __at(0x98) RI;

/* PSW bits */
__sbit __at(0xD7) CY;
__sbit __at(0xD6) AC;
__sbit __at(0xD5) F0;
__sbit __at(0xD4) RS1;
__sbit __at(0xD3) RS0;
__sbit __at(0xD2) OV;
__sbit __at(0xD0) P_bit;

/* P0 bits */
__sbit __at(0x87) P07;
__sbit __at(0x86) P06;
__sbit __at(0x85) P05;
__sbit __at(0x84) P04;
__sbit __at(0x83) P03;
__sbit __at(0x82) P02;
__sbit __at(0x81) P01;
__sbit __at(0x80) P00;

/* P1 bits */
__sbit __at(0x97) P17;
__sbit __at(0x96) P16;
__sbit __at(0x95) P15;
__sbit __at(0x94) P14;
__sbit __at(0x93) P13;
__sbit __at(0x92) P12;
__sbit __at(0x91) P11;
__sbit __at(0x90) P10;

/* ── PWM Extended Registers (accessed via SFR pages) ─────────────────────── */
/* These are accessed by setting SFRS page and using timed-access where needed */

/* PWM register addresses (directly mapped for MS51 PWM module) */
#define PWM0_CLK_ADDR   0xD8
#define PWM0_CON0_ADDR  0xD8
#define PWM0_CON1_ADDR  0xD9

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
#define IAP_BYTE_PROGRAM  0x21
#define IAP_PAGE_ERASE    0x22
#define IAP_AP_SZ_SELECT  0xE1

/* ── Timed Access Macro ──────────────────────────────────────────────────── */
/* MS51 requires timed-access sequence to modify protected SFRs */
#define TIMED_ACCESS()  do { TA = 0xAA; TA = 0x55; } while(0)

/* ── SFR Page Switch Macros ──────────────────────────────────────────────── */
#define SFR_PAGE0()     do { TIMED_ACCESS(); SFRS = 0x00; } while(0)
#define SFR_PAGE1()     do { TIMED_ACCESS(); SFRS = 0x01; } while(0)

/* ── Clock Configuration ─────────────────────────────────────────────────── */
#define FSYS_16MHZ  0
#define FSYS_24MHZ  1

/* HIRC trim to 24MHz: set CKSWT, CKDIV */
#define SET_HIRC_24MHZ() do { \
    TIMED_ACCESS(); \
    CKSWT = 0x00;  /* HIRC as system clock */ \
    CKDIV = 0x00;  /* No divider */ \
} while(0)

#endif /* MS51_REG_H */
