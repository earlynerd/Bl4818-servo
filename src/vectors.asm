; Absolute interrupt vector table for the MS51.
;
; SDCC does not emit a UART1 vector entry for this build, so place the reset
; and interrupt stubs explicitly in CABS and keep relocatable code above the
; vector table.

	.module vectors

	.globl __sdcc_gsinit_startup
	.globl _uart1_isr

	.area CABS (ABS,CODE)

	.org 0x0000
	ljmp	__sdcc_gsinit_startup

	.org 0x0003
	reti

	.org 0x000B
	reti

	.org 0x0013
	reti

	.org 0x001B
	reti

	.org 0x0023
	reti

	.org 0x002B
	reti

	.org 0x0033
	reti

	.org 0x003B
	reti

	.org 0x0043
	reti

	.org 0x004B
	reti

	.org 0x0053
	reti

	.org 0x005B
	reti

	.org 0x0063
	reti

	.org 0x006B
	reti

	.org 0x0073
	reti

	.org 0x007B
	ljmp	_uart1_isr

	.org 0x0083
	reti

	.org 0x008B
	reti
