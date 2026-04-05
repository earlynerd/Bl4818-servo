;/******************************************************************************
; * @file     startup_M2003.s
; * @version  V1.00
; * @brief    CMSIS Cortex-M Core Device Startup File for M2003
; *
; * SPDX-License-Identifier: Apache-2.0
; * @copyright (C) 2017-2020 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/
;/*
;//-------- <<< Use Configuration Wizard in Context Menu >>> ------------------
;*/


; <h> Stack Configuration
;   <o> Stack Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

	IF :LNOT: :DEF: Stack_Size
Stack_Size      EQU     0x00000400
	ENDIF

                AREA    STACK, NOINIT, READWRITE, ALIGN=3
Stack_Mem       SPACE   Stack_Size
__initial_sp


; <h> Heap Configuration
;   <o>  Heap Size (in Bytes) <0x0-0xFFFFFFFF:8>
; </h>

	IF :LNOT: :DEF: Heap_Size
Heap_Size       EQU     0x00000100
	ENDIF

                AREA    HEAP, NOINIT, READWRITE, ALIGN=3
__heap_base
Heap_Mem        SPACE   Heap_Size
__heap_limit


                PRESERVE8
                THUMB


; Vector Table Mapped to Address 0 at Reset

                AREA    RESET, DATA, READONLY
                EXPORT  __Vectors
                EXPORT  __Vectors_End
                EXPORT  __Vectors_Size

__Vectors       DCD     __initial_sp              ; Top of Stack
                DCD     Reset_Handler             ; Reset Handler
                DCD     NMI_Handler               ; NMI Handler
                DCD     HardFault_Handler         ; Hard Fault Handler
                DCD     MemManage_Handler         ; MPU Fault Handler
                DCD     BusFault_Handler          ; Bus Fault Handler
                DCD     UsageFault_Handler        ; Usage Fault Handler
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     0                         ; Reserved
                DCD     SVC_Handler               ; SVCall Handler
                DCD     DebugMon_Handler          ; Debug Monitor Handler
                DCD     0                         ; Reserved
                DCD     PendSV_Handler            ; PendSV Handler
                DCD     SysTick_Handler           ; SysTick Handler

                ; External Interrupts
                DCD     BOD_IRQHandler            ; 0: Brown Out detection
                DCD     Default_Handler           ; 1:
                DCD     PWRWU_IRQHandler          ; 2: Power down wake up
                DCD     Default_Handler           ; 3:
                DCD     Default_Handler           ; 4:
                DCD     ISP_IRQHandler            ; 5: FMC(ISP)
                DCD     Default_Handler           ; 6:
                DCD     Default_Handler           ; 7:
                DCD     WDT_IRQHandler            ; 8: Watchdog timer
                DCD     WWDT_IRQHandler           ; 9: Window watchdog timer
                DCD     EINT0_IRQHandler          ; 10: External Input 0
                DCD     EINT1_IRQHandler          ; 11: External Input 1
                DCD     EINT2_IRQHandler          ; 12: External Input 2
                DCD     EINT3_IRQHandler          ; 13: External Input 3
                DCD     Default_Handler           ; 14:
                DCD     EINT5_IRQHandler          ; 15: External Input 5
                DCD     Default_Handler           ; 16:
                DCD     GPB_IRQHandler            ; 17: GPIO Port B
                DCD     GPC_IRQHandler            ; 18: GPIO Port C
                DCD     Default_Handler           ; 19:
                DCD     GPE_IRQHandler            ; 20: GPIO Port E
                DCD     GPF_IRQHandler            ; 21: GPIO Port F
                DCD     Default_Handler           ; 22:
                DCD     Default_Handler           ; 23:
                DCD     Default_Handler           ; 24:
                DCD     PWM0_IRQHandler           ; 25: PWM0
                DCD     Default_Handler           ; 26:
                DCD     Default_Handler           ; 27:
                DCD     Default_Handler           ; 28:
                DCD     Default_Handler           ; 29:
                DCD     Default_Handler           ; 30:
                DCD     Default_Handler           ; 31:
                DCD     TMR0_IRQHandler           ; 32: Timer 0
                DCD     TMR1_IRQHandler           ; 33: Timer 1
                DCD     TMR2_IRQHandler           ; 34: Timer 2
                DCD     TMR3_IRQHandler           ; 35: Timer 3
                DCD     UART0_IRQHandler          ; 36: UART0
                DCD     UART1_IRQHandler          ; 37: UART1
                DCD     I2C0_IRQHandler           ; 38: I2C0
                DCD     Default_Handler           ; 39:
                DCD     Default_Handler           ; 40:
                DCD     Default_Handler           ; 41:
                DCD     ADC_IRQHandler            ; 42: ADC interrupt
                DCD     Default_Handler           ; 43:
                DCD     Default_Handler           ; 44:
                DCD     Default_Handler           ; 45:
                DCD     Default_Handler           ; 46:
                DCD     Default_Handler           ; 47:
                DCD     Default_Handler           ; 48:
                DCD     Default_Handler           ; 49:
                DCD     Default_Handler           ; 50:
                DCD     Default_Handler           ; 51:
                DCD     USCI0_IRQHandler          ; 52: USCI0
                DCD     Default_Handler           ; 53:
                DCD     Default_Handler           ; 54:
                DCD     Default_Handler           ; 55:
                DCD     Default_Handler           ; 56:
                DCD     Default_Handler           ; 57:
                DCD     Default_Handler           ; 58:
                DCD     Default_Handler           ; 59:
                DCD     ECAP0_IRQHandler          ; 60: ECAP0
                DCD     Default_Handler           ; 61:
                DCD     Default_Handler           ; 62:
                DCD     Default_Handler           ; 63:

__Vectors_End

__Vectors_Size  EQU     __Vectors_End - __Vectors

                AREA    |.text|, CODE, READONLY


; Reset Handler

Reset_Handler   PROC
                EXPORT  Reset_Handler             [WEAK]
                IMPORT  SystemInit
                IMPORT  __main

                 LDR     R0, =0x40000100
                ; Unlock Register

                LDR     R1, =0x59
                STR     R1, [R0]
                LDR     R1, =0x16
                STR     R1, [R0]
                LDR     R1, =0x88
                STR     R1, [R0]

                ; Init POR
                LDR     R2, =0x40000024
                LDR     R1, =0x5AA5
                STR     R1, [R2]

                LDR     R2, =0x400001EC
                STR     R1, [R2]


                ; Lock register
                MOVS    R1, #0
                STR     R1, [R0]

                LDR     R0, =SystemInit
                BLX     R0
                LDR     R0, =__main
                BX      R0

                ENDP


; Dummy Exception Handlers (infinite loops which can be modified)

NMI_Handler     PROC
                EXPORT  NMI_Handler               [WEAK]
                B       .
                ENDP
HardFault_Handler\
                PROC
                IMPORT  ProcessHardFault
                EXPORT  HardFault_Handler         [WEAK]
                MOV     R0, LR
                MRS     R1, MSP
                MRS     R2, PSP
                LDR     R3, =ProcessHardFault
                BLX     R3
                BX      R0
                ENDP
MemManage_Handler\
                PROC
                EXPORT  MemManage_Handler         [WEAK]
                B       .
                ENDP
BusFault_Handler\
                PROC
                EXPORT  BusFault_Handler          [WEAK]
                B       .
                ENDP
UsageFault_Handler\
                PROC
                EXPORT  UsageFault_Handler        [WEAK]
                B       .
                ENDP
SVC_Handler     PROC
                EXPORT  SVC_Handler               [WEAK]
                B       .
                ENDP
DebugMon_Handler\
                PROC
                EXPORT  DebugMon_Handler          [WEAK]
                B       .
                ENDP
PendSV_Handler\
                PROC
                EXPORT  PendSV_Handler            [WEAK]
                B       .
                ENDP
SysTick_Handler\
                PROC
                EXPORT  SysTick_Handler           [WEAK]
                B       .
                ENDP

Default_Handler PROC

                EXPORT  BOD_IRQHandler            [WEAK]
                EXPORT  PWRWU_IRQHandler          [WEAK]
                EXPORT  ISP_IRQHandler            [WEAK]
                EXPORT  WDT_IRQHandler            [WEAK]
                EXPORT  WWDT_IRQHandler           [WEAK]
                EXPORT  EINT0_IRQHandler          [WEAK]
                EXPORT  EINT1_IRQHandler          [WEAK]
                EXPORT  EINT2_IRQHandler          [WEAK]
                EXPORT  EINT3_IRQHandler          [WEAK]
                EXPORT  EINT5_IRQHandler          [WEAK]
                EXPORT  GPB_IRQHandler            [WEAK]
                EXPORT  GPC_IRQHandler            [WEAK]
                EXPORT  GPE_IRQHandler            [WEAK]
                EXPORT  GPF_IRQHandler            [WEAK]
                EXPORT  PWM0_IRQHandler          [WEAK]
                EXPORT  TMR0_IRQHandler           [WEAK]
                EXPORT  TMR1_IRQHandler           [WEAK]
                EXPORT  TMR2_IRQHandler           [WEAK]
                EXPORT  TMR3_IRQHandler           [WEAK]
                EXPORT  UART0_IRQHandler          [WEAK]
                EXPORT  UART1_IRQHandler          [WEAK]
                EXPORT  I2C0_IRQHandler           [WEAK]
                EXPORT  ADC_IRQHandler           [WEAK]
                EXPORT  USCI0_IRQHandler          [WEAK]
                EXPORT  ECAP0_IRQHandler          [WEAK]


Default__IRQHandler
BOD_IRQHandler
IRC_IRQHandler
PWRWU_IRQHandler
ISP_IRQHandler
WDT_IRQHandler
WWDT_IRQHandler
EINT0_IRQHandler
EINT1_IRQHandler
EINT2_IRQHandler
EINT3_IRQHandler
EINT5_IRQHandler
GPB_IRQHandler
GPC_IRQHandler
GPE_IRQHandler
GPF_IRQHandler
PWM0_IRQHandler
TMR0_IRQHandler
TMR1_IRQHandler
TMR2_IRQHandler
TMR3_IRQHandler
UART0_IRQHandler
UART1_IRQHandler
I2C0_IRQHandler
ADC_IRQHandler
USCI0_IRQHandler
ECAP0_IRQHandler


                B       .
                ENDP


                ALIGN


; User Initial Stack & Heap

                IF      :DEF:__MICROLIB

                EXPORT  __initial_sp
                EXPORT  __heap_base
                EXPORT  __heap_limit

                ELSE

                IMPORT  __use_two_region_memory
                EXPORT  __user_initial_stackheap

__user_initial_stackheap PROC
                LDR     R0, =  Heap_Mem
                LDR     R1, =(Stack_Mem + Stack_Size)
                LDR     R2, = (Heap_Mem +  Heap_Size)
                LDR     R3, = Stack_Mem
                BX      LR
                ENDP

                ALIGN

                ENDIF

;int32_t SH_DoCommand(int32_t n32In_R0, int32_t n32In_R1, int32_t *pn32Out_R0)
SH_DoCommand    PROC

                EXPORT      SH_DoCommand
                IMPORT      SH_Return

                BKPT   0xAB                ; Wait ICE or HardFault
                LDR    R3, =SH_Return
                PUSH   {R3 ,lr}
                BLX    R3                  ; Call SH_Return. The return value is in R0
                POP    {R3 ,PC}            ; Return value = R0

                ENDP

__PC            PROC
                EXPORT      __PC

                MOV     r0, lr
                BLX     lr
                ALIGN

                ENDP

                END
;/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
