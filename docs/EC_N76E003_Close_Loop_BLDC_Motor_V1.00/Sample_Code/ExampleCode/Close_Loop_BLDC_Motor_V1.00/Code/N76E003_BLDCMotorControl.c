/**************************************************************************//**
* @file     main.c
* @version  V1.00
* @brief    Close Loop BLDC Motor
*
* @note
* Copyright (C) 2019 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

/*---------------------------------------------------------------------------------------------------------*/
/* Pin Functions                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
//P07 - HALL Sensor W Phase, HALL_W
//P06 - HALL Sensor V Phase, HALL_V
//P05 - HALL Sensor U Phase, HALL_U
//P12 - Upper-arm MOS control of U phase, uh
//P14 - Lower-arm MOS control of U phase, ul
//P10 - Upper-arm MOS control of V phase, vh
//P00 - Lower-arm MOS control of V phase, vl
//P01 - Upper-arm MOS control of W phase, wh
//P03 - Lower-arm MOS control of W phase, wl
//P15 - Motor on/off switch
//P02 - UART RX
//P16 - UART TX
//P17 - ADC input

/*---------------------------------------------------------------------------------------------------------*/
/* Include File                                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#include "N76E003.h"
#include "Common.h"
#include "Delay.h"
#include "SFR_Macro.h"
#include "Function_define.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro                                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/

#define UART1_DEBUG 0

#define ABS(X) (X) >= 0 ? (X) : -(X)

#define TIMER0_VALUE 65536-13333  //10ms
#define MotorBoostDuty 0.15

#define HallSensorPhase1 (0x05<<5)
#define HallSensorPhase2 (0x01<<5)
#define HallSensorPhase3 (0x03<<5)
#define HallSensorPhase4 (0x02<<5)
#define HallSensorPhase5 (0x06<<5)
#define HallSensorPhase6 (0x04<<5)

#define ADC_CONVERT_FINISH 1
#define MOTOR_ON_OFF_SWITCH P15
#define MOTOR_OFF 0

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
unsigned char data g_u8HallSensorMotorPhaseState = 0;
unsigned char data g_u8TimerIntCount;
unsigned char data g_u8TH0_Tmp, g_u8TL0_Tmp;
unsigned short data g_u16CurrentSpeed = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Functions                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/

void CheckMotorPhaseByHallSensor(unsigned char HallSensorInput)
{
    switch (HallSensorInput)
    {
        case HallSensorPhase1:
        {
            g_u8HallSensorMotorPhaseState = 1;
            break;
        }
        case HallSensorPhase2:
        {
            g_u8HallSensorMotorPhaseState = 2;
            break;
        }
        case HallSensorPhase3:
        {
            g_u8HallSensorMotorPhaseState = 3;
            break;
        }
        case HallSensorPhase4:
        {
            g_u8HallSensorMotorPhaseState = 4;
            break;
        }
        case HallSensorPhase5:
        {
            g_u8HallSensorMotorPhaseState = 5;
            break;
        }
        case HallSensorPhase6:
        {
            g_u8HallSensorMotorPhaseState = 6;
            break;
        }
    }
}

void ChangeMotorPhaseClockwise(void)
{
    /* Change motor phase to next phase. */
    switch (g_u8HallSensorMotorPhaseState)
    {
        case 1:
        {
            PMEN = 0xfe; //uh
            PMD = 0x20; //wl
            break;
        }
        case 2:
        {
            PMEN = 0xfb; //vh
            PMD = 0x20; //wl
            break;
        }
        case 3:
        {
            PMEN = 0xfb; //vh
            PMD = 0x02; //ul
            break;
        }
        case 4:
        {
            PMEN = 0xef; //wh
            PMD = 0x02; //ul
            break;
        }
        case 5:
        {
            PMEN = 0xef; //wh
            PMD = 0x08; //vl
            break;
        }
        case 6:
        {
            PMEN = 0xfe; //uh
            PMD = 0x08; //vl
            break;
        }
    }
}

void ChangeMotorPhaseCounterClockwise(void)
{
    /* Change motor phase to next phase. */
    switch (g_u8HallSensorMotorPhaseState)
    {
        case 6:
        {
            PMEN = 0xfe; //uh
            PMD = 0x20; //wl
            break;
        }
        case 5:
        {
            PMEN = 0xfb; //vh
            PMD = 0x20; //wl
            break;
        }
        case 4:
        {
            PMEN = 0xfb; //vh
            PMD = 0x02; //ul
            break;
        }
        case 3:
        {
            PMEN = 0xef; //wh
            PMD = 0x02; //ul
            break;
        }
        case 2:
        {
            PMEN = 0xef; //wh
            PMD = 0x08; //vl
            break;
        }
        case 1:
        {
            PMEN = 0xfe; //uh
            PMD = 0x08; //vl
            break;
        }
    }
}

void InitPWM(unsigned short *u16PWMDutyValue, unsigned char *u8OldMotorPhaseState)
{
    /* Initialize the pwm mode and clock. */
    PWM_GP_MODE_ENABLE;
    PWM_SYNCHRONIZED_MODE;
    PWM_CLOCK_FSYS;
    PWMPH = 0x01;
    PWMPL = 0xF3;
    /*----------------------------------------------------------------------
        PWM frequency = Fpwm/((PWMPH,PWMPL) + 1), Fpwm = Fsys/PWM_CLOCK_DIV
                      = (16MHz)/(0x1F3 + 1)
                      = 32KHz (0.03125 ms)
    ----------------------------------------------------------------------*/
    /* Set PWM duty value on motor boost. */
    *u16PWMDutyValue = 0x1F3 * MotorBoostDuty;
    
    /* Initialize the pwm value */
    PWM0H = HIBYTE(*u16PWMDutyValue);
    PWM0L = LOBYTE(*u16PWMDutyValue);

    /* Initialize the Motor phase parameter */
    *u8OldMotorPhaseState = 0xFF;
    CheckMotorPhaseByHallSensor((P0 & 0xE0));

    /* Initialize the pwm pin mode and close whole MOS. */
    PMEN = 0xff;
    PMD = 0x00;
}

void InitTimer2forCapture(void)
{
    /* Initial the Timer2 for capture motor speed */
    TIMER2_CAP0_Capture_Mode;
    IC6_P05_CAP0_RisingEdge_Capture;
    TIMER2_DIV_512;
    /* Enable Capture interrupt */
    set_ECAP;
    /* Triger Timer2 */
    set_TR2;
}

void InitGPIO(void)
{
    Set_All_GPIO_Quasi_Mode;
    P05_Input_Mode;
    P06_Input_Mode;
    P07_Input_Mode;
    P15_Input_Mode;
    P04_PushPull_Mode;
    P12_PushPull_Mode;
    P14_PushPull_Mode;
    P10_PushPull_Mode;
    P00_PushPull_Mode;
    P01_PushPull_Mode;
    P03_PushPull_Mode;
    PWM0_P12_OUTPUT_ENABLE; /* P12 - Upper-arm MOS control of U phase, uh */
    PWM1_P14_OUTPUT_ENABLE; /* P14 - Lower-arm MOS control of U phase, ul */
    PWM2_P10_OUTPUT_ENABLE; /* P10 - Upper-arm MOS control of V phase, vh */
    PWM3_P00_OUTPUT_ENABLE; /* P00 - Lower-arm MOS control of V phase, vl */
    PWM4_P01_OUTPUT_ENABLE; /* P01 - Upper-arm MOS control of W phase, wh */
    PWM5_P03_OUTPUT_ENABLE; /* P03 - Lower-arm MOS control of W phase, wl */
    Enable_ADC_AIN0; /* ADC input */

    PICON = 0xFC;   /* PORT 0 interrupt (Pin int control) */
    PINEN = 0XE0;   /* Generates the pin interrupt when falling edge trigger */
    PIPEN = 0XE0;   /* Generates the pin interrupt when rising edge trigger */
    set_EPI;        /* Enable pin interrupt */
    set_EX0;        /* Enable external interrupt */
}

void InitTimer0(void)
{
    clr_T0M;        /* T0M=0, Timer0 Clock = Fsys/12 */
    TMOD |= 0x01;   /* Timer0 is 16-bit mode */
    
    /* Calculate the timer counter value for controlling the interrupt period on 10ms. */
    g_u8TH0_Tmp = HIBYTE(TIMER0_VALUE);
    g_u8TL0_Tmp = LOBYTE(TIMER0_VALUE);

    /* Sets the timer counter value for controlling the interrupt period. The period is setting on 10ms. */
    TH0 = g_u8TH0_Tmp;
    TL0 = g_u8TL0_Tmp;

    set_ET0;        /* enable Timer0 interrupt */
    set_TR0;        /* Timer0 start */
}

unsigned int GetTargetSpeed(void)
{
    unsigned int data u16TargetSpeed;
    /* ADC will sample the variable resistor value on ADCRH. */
    /* Calculate the percentage of Max rotate speed 4500 rpm to target speed. */
    u16TargetSpeed = (((unsigned long int)4500 * (unsigned long int)ADCRH) / 255);

    /* Set the upper bound and lower bound on 450 and 4500 rpm. */
    if (u16TargetSpeed < 450) u16TargetSpeed = 450;
    if (u16TargetSpeed > 4500) u16TargetSpeed = 4500;

    /* Clear ADN interrupt flag and re-trigger ADC to convert. */
    clr_ADCF;
    set_ADCS;
    return u16TargetSpeed;
}

void main(void)
{
    int data s16SpeedDiff = 0;
    unsigned int data u16TargetSpeed = 0;
    unsigned char data u8TimerCntForUART;
    unsigned char data u8OldMotorPhaseState;
    unsigned short data u16PWMDutyValue;
    
    InitGPIO();
    
#ifdef UART1_DEBUG
    /* Initialize UART1 for Debug */
    u8TimerCntForUART = 0;
    InitialUART1_Timer3(115200);
#endif
    
    /* Initial PWM for controlling the 3 phase of motor */
    InitPWM(&u16PWMDutyValue, &u8OldMotorPhaseState);
    /* Initial Timer 0 for interrupt per 10 ms */
    InitTimer0();
    /* Initial Timer 2 for capturing the motor speed */
    InitTimer2forCapture();

    /* Reset timer and check the motor phase */
    CheckMotorPhaseByHallSensor((P0 & 0xE0));

    /* Clear ADC Flag and Reset Timer interrupt cnt */
    clr_ADCF;
    g_u8TimerIntCount = 0;      
   
    /* Enable all interrupts */
    set_EA;
    
    /* Start the ADC and PWM */
    set_ADCS;
    set_LOAD;
    set_PWMRUN;

    while (1)
    {
        /* Get Motor realtime speed by ADC */
        if (ADCF == ADC_CONVERT_FINISH)
        {
            u16TargetSpeed = GetTargetSpeed();
        }

        if (MOTOR_ON_OFF_SWITCH == MOTOR_OFF)
        {
            /* Stop the motor */
            clr_PWMRUN;
            PMEN = 0xff;
            PMD = 0x00;

            /* If the on/off switch is keeping in off state, stay on this while loop. */
            while (MOTOR_ON_OFF_SWITCH == MOTOR_OFF);

            /* Motor on/off switch is switching to on, re-initial the pwm for starting rotate. */
            
            /* Set PWM duty value on motor boost. */
            u16PWMDutyValue = 0x1F3 * MotorBoostDuty;
            
            /* Initialize the pwm value */
            PWM0H = HIBYTE(u16PWMDutyValue);
            PWM0L = LOBYTE(u16PWMDutyValue);
            
            /* Initialize the Motor phase parameter */
            u8OldMotorPhaseState = 0xFF;
            CheckMotorPhaseByHallSensor((P0 & 0xE0));
            
            /* Start the PWM */
            set_LOAD;
            set_PWMRUN;
            
            /* Clear the Timer interrupt cnt */
            g_u8TimerIntCount = 0;
        }
        else if (g_u8TimerIntCount >= 1) /* if the time past x * 10ms( x = 1 ), entering this if. */
        {
            /* Reset the timer interrupt times counter */
            g_u8TimerIntCount = 0;
            
            /* Calculate the differentiation between target and current speed */
            /*---------------------------------------------------------------------*/
            //    PWM frequency =  32KHz (0.03125 ms)
            //    Timer Capture (PWM clock number) * PWM clock Period = 1/2 Round
            // => Timer Capture (PWM clock number) * PWM clock Period : 1/2 Round = 60s (1min) : X Round
            // => X(s16SpeedDiff) = ((1/2) * 60) / ( 1/32k * Timer Capture (PWM clock number) )
            /*---------------------------------------------------------------------*/
                        
            s16SpeedDiff = (unsigned long int)u16TargetSpeed - ((unsigned long int)30000000 / ((unsigned long int)(g_u16CurrentSpeed) * 32));

#ifdef UART1_DEBUG
            /* Print the speed of motor */
            u8TimerCntForUART++;
            if (u8TimerCntForUART >= 30)   //Per 300ms send UART data
            {
                Send_Data_To_UART1(((u16TargetSpeed - s16SpeedDiff)));
                u8TimerCntForUART = 0;
            }
#endif
            /* Modified the PWM duty for tracing the target speed */
            u16PWMDutyValue += ((s16SpeedDiff > 0) ? 1 : (-1));

            /* Set new PWM duty */
            PWM0H = HIBYTE(u16PWMDutyValue);
            PWM0L = LOBYTE(u16PWMDutyValue);
            set_LOAD;
        }

        /* Change the Motor phase */
        if (u8OldMotorPhaseState != g_u8HallSensorMotorPhaseState)
        {
            /* Record the last motor phase */
            u8OldMotorPhaseState = g_u8HallSensorMotorPhaseState;
            /* Change the motor phase */
            ChangeMotorPhaseClockwise();
        }
    }
}
/*---------------------------------------------------------------------------------------------------------*/
/* Interrupt                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/

void PinInterrupt_ISR(void) interrupt 7
{
    /* Detecting the motor phase by hall sensor. */ 
    CheckMotorPhaseByHallSensor(P0 & 0xe0);
    /* Clear the Pin interrupt flag. */
    PIF = 0x00;
}

void Timer0_ISR(void) interrupt 1
{
    /* Sets the timer counter value for controlling the interrupt period. The period is setting on 10ms. */
    TH0 = g_u8TH0_Tmp;
    TL0 = g_u8TL0_Tmp;
    /* Recorded the interrupt times. */
    g_u8TimerIntCount++;
}

void Capture_ISR(void) interrupt 12
{
    /* Clear capture0 interrupt flag */
    clr_CAPF0;
    /* Get the current motor speed */
    g_u16CurrentSpeed = (C0L + (C0H << 8));

    clr_TF2;
}
/*** (C) COPYRIGHT 2019 Nuvoton Technology Corp. ***/
