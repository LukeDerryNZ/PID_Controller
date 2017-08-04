//**********************************************************
//
// pwmGen.c - Generates a single PWM output
//            on J4-05 (M0PWM7) with fixed frequency and the
//            duty cycle controlled by UP and DOWN buttons in
//            the range 5% to 95% by 5% increments.
// 2017:  Modified for Tiva and using straightforward, polled
//        button debouncing implemented in 'buttons2' module.
// Created on: 28.3.2017
// Authors: Group 51 : fbf10, ljd77
// Last modified:  28.3.2017
//**********************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/pin_map.h" //Needed for pin configure
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "stdio.h"
#include "buttons2.h"

//**********************************************************
//*     Generates a single PWM signal on Tiva board pin J4-05 =
//*     PC5 (M0PWM7).  This is the same PWM output as the  
//*     helicopter main rotor.
//***********************************************************

//***********************************************************
//*     Constants
//***********************************************************
#define MAX_24BIT_VAL 0X0FFFFFF

//*******************************************
//*     PWM config details.
//*******************************************
#define PWM_DIVIDER_CODE  SYSCTL_PWMDIV_4
#define PWM_DIVIDER  4

//*******************************************
//*     PWM Hardware Details M0PWM7 (gen 3)
//*******************************************
//      - Main Rotor PWM: M0PWM7, PC5, J4-05 -
#define PWM_MAIN_BASE	     PWM0_BASE
#define PWM_MAIN_GEN         PWM_GEN_3
#define PWM_MAIN_OUTNUM      PWM_OUT_7
#define PWM_MAIN_OUTBIT      PWM_OUT_7_BIT
#define PWM_MAIN_PERIPH_PWM	 SYSCTL_PERIPH_PWM0
#define PWM_MAIN_PERIPH_GPIO SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_GPIO_BASE   GPIO_PORTC_BASE
#define PWM_MAIN_GPIO_CONFIG GPIO_PC5_M0PWM7
#define PWM_MAIN_GPIO_PIN    GPIO_PIN_5

//      - Tail Rotor PWM: M1PWM5, PF1, J3-10 -
#define PWM_TAIL_BASE	     PWM1_BASE
#define PWM_TAIL_GEN         PWM_GEN_2
#define PWM_TAIL_OUTNUM      PWM_OUT_5
#define PWM_TAIL_OUTBIT      PWM_OUT_5_BIT
#define PWM_TAIL_PERIPH_PWM	 SYSCTL_PERIPH_PWM1
#define PWM_TAIL_PERIPH_GPIO SYSCTL_PERIPH_GPIOF
#define PWM_TAIL_GPIO_BASE   GPIO_PORTF_BASE
#define PWM_TAIL_GPIO_CONFIG GPIO_PF1_M1PWM5
#define PWM_TAIL_GPIO_PIN    GPIO_PIN_1

//*******************************************
//*     Local prototypes
//*******************************************
void SysTickIntHandler (void);
void initClocks (void);
void initSysTick (void);
void initialisePWM (void);
void setMainPWM (uint32_t u32Freq, uint32_t u32Duty);
void setTailPWM (uint32_t u32Freq, uint32_t u32Duty);

//*******************************************
//*     initialisePWM
//*     M0PWM7 (J4-05, PC5) is used for the main rotor motor
//*******************************************
void initialisePWM (void){
	
	// Main Rotor
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_MAIN_PERIPH_GPIO);

    GPIOPinConfigure(PWM_MAIN_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_MAIN_GPIO_BASE, PWM_MAIN_GPIO_PIN);

    PWMGenConfigure(PWM_MAIN_BASE, PWM_MAIN_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
                    
    PWMGenEnable(PWM_MAIN_BASE, PWM_MAIN_GEN);

    PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, false);


    // Tail Rotor
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_PWM);
    SysCtlPeripheralEnable(PWM_TAIL_PERIPH_GPIO);

    GPIOPinConfigure(PWM_TAIL_GPIO_CONFIG);
    GPIOPinTypePWM(PWM_TAIL_GPIO_BASE, PWM_TAIL_GPIO_PIN);

    PWMGenConfigure(PWM_TAIL_BASE, PWM_TAIL_GEN,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    PWMGenEnable(PWM_TAIL_BASE, PWM_TAIL_GEN);

    PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, false);
}

//********************************************************
//*     Function to set the freq, duty cycle of M0PWM7
//********************************************************
void setMainPWM (uint32_t ui32Freq, uint32_t ui32Duty){

    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period = SysCtlClockGet() / PWM_DIVIDER / ui32Freq;

    PWMGenPeriodSet(PWM_MAIN_BASE, PWM_MAIN_GEN, ui32Period);
    PWMPulseWidthSet(PWM_MAIN_BASE, PWM_MAIN_OUTNUM, 
        ui32Period * ui32Duty / 100);
}

void SetMainOn(bool OnOff){
	PWMOutputState(PWM_MAIN_BASE, PWM_MAIN_OUTBIT, OnOff);
}

void SetTailOn(bool OnOff){
	PWMOutputState(PWM_TAIL_BASE, PWM_TAIL_OUTBIT, OnOff);
}

void setTailPWM (uint32_t ui32Freq, uint32_t ui32Duty){

    // Calculate the PWM period corresponding to the freq.
    uint32_t ui32Period = SysCtlClockGet() / PWM_DIVIDER / ui32Freq;

    PWMGenPeriodSet(PWM_TAIL_BASE, PWM_TAIL_GEN, ui32Period);
    PWMPulseWidthSet(PWM_TAIL_BASE, PWM_TAIL_OUTNUM,
        ui32Period * ui32Duty / 100);
}
