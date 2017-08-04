/*
 * Main.c
 *
 *  Created on:   May 2, 2017
 *  Authors:      Group 51 : fbf10, ljd77
 *  Last Updated: May 25, 2017
 *
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"

#include "utils/uartstdio.h"

#include "driverlib/pin_map.h" //Needed for pin configure
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"

#include "stdio.h"

#include "buttons2.h"
#include "lib/OrbitOLEDInterface.h"
#include "circBufT.h"
#include "helicopterSignal.h"
#include "pwmGen.h"


//********************************************************
// Constants
//********************************************************
#define SYSTICK_RATE_HZ         100
#define DISPLAYTICK_RATE_HZ     4
#define FREQUENCY               250

#define PWM_MAIN_PERIPH_GPIO    SYSCTL_PERIPH_GPIOC
#define PWM_MAIN_PERIPH_PWM     SYSCTL_PERIPH_PWM0
#define PWM_TAIL_PERIPH_PWM     SYSCTL_PERIPH_PWM1
#define PWM_DIVIDER_CODE        SYSCTL_PWMDIV_4
#define PWM_DIVIDER             4
#define PWM_MIN                 5
#define PWM_MAX                 95

// UART
#define BAUD_RATE               9600           // The Baud Rate of the UART
#define UART_USB_BASE           UART0_BASE
#define UART_USB_PERIPH_UART    SYSCTL_PERIPH_UART0
#define UART_USB_PERIPH_GPIO    SYSCTL_PERIPH_GPIOA
#define UART_USB_GPIO_BASE      GPIO_PORTA_BASE
#define UART_USB_GPIO_PIN_RX    GPIO_PIN_0
#define UART_USB_GPIO_PIN_TX    GPIO_PIN_1
#define UART_USB_GPIO_PINS      UART_USB_GPIO_PIN_RX | UART_USB_GPIO_PIN_TX
#define UART_CLK_FREQ           16000000       // The UART clock freq

// Constant used to scale integers for division/multiplication without truncation/rounding
#define DUTYSCALER              1000
//********************************************************
// Prototypes
//********************************************************
/*
void PinChangeIntHandler (void);
void InitPin (void);
float fCalculateDegrees (void);      <--
uint32_t ui32SampleAltitude (void);  <-- Not strictly required
void SampleStoreAltitude (void);     <--
int32_t i32CalcAltPercent (void);
void Display (float fDegrees, uint32_t ui32Alt);
*/

//********************************************************
// Global Vars
//********************************************************
volatile bool displayTick = 0;          // Signal for display
int32_t targetAlt = 0;
int32_t targetYaw = 0;
bool firstRun = true;                   // Flag for switch PA7
bool landTrigger_descend = false;

int32_t MainDuty = 0;                   // Main Duty:
int32_t d_lastError_main = 0;                //
int32_t i_sum_main = 0;                // Accumulated error sum
int32_t PWM_last_main = 0;              //

int32_t TailDuty = 0;                   // Tail Duty:
int32_t d_lastError_tail = 0;                //
int32_t i_sum_tail = 0;                // Accumulated error sum
int32_t PWM_last_tail = 0;              //

// External Vars
extern int32_t yaw;                     // Current Yaw: Calculated in helicopterSignal.c
extern bool StartupAligned;             //

// Mode
typedef enum { USERCTL=0, START, LANDING, LANDED} modeType;
modeType MODE;
static char *enumStrings[] = {"USERCTL", "START ", "LANDING", "LANDED"}; //Used for display

//********************************************************


// PID controller for the main motor
uint32_t Maincontroller(int32_t target, int32_t current){

    // Scales the values up by a constant so for example 1000 * 0.01 can be 1010 instead of 1 * 0.01 getting 1.01
    // because decimals are inacurate and even small changes from rounding could be a problem
    target = target * DUTYSCALER;
    current = current * DUTYSCALER;
    int32_t i_Max = 10000 * DUTYSCALER;
    int32_t i_Min = 0;
    float Kp = 0.7;
    float Ki = 0.01;
    float Kd = 0.1;
    int32_t error = target - current;

    // Proportional: The error times the proportional coefficent (Kp)
    int32_t P = error * Kp;

    // Add the current Error to the error sum
    i_sum_main += error / DUTYSCALER;

    // Limit the summed error to between i_max and i_min
    if (i_sum_main > i_Max) i_sum_main = i_Max;
    else if (i_sum_main < i_Min) i_sum_main = i_Min;

    // Integral: Multiply the sum by the integral coefficent (Ki)
    int32_t I = Ki * i_sum_main;

    // Derivative: Calculate change in error between now and last time through the controller
    // then multiply by the differential coefficent (Kd)
    int32_t D = Kd * (d_lastError_main - error);

    // Store error to be used to calculate the change next time
    d_lastError_main = error;

    // Combine the proportional, intergral and dirrivetave components and then scales back down.
    //      (looking at it again im not sure why this isn't just "P + I + D" as the previous
    //      duty cycle shouldn't matter, I'll test changing this)
    int32_t PWM_Duty = (P + I + D) / DUTYSCALER;

    // Limit the duty cycle to between 95 and 5
    if (PWM_Duty > 95) PWM_Duty = PWM_MAX;
    else if (PWM_Duty < 5) PWM_Duty = PWM_MIN;

    PWM_last_main = PWM_Duty;
    return PWM_Duty;
}

// PID controller for the tail motor [ See above for comments ]
uint32_t Tailcontroller(int32_t target, int32_t current){

    target = target * DUTYSCALER;
    current = current * DUTYSCALER;
    int32_t i_Max = 10000 * DUTYSCALER;
    int32_t i_Min = 0;
    float Kp = 0.7;
    float Ki = 0.03;
    float Kd = 0.1;
    int32_t error = target - current;

    // Proportional
    int32_t P = error * Kp;

    // Integral
    i_sum_tail += error / DUTYSCALER;

    // Limit sum
    if (i_sum_tail > i_Max) i_sum_tail = i_Max;
    else if (i_sum_tail < i_Min) i_sum_tail = i_Min;

    // Integral
    int32_t I = Ki * i_sum_tail;

    // Derivative
    int32_t D = Kd * (d_lastError_tail - error);
    d_lastError_tail = error;

    int32_t PWM_Duty = (P + I + D) / DUTYSCALER;

    // Limit PWM to specification
    if (PWM_Duty > 95) PWM_Duty = PWM_MAX;
    else if (PWM_Duty < 5) PWM_Duty = PWM_MIN;

    PWM_last_tail = PWM_Duty;
    return PWM_Duty;
}


// Intialise UART Port A : Pin 0|1
void UARTinitialise(void){

    // Enable GPIO port A which is used for UART0 pins.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configure the pin muxing for UART0 functions on port A0 and A1.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    // Enable UART0 so that we can configure the clock.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Select the alternate (UART) function for these pins.
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Initialize the UART for console I/O.
    UARTStdioConfig(0, BAUD_RATE, UART_CLK_FREQ);

}

// Send Display message to UART
void UARTSendMessage (char *buffer){

    //Loop while there are more chars to send
    while(*buffer){

        // Write next char to UART Tx FIFO
        UARTCharPut(UART0_BASE, *buffer);
        buffer++;
    }

    // Add a new line after each buffer
    UARTprintf("\n");

}

// Send Display message to OLED using all 4 buffers
void OLEDSendMessage (char *l0Buf, char *l1Buf, char *l2Buf, char *l3Buf){

    //Display Yaw
    OLEDStringDraw (l0Buf, 0, 0);
    //Display Altitude
    OLEDStringDraw (l1Buf, 0, 1);
    // Draw Duty Cycles
    OLEDStringDraw (l2Buf, 0, 2);
    // Draw Mode
    OLEDStringDraw (l3Buf, 0, 3);

}

// UART/OLED Display
void Display(int32_t alt, int32_t yaw){

    // Orbit OLED handles up to 16 characters across
    char line0Buffer[17]; // Target / current Yaw
    char line1Buffer[17]; // Target / current Alt
    char line2Buffer[17]; // Duty cycles for Main/Tail Motors
    char line3Buffer[17]; // Mode - Perhaps append to existing buffer

    // Fill our buffers
    sprintf (line0Buffer, "Y: %3.2f [%3d]\n", fCalculateDegrees(yaw), targetYaw);
    sprintf (line1Buffer, "A: %3d [%4d]\n", alt, targetAlt);
    sprintf (line2Buffer, "MD %3d TD %3d\n", MainDuty, TailDuty);
    sprintf (line3Buffer, "[%s]\n", enumStrings[MODE]);

    // Send to OLED [ All Lines ]
    OLEDSendMessage(line0Buffer, line1Buffer, line2Buffer, line3Buffer);

    // Send to UART [ Per Line ]
    UARTSendMessage(line0Buffer);
    UARTSendMessage(line1Buffer);
    UARTSendMessage(line2Buffer);
    UARTSendMessage(line3Buffer);

}


// Start up sequence
void setStartupSequence(){
	// Turn Tail Rotor on, make sure Main Rotor is off
	SetMainOn(false);
	SetTailOn(true);

    // If we haven't started the program facing reference
    if (StartupAligned == false){

        // Retrieve current alt
        uint32_t alt = i32CalcAltPercent();
        // Tell our heli to rotate CW
        targetYaw = yaw + 10;
        // Required for resetting purposes
        targetAlt = alt - 10;

    } else {

        // If in MODE:START and we're already facing reference set to LANDED
        MODE = LANDED;

        landTrigger_descend = false;

        SetMainOn(false);
        SetTailOn(false);

        yaw = 0;
        targetYaw = 0;
        targetAlt = 0;
        calcAltLimits();
        // yaw = targetYaw = targetAlt = i_sum_main = 0;
        i_sum_main = 0;
    }
}

// Sets target yaw and once met, sets altitude then hands
// back control to the user with (MODE = USER)
void landSequence(int32_t alt){

    // This should never run if we have control. Sanity check.
    if ( MODE == USERCTL ) return;

    // Set yaw target to origin (round always returns an int)
    targetYaw = round(fCalculateDegrees(yaw) / 360.0) * 360;

    // If we've reached a multiple of reference
    if ( yaw == fCalculateFromDegrees(targetYaw)){

        // Begin descending
        //targetAlt = 0;

        // Set flag to land
    	landTrigger_descend = true;
        
        // Exit before we check altitude
        //return;
    }

    // Land smoothlyerable
    if (landTrigger_descend){
    	targetAlt = alt - 8;
    }

    // If we're on the ground
    if ( alt <= 1 ){

        // We've landed AND facing origin, change MODE
        MODE = LANDED;
        landTrigger_descend = false;

        // Ensure we set the target altitude to zero in case a bounce triggers this
        targetAlt = 0;

        // Turn off our motors
        SetMainOn(false);
        SetTailOn(false);

        // Eliminate accumulated error sum
        i_sum_tail = 0;
        i_sum_main = 0;
     }

}



// Interrupt handler for Systick
void SysTickIntHandler (void){

    static uint8_t ticks = 0;

    // Slow tick rate ~250
    const uint8_t ticksPerSlow = SYSTICK_RATE_HZ / DISPLAYTICK_RATE_HZ;

    // Every 100/4=25 systicks, set display flag
    if (++ticks >= ticksPerSlow){
        ticks = 0;
        displayTick = true;
    }

}


// Initialises both Systick period and interrupts
void InitSysTick (void){

    SysCtlClockSet (SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL |
                    SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Set the PWM clock rate (using the prescaler)
    SysCtlPWMClockSet(PWM_DIVIDER_CODE);

    // Set up period as a function of the system clock
    SysTickPeriodSet(SysCtlClockGet() / SYSTICK_RATE_HZ);

    // Register interrupt handler
    SysTickIntRegister(SysTickIntHandler);

    // Enable interrupt and device
    SysTickIntEnable();
    SysTickEnable();

}

// Alters both Alt and Yaw user input events
// UP and DOWN btns effecting targetAlt is limited to range (0 - 100)
void handleUSERCTLInput(){

    if ( (checkButton (UP) == PUSHED) && (targetAlt < 100) ){

        targetAlt += 10;
    }
    else if ( (checkButton (DOWN) == PUSHED) && (targetAlt > 0) ){

        targetAlt -= 10;
    }
    else if (checkButton (LEFT) == PUSHED){

        targetYaw -= 15.0;
    }
    else if (checkButton (RIGHT) == PUSHED){

        targetYaw += 15.0;
    }

}

// Handles non-directional user input
/*  - If we have just started the program and PA7 is High
    - Do not move out of LANDED MODE until switchUp is Low first
    - [bool firstRun] is set during intialization : [Line 86]
      this is set to false when we detect switchUp == false (low)
      we never set it to true outside of initialization. */
void handleGeneralInput(void){

    if ( checkButton(RESET) == 0 ){
            SysCtlReset();
    }

    // Read MODETOGGLE[PA7]
    bool switchUp = (GPIOPinRead (GPIO_PORTA_BASE, GPIO_PIN_7) == GPIO_PIN_7);

    if (switchUp){ // PA7 HIGH

        // Bail if program started with switch up
        if (firstRun) return;

        // Handle MODE changes
        if (MODE == LANDED){
            MODE = USERCTL;
            i_sum_main = 0;
			i_sum_tail = 0;
            // Start our motors
            SetMainOn(true);
            SetTailOn(true);
        }

    } else { // LOW

        // Set application ready for [PA7 HIGH]
        firstRun = false;

        // Set to Landing Sequence
        if (MODE == USERCTL){
            MODE = LANDING;
        }

    }

}


// Our Initialization/Boot sequence
void initialization(void){

    // As a precaution, make sure that the peripherals used are reset
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_GPIO);
    SysCtlPeripheralReset (PWM_MAIN_PERIPH_PWM);  // Main Rotor PWM
    SysCtlPeripheralReset (PWM_TAIL_PERIPH_PWM);  // Main Rotor PWM
    SysCtlPeripheralReset (UP_BUT_PERIPH);        // UP button GPIO
    SysCtlPeripheralReset (DOWN_BUT_PERIPH);      // DOWN button GPIO
    SysCtlPeripheralReset (LEFT_BUT_PERIPH);        // LEFT button GPIO
    SysCtlPeripheralReset (RIGHT_BUT_PERIPH);      // RIGHT button GPIO
    SysCtlPeripheralReset (RESET_BUT_PERIPH);      // RESET button GPIO
    SysCtlPeripheralReset (MODETOGGLE_BUT_PERIPH);      // switch GPIO


    // Enable interrupts to the processor.
    IntMasterEnable();

    // Initialise our Pins
    InitPin();

    // Initialise Systick and interrupts
    InitSysTick();

    // Initialise altitue read voltage at start as equaling 0% percentage and setting 100%.
    calcAltLimits();

    // Initialise both displays.
    OLEDInitialise();
    UARTinitialise();

    // Init Mode [ CHANGE ME TO LANDED ]
    MODE = START;

    // Initialize buttons and PWM
    initButtons ();
    initialisePWM ();

    SysCtlDelay(1000);

    // Set switch as first viewed/polled
    // It is assumed that both reset methods will re-call initialize()
    firstRun = true;

}

//
int main (void){

    // Init board
    initialization();

    // Main loop
    while (true){

        // Calc altitude before input
        SampleStoreAltitude();
        int32_t alt = i32CalcAltPercent(); // Get Current Altitude
        updateButtons();
        switch( MODE ){

            case START:
                setStartupSequence();
                break;

            case LANDING:
                landSequence(alt);
                break;

            case USERCTL:
                // Handle input moved into own function
                handleUSERCTLInput();
                break;

            case LANDED:
                break; // Null case - Do nothing
        }

        handleGeneralInput();

        // After input //

        MainDuty = Maincontroller(targetAlt, alt);
        TailDuty = Tailcontroller(fCalculateFromDegrees(targetYaw), yaw);

        setMainPWM(FREQUENCY, MainDuty);
        setTailPWM(FREQUENCY, TailDuty);

        // Display if flagged
        if (displayTick){
            displayTick = false;
            //Send current alt/yaw for Display
            Display(alt, yaw);
        }
    }

}
