/*
 * helicopterSignal.c
 *
 *  Created on: May 17, 2017
 *      Authors Group 51 : fbf10, ljd77
 *  Last updated: May 17, 2017
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h" //Needed for pin configure
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "stdio.h"
#include "buttons2.h"
#include "lib/OrbitOLEDInterface.h"
#include "circBufT.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/adc.h"


//********************************************************
// Constants
//********************************************************
#define MAX_24BIT_VAL 0X0FFFFFF
#define BUF_SIZE 20

//********************************************************
// Global Vars
//********************************************************
static circBuf_t g_inBuffer;            // uint32_t buffer of size BUF_SIZE
volatile static uint32_t g_u32IntCnt;   // Monitors interrupts
static int32_t minAltVolt = 0;         // Highest voltage <- Lowest altitude
static int32_t maxAltVolt = 0;          // Lowest voltage  <- highest altitude
static int32_t state = 0;               // Current state of yaw
volatile int32_t yaw = 0;               // Signed to allow for negatives
volatile bool StartupAligned = false;   //

//Complete (Tested)
void PinChangeIntHandler (void){

	// Clear the interrupt (documentation recommends doing this early)
	GPIOIntClear (GPIO_PORTB_BASE, GPIO_PIN_0);
	GPIOIntClear (GPIO_PORTB_BASE, GPIO_PIN_1);

    // Read the pin
    int32_t chA;
    int32_t chB;
    int32_t PortVal;
    PortVal = GPIOPinRead (GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    chA = ((PortVal & GPIO_PIN_0) == GPIO_PIN_0);
    chB = ((PortVal & GPIO_PIN_1) == GPIO_PIN_1);

    // Cache old state
    int32_t oldState = state;

    // Set new state
    if ( chA == 0 && chB == 0 )
        state = 0;
    else if ( chA == 0 && chB == 1 )
        state = 1;
    else if ( chA == 1 && chB == 1 )
        state = 2;
    else if ( chA == 1 && chB == 0 )
        state = 3;
    //

    // Determine rotation direction
    if ( oldState == (state+3) % 4) // +3 and -1 are equivalent
        yaw++;  // Clockwise
    else if ( oldState == (state+1) % 4)
        yaw--;  // AntiClockwise
}

void ReferenceIntHandler (void){

	// Clear the interrupt (documentation recommends doing this early)
	GPIOIntClear (GPIO_PORTC_BASE, GPIO_PIN_4);
	if (StartupAligned == false){

	    // If we are at reference, set yaw and flag that we're aligned.
		if (GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4) == GPIO_PIN_4){
			yaw = 0;
			StartupAligned = true;
		}
	}

}

void calcAltLimits(){

	minAltVolt = ui32SampleAltitude();
    //2^12 val is 3.3V
    //0 val is 0V
    //0.8V is 2^12 / (3.3/0.8) val
    //992.9696969 val is 0.8V
    maxAltVolt = minAltVolt - 993;
}

//Complete (tested)
void InitPin (void) {

    // Enable the port used for input = Port B
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOB); // Yaw
    // Enable the port used for the reference signal = Port C
    SysCtlPeripheralEnable (SYSCTL_PERIPH_GPIOC); // Reference
    //
    // Register the handler for Port B and C into the vector table
    GPIOIntRegister (GPIO_PORTB_BASE, PinChangeIntHandler);
    GPIOIntRegister (GPIO_PORTC_BASE, ReferenceIntHandler);
    //
    // Configure the pins used:  input on PB0 (J1-03) and PB1 (J1-04)
    GPIOPinTypeGPIOInput (GPIO_PORTB_BASE, GPIO_PIN_0); // Channel A
    GPIOPinTypeGPIOInput (GPIO_PORTB_BASE, GPIO_PIN_1); // Channel B
    // Configure the pin used for Reference also: PC4
    GPIOPinTypeGPIOInput (GPIO_PORTC_BASE, GPIO_PIN_4); // Reference
    //
    // Set up the pin change interrupt (both edges)
    GPIOIntTypeSet (GPIO_PORTB_BASE, GPIO_PIN_0, GPIO_BOTH_EDGES);
    GPIOIntTypeSet (GPIO_PORTB_BASE, GPIO_PIN_1, GPIO_BOTH_EDGES);
    GPIOIntTypeSet (GPIO_PORTC_BASE, GPIO_PIN_4, GPIO_BOTH_EDGES);
    //
    // Enable the pin change interrupt
    GPIOIntEnable (GPIO_PORTB_BASE, GPIO_PIN_0);
    GPIOIntEnable (GPIO_PORTB_BASE, GPIO_PIN_1);
    GPIOIntEnable (GPIO_PORTC_BASE, GPIO_PIN_4);
    //
    // Configure ADC Sequence
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_4);
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH9 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 3);
    ADCIntClear(ADC0_BASE, 3);

    initCircBuf(&g_inBuffer, BUF_SIZE);
}


// Convert from int(Yaw) to Degrees as a float
float fCalculateDegrees (int32_t yaw){

    float d = 0.0;
    //112 slots per full rotation
    //however each sensor increments/decrements the yaw by 1.
    //So 448 interrupts will be generated per full rotation
    d = yaw * 360.0 / 448;
    return d;
}

// Convert from float(degree) to Yaw as an int
int32_t fCalculateFromDegrees (float degree){

	int32_t yaw = 0.0;
    //112 slots per full rotation
    //however each sensor increments/decrements the yaw by 1.
    //So 448 interrupts will be generated per full rotation
	yaw = round(degree * 448 / 360.0);
    return yaw;
}


// Sample our altitude
uint32_t ui32SampleAltitude (void){

    uint32_t        uiValue = 0; // a single ADC value
    uint32_t        ulAIN2[1];   // The ADC sequencer uses a FIFO buffer

    //Initiate ADC Conversion
    ADCProcessorTrigger(ADC0_BASE, 3);

    while(!ADCIntStatus(ADC0_BASE, 3, false));

    ADCSequenceDataGet(ADC0_BASE, 3, ulAIN2);

    uiValue =  ulAIN2[0];       // select a single value from the FIFO buffer

    return uiValue;
}


// Place the sampled altitude value measured into Phil's FIFO buffer
void SampleStoreAltitude (void){

    //uses above function to sample altitidue then stores it in the buffer
    uint32_t ui32Sample = ui32SampleAltitude();

    // write ADC value to Phil's circular buffer
    writeCircBuf (&g_inBuffer, ui32Sample);

    // Legacy code : OBSOLETE
    //if (g_inBuffer.windex == 0) {
    //        g_inBuffer.windex = g_inBuffer.windex;
    //};
}

// Calculate and return an average from our buffer
int32_t i32CalcAltPercent (void){

    // Sum buffer values
    uint32_t i;
    uint32_t sum = 0;
    for ( i=0; i<BUF_SIZE; i++ ) {
        sum = sum + readCircBuf( &g_inBuffer );
    }

    // Compute average
    uint32_t average = sum / BUF_SIZE;

    //voltage value above minalt
    int32_t raised = minAltVolt - average;

    //interger divison, if percentage has bugs check here
    int32_t onePercent = (minAltVolt - maxAltVolt) / 100;

    // and here
    int32_t altPercent = raised / onePercent;

    return altPercent;
}
