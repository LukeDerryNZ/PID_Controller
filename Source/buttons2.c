/*
 * buttons2.c
 *
 *  Created on:   May 2, 2017
 *  Authors:      Group 51 : fbf10, ljd77
 *  Last Updated: May 23, 2017
 * 
 * Support for a set of specific buttons on the Tiva/Orbit.
 * The buttons are: UP    [PE0]  DOWN  [PD2]
 *                  LEFT  [PF4]  RIGHT [PF0]
 *                  RESET [PA6]  MODE  [PA7]
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/debug.h"
#include "buttons2.h"


// *******************************************************
// Globals to module
// *******************************************************
static bool but_state[NUM_BUTS];	// Corresponds to the electrical state
static uint8_t but_count[NUM_BUTS];
static bool but_flag[NUM_BUTS];
static bool but_normal[NUM_BUTS];   // Corresponds to the electrical state

// *******************************************************
// initButtons: Initialise the variables associated with the set of buttons
// defined by the constants in the buttons2.h header file.
void initButtons (void){

	int i;

	// UP button (active HIGH)
    SysCtlPeripheralEnable (UP_BUT_PERIPH);
    GPIOPinTypeGPIOInput (UP_BUT_PORT_BASE, UP_BUT_PIN);
    GPIOPadConfigSet (UP_BUT_PORT_BASE, UP_BUT_PIN, GPIO_STRENGTH_2MA,
       GPIO_PIN_TYPE_STD_WPD);
    but_normal[UP] = UP_BUT_NORMAL;

	// DOWN button (active HIGH)
    SysCtlPeripheralEnable (DOWN_BUT_PERIPH);
    GPIOPinTypeGPIOInput (DOWN_BUT_PORT_BASE, DOWN_BUT_PIN);
    GPIOPadConfigSet (DOWN_BUT_PORT_BASE, DOWN_BUT_PIN, GPIO_STRENGTH_2MA,
       GPIO_PIN_TYPE_STD_WPD);
    but_normal[DOWN] = DOWN_BUT_NORMAL;

    // LEFT button (active LOW)
    SysCtlPeripheralEnable (LEFT_BUT_PERIPH);
    GPIOPinTypeGPIOInput (LEFT_BUT_PORT_BASE, LEFT_BUT_PIN);
    GPIOPadConfigSet (LEFT_BUT_PORT_BASE, LEFT_BUT_PIN, GPIO_STRENGTH_2MA,
       GPIO_PIN_TYPE_STD_WPU);
    but_normal[LEFT] = LEFT_BUT_NORMAL;

    // RIGHT button (active LOW)
    SysCtlPeripheralEnable (RIGHT_BUT_PERIPH);
    
    // Unlock PF0 for the right button:
    GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY;
    GPIO_PORTF_CR_R |= GPIO_PIN_0; //PF0 unlocked
    GPIO_PORTF_LOCK_R = GPIO_LOCK_M;
    
    GPIOPinTypeGPIOInput (RIGHT_BUT_PORT_BASE, RIGHT_BUT_PIN);
    GPIOPadConfigSet (RIGHT_BUT_PORT_BASE, RIGHT_BUT_PIN, GPIO_STRENGTH_2MA,
       GPIO_PIN_TYPE_STD_WPU);
    but_normal[RIGHT] = RIGHT_BUT_NORMAL;

    // RESET button (active LOW)
    SysCtlPeripheralEnable (RESET_BUT_PERIPH);
    GPIOPinTypeGPIOInput (RESET_BUT_PORT_BASE, RESET_BUT_PIN);
    GPIOPadConfigSet (RESET_BUT_PORT_BASE, RESET_BUT_PIN, GPIO_STRENGTH_2MA,
       GPIO_PIN_TYPE_STD_WPD);
    but_normal[RESET] = RESET_BUT_NORMAL;

    // MODE switch
    SysCtlPeripheralEnable (MODETOGGLE_BUT_PERIPH);
    GPIOPinTypeGPIOInput (MODETOGGLE_BUT_PORT_BASE, MODETOGGLE_BUT_PIN);
    GPIOPadConfigSet (MODETOGGLE_BUT_PORT_BASE, MODETOGGLE_BUT_PIN, GPIO_STRENGTH_2MA,
       GPIO_PIN_TYPE_STD_WPD);
    but_normal[MODETOGGLE] = MODETOGGLE_BUT_NORMAL;

	for (i = 0; i < NUM_BUTS; i++){
	
		but_state[i] = but_normal[i];
		but_count[i] = 0;
		but_flag[i] = false;
	}
}

// *******************************************************
// updateButtons: Function designed to be called regularly. It polls all
// buttons once and updates variables associated with the buttons if
// necessary.  It is efficient enough to be part of an ISR, e.g. from
// a SysTick interrupt.
// Debounce algorithm: A state machine is associated with each button.
// A state change occurs only after NUM_BUT_POLLS consecutive polls have
// read the pin in the opposite condition, before the state changes and
// a flag is set.  Set NUM_BUT_POLLS according to the polling rate.
void updateButtons (void){

	bool but_value[NUM_BUTS];
	int i;
	
	// Read the pins; true means HIGH, false means LOW
	but_value[UP] = (GPIOPinRead (UP_BUT_PORT_BASE, UP_BUT_PIN) == UP_BUT_PIN);
	but_value[DOWN] = (GPIOPinRead (DOWN_BUT_PORT_BASE, DOWN_BUT_PIN) == DOWN_BUT_PIN);
	but_value[LEFT] = (GPIOPinRead (LEFT_BUT_PORT_BASE, LEFT_BUT_PIN) == LEFT_BUT_PIN);
	but_value[RIGHT] = (GPIOPinRead (RIGHT_BUT_PORT_BASE, RIGHT_BUT_PIN) == RIGHT_BUT_PIN);
	but_value[RESET] = (GPIOPinRead (RESET_BUT_PORT_BASE, RESET_BUT_PIN) == RESET_BUT_PIN);
	
	// Iterate through the buttons, updating button variables as required
	for (i = 0; i < NUM_BUTS; i++){
	
        if (but_value[i] != but_state[i]){
        
        	but_count[i]++;
        	if (but_count[i] >= NUM_BUT_POLLS){
        	
        		but_state[i] = but_value[i];
        		but_flag[i] = true;	   // Reset by call to checkButton()
        		but_count[i] = 0;
        	}
        } else {
        	but_count[i] = 0;
    	}
	}
}

// *******************************************************
// checkButton: Function returns the new button logical state if the button
// logical state (PUSHED or RELEASED) has changed since the last call,
// otherwise returns NO_CHANGE.
uint8_t checkButton (uint8_t butName){

	if (but_flag[butName]){
	
		but_flag[butName] = false;
		if (but_state[butName] == but_normal[butName]){
			return RELEASED;
		} else {
			return PUSHED;
		}
	}
	return NO_CHANGE;
}

