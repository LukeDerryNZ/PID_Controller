/*
 * OrbitOLEDInterface.c
 *
 *	This module provides function(s) which have the same inputs and behaviour
 *	as the RIT128x96x4StringDraw function from the EVK-LM3S1968 display.
 *
 *  Created on: 23/11/2016
 *      Author: mdp46
 *
 *   Modified : 08/03/2017
 *      Author: mdp46
 */

//Defines/includes for Orbit OLED:
#include "lib_OrbitOled/OrbitOled.h"
#include "lib_OrbitOled/delay.h"
#include "lib_OrbitOled/FillPat.h"
#include "lib_OrbitOled/LaunchPad.h"
#include "lib_OrbitOled/OrbitBoosterPackDefs.h"
#include "lib_OrbitOled/OrbitOled.h"
#include "lib_OrbitOled/OrbitOledChar.h"
#include "lib_OrbitOled/OrbitOledGrph.h"

//*****************************************************************************
//
//!
//! Displays a string on the OLED display.
//!
//! \param pcStr is a pointer to the string to display.
//! \param ulColumn is the horizontal position to display the string, specified in
//! multiples of 8 pixels from the left edge of the display.
//! \param ulRow is the vertical position to display the string, specified in
//! multiples of 8 pixels from the top edge of the display.
//!
//! This function will draw a string on the display.  Only the ASCII characters
//! between 32 (space) and 126 (tilde) are supported; other characters will
//! result in random data being draw on the display (based on whatever appears
//! before/after the font in memory).  The font is mono-spaced, so characters
//! such as ``i'' and ``l'' have more white space around them than characters
//! such as ``m'' or ``w''.
//!
//! ---TODO: Check behaviour of OLEDStringDraw, for unexpected characters.
//!
//! If the drawing of the string reaches the right edge of the display, the
//! characters will be drawn on the next row.  Therefore, special care is 
//! required to avoid unintentionally overwriting the display on the next row.
//!
//! (Based on RIT128x96x4StringDraw from Stellaris EVK-LM3S1968)
//!
//!  Characters are 8 pixels wide (x axis)
//!		     	and 8 pixels tall (y axis)
//!
//! \return None.
//
//*****************************************************************************
void
OLEDStringDraw(char *pcStr, unsigned long ulColumn, unsigned long ulRow)
{

    //-------Use the Orbit Functions:---------

	int charX = ulColumn;	//Char index in X axis. (character column)
	int charY = ulRow;		//Char index in Y axis. (character row)

    //Put the cursor in the right place:
    OrbitOledSetCursor(charX, charY);

    //Print the string:
    OrbitOledPutString(pcStr);
}


/*****************************************************************************
 * OLEDInitialise
 *   	return: 	void
 *   	input: 		void
 *
 *   	purpose:	Runs the required initialiser routines for the OLED display
 *****************************************************************************/
void
OLEDInitialise (void){

	/*
	 * Initialize the OLED
	 */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);    //Need signals on GPIOD
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);    //Need signals on GPIOE

	OrbitOledInit();
}



