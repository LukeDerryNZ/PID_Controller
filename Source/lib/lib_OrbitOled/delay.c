/************************************************************************/
/*																		*/
/*	delay.c	--	Time Delay Functions									*/
/*																		*/
/************************************************************************/
/*	Author: 	Gene Apperson											*/
/*	Copyright 2013, Digilent Inc.										*/
/************************************************************************/
/*  Module Description: 												*/
/*																		*/
/*																		*/
/************************************************************************/
/*  Revision History:													*/
/*																		*/
/*	04/29/2011(GeneA): created for PmodOLED								*/
/*	04/04/2013(JordanR):  Ported for Stellaris LaunchPad + Orbit BP		*/
/*	06/06/2013(JordanR):  Prepared for release							*/
/*																		*/
/************************************************************************/


/* ------------------------------------------------------------ */
/*				Include File Definitions						*/
/* ------------------------------------------------------------ */

#include "delay.h"
#include "LaunchPad.h"
#include "OrbitBoosterPackDefs.h"

/* ------------------------------------------------------------ */
/*				Local Type Definitions							*/
/* ------------------------------------------------------------ */


/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */


/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */


/* ------------------------------------------------------------ */
/*				Forward Declarations							*/
/* ------------------------------------------------------------ */


/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */
/***	DelayInit
**
**	Parameters:
**		none
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Initialized the hardware for use by delay functions. This
**		initializes Timer 1 to count
*/

void
DelayInit()	
	{

	/* Configure Timer 1. 
	*/
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

	TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC_UP);
	TimerEnable(TIMER1_BASE, TIMER_A);


}

/* ------------------------------------------------------------ */
/***	DelayMs
**
**	Parameters:
**		cms			- number of milliseconds to delay
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Delay the requested number of milliseconds. Uses Timer1.
*/

void
DelayMs(int cms)
	{
	int		ims;

	for (ims=0; ims<cms; ims++) {
		/*
		 * Clear Timer1
		 */
		HWREG(TIMER1_BASE + TIMER_O_TAV) = 0;
		while (TimerValueGet(TIMER1_BASE, TIMER_A) < cntMsDelay);
	}

}		

/* ------------------------------------------------------------ */
/***	ProcName
**
**	Parameters:
**
**	Return Value:
**
**	Errors:
**
**	Description:
**
*/

/* ------------------------------------------------------------ */

/************************************************************************/

