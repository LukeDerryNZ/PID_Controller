/************************************************************************/
/*																		*/
/*	OrbitOled.c	--	Graphics Driver Library for OLED Display			*/
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
#include "OrbitOled.h"
#include "OrbitOledChar.h"
#include "OrbitOledGrph.h"

/* ------------------------------------------------------------ */
/*				Local Type Definitions							*/
/* ------------------------------------------------------------ */


/* ------------------------------------------------------------ */
/*				Global Variables								*/
/* ------------------------------------------------------------ */

extern char		rgbOledFont0[];
extern char		rgbOledFontUser[];
extern char		rgbFillPat[];

extern int		xchOledMax;
extern int		ychOledMax;

/* Coordinates of current pixel location on the display. The origin
** is at the upper left of the display. X increases to the right
** and y increases going down.
*/
int		xcoOledCur;
int		ycoOledCur;

char *	pbOledCur;			//address of byte corresponding to current location
int		bnOledCur;			//bit number of bit corresponding to current location
char	clrOledCur;			//drawing color to use
char *	pbOledPatCur;		//current fill pattern
int		fOledCharUpdate;

int		dxcoOledFontCur;
int		dycoOledFontCur;

char *	pbOledFontCur;
char *	pbOledFontUser;

/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */

/* This array is the offscreen frame buffer used for rendering.
** It isn't possible to read back frome the OLED display device,
** so display data is rendered into this offscreen buffer and then
** copied to the display.
*/
char	rgbOledBmp[cbOledDispMax];

/* ------------------------------------------------------------ */
/*				Forward Declarations							*/
/* ------------------------------------------------------------ */

void	OrbitOledHostInit();
void	OrbitOledDevInit();
void	OrbitOledDvrInit();
char	Ssi3PutByte(char bVal);
void	OrbitOledPutBuffer(int cb, char * rgbTx);

/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */
/***	OrbitOledInit
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
**		Initialize the OLED display subsystem.
*/

void
OrbitOledInit()
	{

	/* Init the LM4F120 peripherals used to talk to the display.
	*/
	OrbitOledHostInit();

	/* Init the memory variables used to control access to the
	** display.
	*/
	OrbitOledDvrInit();

	/* Init the OLED display hardware.
	*/
	OrbitOledDevInit();

	/* Clear the display.
	*/
	OrbitOledClear();

}

/* ------------------------------------------------------------ */
/***	OrbitOledHostInit
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
**		Perform Stellaris device initialization to prepare for use
**		of the OLED display.
*/

void
OrbitOledHostInit()
	{

	DelayInit();

	/* Initialize SSI port 3.
	*/
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);
	GPIOPinTypeSSI(SCK_OLEDPort, SCK_OLED_PIN);
	GPIOPinTypeSSI(SDI_OLEDPort, SDI_OLED_PIN);
	GPIOPinConfigure(SDI_OLED);
	GPIOPinConfigure(SCK_OLED);
	SSIClockSourceSet(SSI3_BASE, SSI_CLOCK_SYSTEM);
	SSIConfigSetExpClk(SSI3_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 8000000, 8);
	SSIEnable(SSI3_BASE);

	/* Make power control pins be outputs with the supplies off
	*/
	GPIOPinWrite(VBAT_OLEDPort, VBAT_OLED, VBAT_OLED);
	GPIOPinWrite(VDD_OLEDPort, VDD_OLED, VDD_OLED);
	GPIOPinTypeGPIOOutput(VBAT_OLEDPort, VBAT_OLED);	//VDD power control (1=off)
	GPIOPinTypeGPIOOutput(VDD_OLEDPort, VDD_OLED);		//VBAT power control (1=off)

	/* Make the Data/Command select, Reset, and SSI CS pins be outputs.
	 * The nDC_OLED pin is PD7 an is a special GPIO (it is an NMI pin)
	 * Therefore, we must unlock it first:
	 * 1. Write 0x4C4F434B to GPIOLOCK register to unlock the GPIO Commit register
	 * 2. Write to appropriate bit in the Commit Register (bit 7)
	 * 3. Re-lock the GPIOLOCK register
	*/
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0x4C4F434B;	// unlock
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 1 << 7; 		// allow writes
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0x0;			// re-lock
	GPIOPinWrite(nDC_OLEDPort, nDC_OLED, nDC_OLED);
	GPIOPinTypeGPIOOutput(nDC_OLEDPort, nDC_OLED);
	GPIOPinWrite(nDC_OLEDPort, nDC_OLED, nDC_OLED);
	GPIOPinWrite(nRES_OLEDPort, nRES_OLED, nRES_OLED);
	GPIOPinTypeGPIOOutput(nRES_OLEDPort, nRES_OLED);
	GPIOPinWrite(nCS_OLEDPort, nCS_OLED, nCS_OLED);
	GPIOPinTypeGPIOOutput(nCS_OLEDPort, nCS_OLED);

}

/* ------------------------------------------------------------ */
/***	OrbitOledDvrInit
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
**		Initialize the OLED software system
*/

void
OrbitOledDvrInit()
	{
	int		ib;

	/* Init the parameters for the default font
	*/
	dxcoOledFontCur = cbOledChar;
	dycoOledFontCur = 8;
	pbOledFontCur = rgbOledFont0;
	pbOledFontUser = rgbOledFontUser;

	for (ib = 0; ib < cbOledFontUser; ib++) {
		rgbOledFontUser[ib] = 0;
	}

	xchOledMax = ccolOledMax / dxcoOledFontCur;
	ychOledMax = crowOledMax / dycoOledFontCur;

	/* Set the default character cursor position.
	*/
	OrbitOledSetCursor(0, 0);

	/* Set the default foreground draw color and fill pattern
	*/
	clrOledCur = 0x01;
	pbOledPatCur = rgbFillPat;
	OrbitOledSetDrawMode(modOledSet);

	/* Default the character routines to automatically
	** update the display.
	*/
	fOledCharUpdate = 1;

}

/* ------------------------------------------------------------ */
/***	OrbitOledDevInit
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
**		Initialize the OLED display controller and turn the display on.
*/

void
OrbitOledDevInit()
	{

	/* We're going to be sending commands, so clear the Data/Cmd bit
	*/
	GPIOPinWrite(nDC_OLEDPort, nDC_OLED, LOW);

	/* Start by turning VDD on and wait a while for the power to come up.
	*/
	GPIOPinWrite(VDD_OLEDPort, VDD_OLED, LOW);
	DelayMs(1);

	/* Display off command
	*/
	Ssi3PutByte(0xAE);

	/* Bring Reset low and then high
	*/
	GPIOPinWrite(nRES_OLEDPort, nRES_OLED, LOW);
	DelayMs(1);
	GPIOPinWrite(nRES_OLEDPort, nRES_OLED, nRES_OLED);

	/* Send the Set Charge Pump and Set Pre-Charge Period commands
	*/
	Ssi3PutByte(0x8D);
	Ssi3PutByte(0x14);

	Ssi3PutByte(0xD9);
	Ssi3PutByte(0xF1);

	/* Turn on VCC and wait 100ms
	*/
	GPIOPinWrite(VBAT_OLEDPort, VBAT_OLED, LOW);
	DelayMs(100);

	/* Send the commands to invert the display.
	*/
	Ssi3PutByte(0xA1);			//remap columns
	Ssi3PutByte(0xC8);			//remap the rows

	/* Send the commands to select sequential COM configuration
	*/
	Ssi3PutByte(0xDA);			//set COM configuration command
	Ssi3PutByte(0x20);			//sequential COM, left/right remap enabled

	/* Send Display On command
	*/
	Ssi3PutByte(0xAF);

}

/* ------------------------------------------------------------ */
/***	OrbitOledClear
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
**		Clear the display. This clears the memory buffer and then
**		updates the display.
*/

void
OrbitOledClear()
	{

	OrbitOledClearBuffer();
	OrbitOledUpdate();

}

/* ------------------------------------------------------------ */
/***	OrbitOledClearBuffer
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
**		Clear the display memory buffer.
*/

void
OrbitOledClearBuffer()
	{
	int			ib;
	char *		pb;

	pb = rgbOledBmp;

	/* Fill the memory buffer with 0.
	*/
	for (ib = 0; ib < cbOledDispMax; ib++) {
		*pb++ = 0x00;
	}

}

/* ------------------------------------------------------------ */
/***	OrbitOledUpdate
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
**		Update the OLED display with the contents of the memory buffer
*/

void
OrbitOledUpdate()
	{
	int		ipag;
//	int		icol;
	char *	pb;

	pb = rgbOledBmp;

	for (ipag = 0; ipag < cpagOledMax; ipag++) {

		GPIOPinWrite(nDC_OLEDPort, nDC_OLED, LOW);

		/* Set the page address
		*/
		Ssi3PutByte(0x22);		//Set page command
		Ssi3PutByte(ipag);		//page number

		/* Start at the left column
		*/
		Ssi3PutByte(0x00);		//set low nibble of column
		Ssi3PutByte(0x10);		//set high nibble of column

		GPIOPinWrite(nDC_OLEDPort, nDC_OLED, nDC_OLED);

		/* Copy this memory page of display data.
		*/
		OrbitOledPutBuffer(ccolOledMax, pb);
		pb += ccolOledMax;

	}

}

/* ------------------------------------------------------------ */
/***	OrbitOledPutBuffer
**
**	Parameters:
**		cb		- number of bytes to send/receive
**		rgbTx	- pointer to the buffer to send
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Send the bytes specified in rgbTx to the slave 
*/

void
OrbitOledPutBuffer(int cb, char * rgbTx)
	{
	int32_t				ib;
	uint32_t	    bTmp;

	/* Bring the slave select line low
	*/
	GPIOPinWrite(nCS_OLEDPort, nCS_OLED, LOW);

	/* Write/Read the data
	*/
	for (ib = 0; ib < cb; ib++) {
		/* Wait for transmitter to be ready
		*/
		while (SSIBusy(SSI3_BASE));

		/* Write the next transmit byte.
		*/
		SSIDataPut(SSI3_BASE, (uint32_t)*rgbTx++);

		/* Wait for receive byte.
		*/
		while (SSIBusy(SSI3_BASE));
		SSIDataGet(SSI3_BASE, &bTmp);

	}

	/* Bring the slave select line high
	*/
	GPIOPinWrite(nCS_OLEDPort, nCS_OLED, nCS_OLED);
	
}

/* ------------------------------------------------------------ */
/***	Ssi3PutByte
**
**	Parameters:
**		bVal		- byte value to write
**
**	Return Value:
**		Returns byte read
**
**	Errors:
**		none
**
**	Description:
**		Write/Read a byte on SSI port 3
*/

char
Ssi3PutByte(char bVal)
	{
	uint32_t	        bRx;

	/* Bring the slave select line low
	*/
	GPIOPinWrite(nCS_OLEDPort, nCS_OLED, LOW);

	/* Wait for transmitter to be ready
	*/
	while (SSIBusy(SSI3_BASE));

	/* Write the next transmit byte.
	*/
	SSIDataPut(SSI3_BASE, (uint32_t)bVal);

	/* Wait for receive byte.
	*/
	while (SSIBusy(SSI3_BASE));

	/* Put the received byte in the buffer.
	*/
	SSIDataGet(SSI3_BASE, &bRx);

	/* Bring the slave select line high
	*/
	GPIOPinWrite(nCS_OLEDPort, nCS_OLED, nCS_OLED);
	
	return (char)bRx;

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

