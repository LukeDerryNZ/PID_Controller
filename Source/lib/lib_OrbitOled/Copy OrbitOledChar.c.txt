/************************************************************************/
/*																		*/
/*	OrbitledChar.c	--	Character Output Routines for OLED Display		*/
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

extern int		xcoOledCur;
extern int		ycoOledCur;

extern char *	pbOledCur;
extern char		mskOledCur;
extern int		bnOledCur;
extern int		fOledCharUpdate;

extern char		rgbOledBmp[];

extern int		dxcoOledFontCur;
extern int		dycoOledFontCur;

extern	char *	pbOledFontCur;
extern	char *	pbOledFontUser;

/* ------------------------------------------------------------ */
/*				Local Variables									*/
/* ------------------------------------------------------------ */

int		xchOledCur;
int		ychOledCur;

int		xchOledMax;
int		ychOledMax;

char *	pbOledFontExt;

char	rgbOledFontUser[cbOledFontUser];

/* ------------------------------------------------------------ */
/*				Forward Declarations							*/
/* ------------------------------------------------------------ */

void	OrbitOledDrawGlyph(char ch);
void	OrbitOledAdvanceCursor();

/* ------------------------------------------------------------ */
/*				Procedure Definitions							*/
/* ------------------------------------------------------------ */
/***	OrbitOledSetCursor
**
**	Parameters:
**		xch			- horizontal character position
**		ych			- vertical character position
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Set the character cursor position to the specified location.
**		If either the specified X or Y location is off the display, it
**		is clamped to be on the display.
*/

void
OrbitOledSetCursor(int xch, int ych)
	{

	/* Clamp the specified location to the display surface
	*/
	if (xch >= xchOledMax) {
		xch = xchOledMax-1;
	}

	if (ych >= ychOledMax) {
		ych = ychOledMax-1;
	}

	/* Save the given character location.
	*/
	xchOledCur = xch;
	ychOledCur = ych;

	/* Convert the character location to a frame buffer address.
	*/
	OrbitOledMoveTo(xch*dxcoOledFontCur, ych*dycoOledFontCur);

}

/* ------------------------------------------------------------ */
/***	OrbitOledGetCursor
**
**	Parameters:
**		pxch		- pointer to variable to receive horizontal position
**		pych		- pointer to variable to receive vertical position
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Fetch the current cursor position
*/

void
OrbitOledGetCursor( int * pxch, int * pych)
	{

	*pxch = xchOledCur;
	*pych = ychOledCur;

}

/* ------------------------------------------------------------ */
/***	OrbitOledDefUserChar
**
**	Parameters:
**		ch		- character code to define
**		pbDef	- definition for the character
**
**	Return Value:
**		none
**
**	Errors:
**		Returns TRUE if successful, FALSE if not
**
**	Description:
**		Give a definition for the glyph for the specified user
**		character code. User definable character codes are in
**		the range 0x00 - 0x1F. If the code specified by ch is
**		outside this range, the function returns false.
*/

int
OrbitOledDefUserChar(char ch, char * pbDef)
	{
	char *	pb;
	int		ib;

	if (ch < chOledUserMax) {
		pb = pbOledFontUser + ch * cbOledChar;
		for (ib = 0; ib < cbOledChar; ib++) {
			*pb++ = *pbDef++;
		}
		return 1;
	}
	else {
		return 0;
	}

	}

/* ------------------------------------------------------------ */
/***	OrbitOledSetCharUpdate
**
**	Parameters:
**		f		- enable/disable automatic update
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Set the character update mode. This determines whether
**		or not the display is automatically updated after a
**		character or string is drawn. A non-zero value turns
**		automatic updating on.
*/

void
OrbitOledSetCharUpdate(int f)
	{

	fOledCharUpdate = (f != 0) ? 1 : 0;

}

/* ------------------------------------------------------------ */
/***	OrbitOledGetCharUpdate
**
**	Parameters:
**		none
**
**	Return Value:
**		returns current character update mode
**
**	Errors:
**		none
**
**	Description:
**		Return the current character update mode.
*/

int
OrbitOledGetCharUpdate()
	{

	return fOledCharUpdate;

}

/* ------------------------------------------------------------ */
/***	OrbitOledPutChar
**
**	Parameters:
**		ch			- character to write to display
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Write the specified character to the display at the current
**		cursor position and advance the cursor.
*/

void
OrbitOledPutChar(char ch)
	{

	OrbitOledDrawGlyph(ch);
	OrbitOledAdvanceCursor();
	if (fOledCharUpdate) {
		OrbitOledUpdate();
	}

}

/* ------------------------------------------------------------ */
/***	OrbitOledPutString
**
**	Parameters:
**		sz		- pointer to the null terminated string
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Write the specified null terminated character string to the
**		display and advance the cursor.
*/

void
OrbitOledPutString(char * sz)
	{

	while (*sz != '\0') {
		OrbitOledDrawGlyph(*sz);
		OrbitOledAdvanceCursor();
		sz += 1;
	}

	if (fOledCharUpdate) {
		OrbitOledUpdate();
	}

}

/* ------------------------------------------------------------ */
/***	OrbitOledDrawGlyph
**
**	Parameters:
**		ch		- character code of character to draw
**
**	Return Value:
**		none
**
**	Errors:
**		none
**
**	Description:
**		Renders the specified character into the display buffer
**		at the current character cursor location. This does not
**		affect the current character cursor location or the 
**		current drawing position in the display buffer.
*/

void
OrbitOledDrawGlyph(char ch)
	{
	char *	pbFont;
	char *	pbBmp;
	int		ib;

	if ((ch & 0x80) != 0) {
		return;
	}

	if (ch < chOledUserMax) {
		pbFont = pbOledFontUser + ch*cbOledChar;
	}
	else if ((ch & 0x80) == 0) {
		pbFont = pbOledFontCur + (ch-chOledUserMax) * cbOledChar;
	}

	pbBmp = pbOledCur;

	for (ib = 0; ib < dxcoOledFontCur; ib++) {
		*pbBmp++ = *pbFont++;
	}

}

/* ------------------------------------------------------------ */
/***	OrbitledAdvanceCursor
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
**		Advance the character cursor by one character location,
**		wrapping at the end of line and back to the top at the
**		end of the display.
*/

void
OrbitOledAdvanceCursor()
	{

	xchOledCur += 1;
	if (xchOledCur >= xchOledMax) {
		xchOledCur = 0;
		ychOledCur += 1;
	}
	if (ychOledCur >= ychOledMax) {
		ychOledCur = 0;
	}

	OrbitOledSetCursor(xchOledCur, ychOledCur);

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

