/************************************************************************/
/*																		*/
/*	OrbitOledGrph.h	--	Declarations for OLED Graphics Routines			*/
/*																		*/
/************************************************************************/
/*	Author:		Gene Apperson											*/
/*	Copyright 2013, Digilent Inc.										*/
/************************************************************************/
/*  File Description:													*/
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
/*					Miscellaneous Declarations					*/
/* ------------------------------------------------------------ */



/* ------------------------------------------------------------ */
/*					General Type Declarations					*/
/* ------------------------------------------------------------ */

/* ------------------------------------------------------------ */
/*					Object Class Declarations					*/
/* ------------------------------------------------------------ */



/* ------------------------------------------------------------ */
/*					Variable Declarations						*/
/* ------------------------------------------------------------ */



/* ------------------------------------------------------------ */
/*					Procedure Declarations						*/
/* ------------------------------------------------------------ */

void	OrbitOledSetDrawColor(char clr);
void	OrbitOledSetDrawMode(int mod);
int		OrbitOledGetDrawMode();
char *	OrbitOledGetStdPattern(int ipat);
void	OrbitOledSetFillPattern(char * pbPat);

void	OrbitOledMoveTo(int xco, int yco);
void	OrbitOledGetPos(int * pxco, int * pyco);
void	OrbitOledDrawPixel();
char	OrbitOledGetPixel();
void	OrbitOledLineTo(int xco, int yco);
void	OrbitOledDrawRect(int xco, int yco);
void	OrbitOledFillRect(int xco, int yco);
void	OrbitOledGetBmp(int dxco, int dyco, char * pbBmp);
void	OrbitOledPutBmp(int dxco, int dyco, char * pbBmp);
void	OrbitOledDrawChar(char ch);
void	OrbitOledDrawString(char * sz);

/* ------------------------------------------------------------ */

/************************************************************************/
