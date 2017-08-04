/*
 * OrbitOLEDInterface.h
 *
 *  Created on: 23/11/2016
 *      Author: mdp46
 *	Modified on: 08/03/2017
 */

#ifndef ORBITOLEDINTERFACE_H_
#define ORBITOLEDINTERFACE_H_


/*
 * OLEDStringDraw
 * 		return:		void
 * 		input:		*pcStr	zero terminated character string
 * 					ulColumn	Character column in x axis
 * 					ulRow		Character row in y axis
 *
 * 		purpose:	Prints string in character row and column specified
 *
 * 		Note: 8x8 pixel character rows and columns are used.
 *
 *		TODO: Update types to stdint.h datatypes.
 *
 */

void OLEDStringDraw(const char *pcStr, unsigned long ulColumn, unsigned long ulRow);

/*
 * OLEDInitialise
 *   	return: 	void
 *   	input: 		void
 *
 *   	purpose:	Runs the initialise routines for the OLED display
 */
void OLEDInitialise (void);


#endif /* ORBITOLEDINTERFACE_H_ */
