/*
 * OrbitBoosterPackDefs.h
 *
 *  Created on: Apr 3, 2013
 *      Author: Jretz
 *  Modified on: Feb 27, 2017
 *       mod by: Matthew Pike
 */

#ifndef ORBITBOOSTERPACKDEFS_H_
#define ORBITBOOSTERPACKDEFS_H_

/*
 * Include LaunchPad.h for GPIO Defs
 */
#include "LaunchPad.h"

/*
 * General
 */
#define OUTPUT			GPIO_DIR_MODE_OUT
#define INPUT			GPIO_DIR_MODE_IN
#define HARDWARE		GPIO_DIR_MODE_HW
#define LOW				0
#define HIGH			1

/*
 * OLED Control
 */
#define SCK_OLEDPort	GPIO_PORTD_BASE     //PD0
#define SDI_OLEDPort	GPIO_PORTD_BASE     //PD3
#define VBAT_OLEDPort	GPIO_PORTE_BASE     //PE1
#define VDD_OLEDPort	GPIO_PORTE_BASE     //PE2
#define nCS_OLEDPort	GPIO_PORTD_BASE     //PD1
#define nRES_OLEDPort	GPIO_PORTE_BASE     //PE5
#define nDC_OLEDPort	GPIO_PORTD_BASE     //PD7
#define SCK_OLED_PIN	GPIO_PIN_0
#define SDI_OLED_PIN	GPIO_PIN_3
#define SCK_OLED		0x00030001	// GPIO_PD0_SSI3CLK <- hard coded, should be in pin_map.h
#define SDI_OLED		0x00030C01	// GPIO_PD3_SSI3TX  <- hard coded, should be in pin_map.h
#define VBAT_OLED		GPIO_PIN_1
#define VDD_OLED		GPIO_PIN_2
#define nCS_OLED		GPIO_PIN_1
#define nRES_OLED		GPIO_PIN_5
#define nDC_OLED		GPIO_PIN_7

/*
 * Analog Control
 */
#define AINPort			GPIO_PORTE_BASE
#define AIN				GPIO_PIN_3		// AIN0

/*
 * Switch Control
 */
#define SWTPort			GPIO_PORTA_BASE
#define SWT1Port		GPIO_PORTA_BASE
#define SWT2Port		GPIO_PORTA_BASE
#define SWT1			GPIO_PIN_7
#define SWT2			GPIO_PIN_6

/*
 * Button Control
 */
#define BTN1Port		GPIO_PORTD_BASE
#define BTN2Port		GPIO_PORTE_BASE
#define BTN1			GPIO_PIN_2
#define BTN2			GPIO_PIN_0

/*
 * LED Control
 */
#define LED1Port		GPIO_PORTC_BASE
#define LED2Port		GPIO_PORTC_BASE
#define LED3Port		GPIO_PORTD_BASE
#define	LED4Port		GPIO_PORTB_BASE
#define LED1			GPIO_PIN_6
#define LED2			GPIO_PIN_7
#define LED3			GPIO_PIN_6
#define LED4			GPIO_PIN_5

/*
 * Pmod Control
 */
#define JAPort			GPIO_PORTA_BASE
#define JA1Port			GPIO_PORTA_BASE
#define JA2Port			GPIO_PORTA_BASE
#define JA3Port			GPIO_PORTA_BASE
#define	JA4Port			GPIO_PORTA_BASE
#define JBInPort		GPIO_PORTB_BASE	// Inner Two Pins in Row
#define JBOutPort		GPIO_PORTC_BASE	// Outer Two Pins in Row
#define JB1Port			GPIO_PORTC_BASE
#define JB2Port			GPIO_PORTB_BASE
#define JB3Port			GPIO_PORTB_BASE
#define	JB4Port			GPIO_PORTC_BASE
#define JA1				GPIO_PIN_3
#define JA2				GPIO_PIN_5
#define JA3				GPIO_PIN_4
#define JA4				GPIO_PIN_2
#define JB1				GPIO_PIN_5		// U1CTS
#define JB2				GPIO_PIN_1
#define JB3				GPIO_PIN_0
#define JB4				GPIO_PIN_4		// U1RTS

/*
 * UART Control (on Pmod Header JB)
 */
#define U1RXTXPort		GPIO_PORTB_BASE
#define U1RTSPort		GPIO_PORTC_BASE
#define U1CTSPort		GPIO_PORTC_BASE
#define UART1CTS		JB1
#define UART1TX			JB2
#define UART1RX			JB3
#define UART1RTS		JB4

/*
 * SSI Control (on Pmod Header JA)
 */
#define SSI0Port		GPIO_PORTA_BASE
#define SSI0FSS			JA1
#define SSI0TX			JA2
#define SSI0RX			JA3
#define SSI0CLK			JA4

/*
 * I2C Control
 */
#define I2CPort			GPIO_PORTB_BASE
#define I2CSDAPort		GPIO_PORTB_BASE
#define I2CSCLPort		GPIO_PORTB_BASE
#define I2CSDA_PIN		GPIO_PIN_3
#define I2CSCL_PIN		GPIO_PIN_2
#define I2CSCL			0x00010803			// GPIO_PB2_I2C0SCL
#define I2CSDA			0x00010C03			// GPIO_PB3_I2C0SDA

/*
 * Accl Interrupt Pins
 */
#define ACCL_INT1Port	GPIO_PORTB_BASE
#define ACCL_INT2Port	GPIO_PORTE_BASE
#define ACCL_INT1		GPIO_PIN_4
#define	ACCL_INT2		GPIO_PIN_4

/*
 * Address Reference
 * The A2, A1, A0 pins are default tied high
 * See device datasheet for address changes
 * if these pins are altered
 */
#define	EEPROM			0
#define TEMP			1
#define ACCL			2
#define TEMPADDR		0x4F
#define ACCLADDR		0x1D
#define EEPROMADDR		0x54		// A1 and A0 are tied internally low
#define READ			true
#define WRITE			false


#endif /* ORBITBOOSTERPACKDEFS_H_ */
