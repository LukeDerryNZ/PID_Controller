/*
 * LaunchPad.h
 *
 *Created on: Apr 3, 2013
 *      Author: Jordan Retz
 *Revised on: Oct 23, 2014
 *      Author: jnjuguna
 *
 *  Note: Make sure that is located under the "Includes" drop down
 *  in the Project. If not:
 *  	1. Go to Properties->Include Options and add this location
 *
 */



#ifndef LAUNCHPAD_H_
#define LAUNCHPAD_H_

//#include <time.h>
#include <stdint.h>
#include <stdbool.h>

//#include "inc/asmdefs.h"
//#include "inc/hw_adc.h"
//#include "inc/hw_can.h"
//#include "inc/hw_comp.h"
//#include "inc/hw_eeprom.h"
//#include "inc/hw_epi.h"
//#include "inc/hw_ethernet.h"
//#include "inc/hw_fan.h"
//#include "inc/hw_flash.h"
#include "inc/hw_gpio.h"
//#include "inc/hw_hibernate.h"
//#include "inc/hw_i2c.h"
//#include "inc/hw_i2s.h"
//#include "inc/hw_ints.h"
//#include "inc/hw_lpc.h"
#include "inc/hw_memmap.h"
//#include "inc/hw_nvic.h"
//#include "inc/hw_peci.h"
//#include "inc/hw_pwm.h"
//#include "inc/hw_qei.h"
//#include "inc/hw_ssi.h"
//#include "inc/hw_sysctl.h"
//#include "inc/hw_sysexc.h"
#include "inc/hw_timer.h"
#include "inc/hw_types.h"
//#include "inc/hw_uart.h"
//#include "inc/hw_udma.h"
//#include "inc/hw_usb.h"
//#include "inc/hw_watchdog.h"

/*
 * Include All Peripheral Driver Headers
 */
#include "driverlib/adc.h"
//#include "driverlib/can.h"
//#include "driverlib/comp.h"
//#include "driverlib/cpu.h"
//#include "driverlib/debug.h"
//#include "driverlib/eeprom.h"
//#include "driverlib/epi.h"
//#include "driverlib/ethernet.h"
//#include "driverlib/fan.h"
//#include "driverlib/flash.h"
//#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
//#include "driverlib/hibernate.h"
//#include "driverlib/i2c.h"
//#include "driverlib/i2s.h"
//#include "driverlib/interrupt.h"
//#include "driverlib/lpc.h"
//#include "driverlib/mpu.h"
//#include "driverlib/peci.h"
//#include "driverlib/pin_map.h"
//#include "driverlib/pwm.h"
//#include "driverlib/qei.h"
//#include "driverlib/rom_map.h"
//#include "driverlib/rom.h"
//#include "driverlib/rtos_bindings.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
//#include "driverlib/sysexc.h"
//#include "driverlib/systick.h"
#include "driverlib/timer.h"
//#include "driverlib/uart.h"
//#include "driverlib/udma.h"
//#include "driverlib/usb.h"
//#include "driverlib/watchdog.h"

/*
 * Include All Utility Drivers
 */
//#include "utils/cmdline.h"
//#include "utils/cpu_usage.h"
//#include "utils/crc.h"
//#include "utils/flash_pb.h"
//#include "utils/isqrt.h"
//#include "utils/ringbuf.h"
///#include "utils/scheduler.h"
//#include "utils/sine.h"
//#include "utils/softi2c.h"
//#include "utils/softssi.h"
//#include "utils/softuart.h"
//#include "utils/uartstdio.h"
//#include "utils/ustdlib.h"

/*
 * On board button (switch) definitions
 */
#define lpSWPort	GPIO_PORTF_BASE
#define lpSW1Port	GPIO_PORTF_BASE
#define lpSW2Port	GPIO_PORTF_BASE
#define lpSW1		GPIO_PIN_4
#define lpSW2		GPIO_PIN_0




#endif /* LAUNCHPAD_H_ */
