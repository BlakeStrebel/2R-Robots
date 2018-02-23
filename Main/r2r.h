/*
 * r2r.h
 *
 *  Created on: Feb 21, 2018
 *      Author: Ben
 *
 *      This header file combines initialization and functions for the R2R firmware
 */

#ifndef R2R_H_
#define R2R_H_

// These refer to the LEDs on the Tiva Launchpad
#define RED_LED   GPIO_PIN_1
#define BLUE_LED  GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3

extern void sysInit(void);
extern void pwmInit(void);


extern void uartInit(void);
//extern void uartIntHandler(void);
/*
 * Other external useful functions for UART
 * UARTprintf()
 * UARTgets()
 * UARTwrite()
 * UARTsend()
 */

extern void i2cInit(tI2CMInstance g_sI2CMSimpleInst);
//extern void i2cIntHandler(void);


/*
 *
 * extern void spiInit(void);
 *
 */

#endif /* R2R_H_ */
