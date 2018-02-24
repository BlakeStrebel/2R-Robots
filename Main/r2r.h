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

/*
 * General functions
 */
extern void sysInit(void);
extern void pwmInit(void);

// Always call sensor update at beginning of loop!
extern void sensorUpdate(void);


/*
 * ADC functions
 */
extern uint32_t adcArray[4]; // This variable is defined globally

extern void adcInit(void);
extern void adcRead(void);
// Wrapper functions for reading current and temperature
extern uint32_t currentRead1(void);
extern uint32_t currentRead2(void);
extern uint32_t  tempRead1(void);
extern uint32_t  tempRead2(void);





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


extern void spiInit(void);
/*
 * Other useful functions for SSI
 * SSIDataGetNonBlocking(SSI0_Base, &pui32DataRx)
 * SSIDataPut()
 * SSIBusy()
 * SSIDataGet()
 */


/*
 *
 *
 *
 */

#endif /* R2R_H_ */
