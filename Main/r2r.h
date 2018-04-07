/*
 * r2r.h
 *
 *  Created on: Feb 21, 2018
 *  Updated on: March 16, 2018
 *      Author: Ben
 *
 *      This header file combines initialization and functions for the R2R firmware
 */

#ifndef R2R_H_
#define R2R_H_

/*
 * General functions
 */
extern void r2rDefaultInit(void);
extern void sysInit(void);
extern void pwmInit(void);
extern void gpioInit(void); // inits all unused GPIO
extern void safetyCheck(void); //explicit function to test for end conditions
extern void delayMS(int ms); // stops processor for a given amount of time in ms, this is approximate
extern void sensorUpdate(void); // Updates sensor data. Always call sensor update at beginning of loop!

/*
 * Menu functions
 * This code writes menu options to the matlab script
 */
extern void matlabMenu(char menuchar);
extern void menuHeader(uint8_t input);
extern void menuInit(void);
extern void menuOptions(uint8_t input);
extern void processUserInput(void);


/*
 * Motor functions
 * This section contains all the functions written to get the motor to work.
 * TODO: write function to set various PWM values
 */
extern void motorInit(void);
extern void motorPWM1(int pwmValue); //sets the motor PWM values
extern void motorPWM2(int pwmValue);
extern void motor2ControlPWM(int control);
extern int motorError(void); // returns 50 on no error, 51 on motor 1 error, 52 on motor 2 error
extern void encoderRead(void);

/*
 * Control functions
 */
extern void PIDUpdate(void);
extern void PIDCurrUpdate(void);


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
extern void uartSend(const uint8_t *pui8Buffer, uint32_t ui32Count);
extern void initConsole(void);
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
