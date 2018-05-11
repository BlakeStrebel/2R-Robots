/**
 * @file r2r.h
 * @brief Main R2R library header
 *
 * This file contains all the prototypes for the R2R Project
 *
 * @author Benjamen Lim
 * @author Huan Weng
 *
 */

#ifndef R2R_H_
#define R2R_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/hw_mpu6050.h"
#include "sensorlib/mpu6050.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"


/*
 * General functions
 */

/**
 * @brief Initializes the default connections for the R2R project
 *
 * This function initializes all peripherals and GPIO pins.
 * This function only needs to be called in main.c once
 *
 * @param Void
 * @return Void
 */
extern void r2rDefaultInit(void);


/**
 * @brief Initializes the system clock and the master interrupts for the R2R project
 *
 *
 * @param Void
 * @return Void
 */
extern void sysInit(void);

/**
 * @brief Initializes the GPIO to control the motor's direction, enable and brake pins.
 *        It also calls pwmInit to initialize the PWM pins for 1x PWM mode
 * @param Void
 * @return Void
 */
extern void motorInit(void);
/**
 * @brief Initializes the SPI channels for the motor drivers, encoders, and RAM.
 *
 * Initializes SSI1 ~ 4 and their corresponding GPIO pins
 *
 * @param Void
 * @return Void
 */
extern void spiInit(void);

/**
 * @brief Initializes the SPI commands to set the motor's configuration for 1x PWM mode.
 * @param Void
 * @return Void
 */
extern void motorDriverInit(void);

/**
 * @brief Initializes PWM
 *
 * Initialize PWM on PF0 (motor 1) and PG0 (motor 2), and the default period if 320 SysClk
 * cycles.
 *
 * @param Void
 * @return Void
 */
extern void pwmInit(void);
extern void timerIntInit(void);
/**
 * @brief Initializes all unused GPIO pins in High-Z state
 *
 * @param Void
 * @return Void
 */
extern void gpioInit(void); // inits all unused GPIO

/**
 * @brief Checks for all end conditions and shuts down the motor control if any are detected
 *
 * @param Void
 * @return Void
 */
extern void safetyCheck(void); //explicit function to test for end conditions

/**
 * @brief Stops the processor for a given amount of time. This is an approximate time
 *
 * @param ms milliseconds
 * @return Void
 */
extern void delayMS(int ms); // stops processor for a given amount of time in ms, this is approximate.

/**
 * @brief Updates all sensor arrays.
 *
 * Sensors are grouped for the ease of use into one function and the array is updated each time this function is called.
 *
 * @param Void
 * @return Void
 *
 */
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


/**
 * @brief Sets the PWM value to motor 1
 *
 * @param pwmValue The pwmValue ranges from 0 to the pwm period. The default pwm period is 320 SysClk cycles
 * @return Void
 *
 */
extern void motor1PWM(int pwmValue);
/**
 * @brief Sets the PWM value to motor 1
 *
 * @param pwmValue The pwmValue ranges from 0 to the pwm period. The default pwm period is 320 SysClk cycles
 * @return Void
 *
 */
extern void motor2PWM(int pwmValue);
/**
 * @brief Sets the PWM value and direction to motor 1
 *
 * @param control The pwmValue ranges from -maximum pwm period to the positive period. The default pwm period is 320 SysClk cycles
 * @return Void
 *
 */
extern void motor1ControlPWM(int control);
/**
 * @brief Sets the PWM value and direction to motor 2
 *
 * @param control The pwmValue ranges from -maximum pwm period to the positive period. The default pwm period is 320 SysClk cycles
 * @return Void
 *
 */
extern void motor2ControlPWM(int control);


extern int motorError(void); // returns 50 on no error, 51 on motor 1 error, 52 on motor 2 error'

/**
 * @brief Updates value in sensor array
 *
 * This function sends a 't' and 40 pulses via SPI to the encoder and reads out the encoder's reply to
 * the encoderVal array
 *
 * @param Void
 * @return Void
 *
 */
extern void encoderRead(void);

/**
 * @brief Reads value in encoderVal array for motor 1 and converts it to an angle
 *
 * @param Void
 * @return The angle as a float
 *
 */
extern float readMotor1Angle(void);

/**
 * @brief Reads value in encoderVal array for motor 2 and converts it to an angle
 *
 * @param Void
 * @return The angle as a float
 *
 */
extern float readMotor2Angle(void);

/**
 * @brief Reads value in encoderVal array for motor 1
 *
 * The range for the read angle is 0 to 16383
 *
 * @param Void
 * @return The angle as an int
 *
 */
extern int readMotor1Raw(void);

/**
 * @brief Reads value in encoderVal array for motor 2
 *
 * The range for the read angle is 0 to 16383
 *
 * @param Void
 * @return The angle as an int
 *
 */
extern int readMotor2Raw(void);

/**
 * @brief Reads value in encoderVal array for motor 1 and keeps track of multiturn counts
 *
 * This function reads out the angle and keeps track of it for multiple turns. The value
 * ranges from -2,147,483,648 to 2,147,483,648. Each turn is 16383 counts
 *
 * @param Void
 * @return The counts as an int
 *
 */
extern uint32_t readMotor1RawRelative(void);

/**
 * @brief Reads value in encoderVal array for motor 2 and keeps track of multiturn counts
 *
 * This function reads out the angle and keeps track of it for multiple turns. The value
 * ranges from -2,147,483,648 to 2,147,483,648
 *
 * @param Void
 * @return The counts as an int
 *
 */
extern uint32_t readMotor2RawRelative(void);

/**
 * @brief Reads value in encoderVal array for motor 1 and keeps track of multiturn angles
 *
 * This function reads out the angle and keeps track of it for multiple turns.
 *
 * @param Void
 * @return The angle as an int
 *
 */
extern float readMotor1AngleRelative(void);

/**
 * @brief Reads value in encoderVal array for motor 2 and keeps track of multiturn angles
 *
 * This function reads out the angle and keeps track of it for multiple turns.
 *
 * @param Void
 * @return The angle as an int
 *
 */
extern float readMotor2AngleRelative(void);

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
