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
 * @brief Checks for all end conditions and shuts down the motor control if any are detected
 *
 * @param Void
 * @return Void
 */
extern void safetyCheck(void); //explicit function to test for end conditions



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
 * ADC functions
 */
extern uint32_t adcArray[8]; // This variable is defined globally

/**
 * @brief Init ADC.
 *
 * Initialises ADC with Sequence 0, capturing 8 samples and a FIFO depth of 8 32-bit words, with last 12 bits containing the conversion result
 *
 * @param Void
 * @return Void
 *
 */
extern void adcInit(void);

/**
 * @brief Sets the ADC MUX to read from either the current sense resistor 1, 2 or 3 from motor 1 or 2
 *
 *
 * @param Void
 * @return Void
 *
 */
extern void setADCMux(int number,int motor);

/**
 * @brief Reads ADC values and stores them into an array for current and tempertature
 *
 *
 *
 * @param Void
 * @return Void
 *
 */
extern void adcRead(void);
// Wrapper functions for reading current and temperature

/**
 * @brief Reads current from an array
 *
 *
 *
 * @param Void
 * @return uint32_t current
 *
 */
extern uint32_t currentRead1(void);

/**
 * @brief Reads current from an array
 *
 *
 *
 * @param Void
 * @return uint32_t current
 *
 */
extern uint32_t currentRead2(void);

/**
 * @brief Reads temperature from an array
 *
 *
 *
 * @param Void
 * @return uint32_t temperature
 *
 */
extern int32_t  tempRead1(void);

/**
 * @brief Reads temperature from an array
 *
 *
 *
 * @param Void
 * @return uint32_t temperature
 *
 */
extern int32_t  tempRead2(void);

extern void i2cInit(tI2CMInstance g_sI2CMSimpleInst);
//extern void i2cIntHandler(void);


#endif /* R2R_H_ */
