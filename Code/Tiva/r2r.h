/**
 * @file r2r.h
 * @brief Main R2R library header
 *
 * This file contains system functions for the R2R Project, including functions related to system initializations, UART, and debugging.
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
#include "driverlib/ssi.h"
#include "driverlib/systick.h"
#include "driverlib/adc.h"
#include "utils/uartstdio.h"


/*
 * General functions
 */

/**
 * @brief Initializes the default connections for the R2R project
 *
 * This function initializes all peripherals and GPIO pins.
 * This function only needs to be called in main once.
 *
 * Example:
 *
 *     int main() {
 *			r2rDefaultInit();
 *     }
 *
 * @param Void
 * @return Void
 */
extern void r2rDefaultInit(void);


#define MOTOR1 1
#define MOTOR2 2

uint32_t ui32SysClock;
uint32_t micros;

//int error_state = 0;

/**
 * @brief Initializes the system clock and the master interrupts for the R2R project
 *
 *
 * @param Void
 * @return Void
 */
extern void sysInit(void);

/**
 * @brief Initializes the UART for 115200 Baud Rate and 8-N-1 transfer rate.
 *
 *
 * @param Void
 * @return Void
 */
extern void uartInit(void);

/**
 * @brief Reads input from UART
 *
 *
 *
 * @param char* message pointer to a buffer
 * @param int maxLength the maximum length of the buffer
 * @return Void
 */
extern void UART0read(char * message, int maxLength);


/**
 * @brief Writes output from UART
 *
 *
 *
 * @param const char* message pointer to a buffer
 * @return Void
 */
extern void UART0write(const char * string);

/**
 * @brief Initializes custom timers
 *
 * This function initializes timers 6 and 7 at preset frequencies as defined in r2r.c
 *
 *
 * @param Void
 * @return Void
 */
extern void customTimersInit(void);

/**
 * @brief Custom timer 6
 *
 * This function executes at the rate set by TIMER_6_FREQ set in r2r.c
 * This function does not have to be called anywhere. This function does not return any values
 *
 * Example:
 *
 *		 void TIMER6IntHandler() {
 *	
 *			// add custom controller code here
 *
 * 		 }
 *
 *
 *
 * @param Void
 * @return Void
 */

extern void TIMER6IntHandler(void);

/**
 * @brief Custom timer 7
 *
 * This function executes at the rate set by TIMER_6_FREQ set in r2r.c
 * This function does not have to be called anywhere. 
 *
 * @param Void
 * @return Void
 */
extern void TIMER7IntHandler(void);

/**
 * @brief Stops the processor for a given amount of time. This is an approximate time
 *
 * @param ms milliseconds
 * @return Void
 */
extern void delayMS(int ms); // stops processor for a given amount of time in ms, this is approximate.

/**
 * @brief Initializes the general purpose systick timer
 *
 *
 * @param Void
 * @return Void
 */
extern void timeInit(void);

/**
 * @brief Returns the time in microseconds
 *
 * This function can be used for delays as an alternative to delayMS() because it is interrupt-based.
 *
 * Example:
 *
 * 		uint32_t curr_time = getTime();
 *		...
 *		uint32_t time_taken = getTime()-curr_time;
 *
 * @param us microseconds
 * @return Void
 */
extern uint32_t getTime(void);

/**
 * @brief The general purpose timer interrupt function.
 *
 * This function is called every microsecond and updates a counter.
 *
 * @param ms milliseconds
 * @return Void
 */
extern void timeInt(void);

#endif /* R2R_H_ */
