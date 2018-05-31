/**
 * @file System.h
 * @brief system header
 *
 * This file contains the uart, sysclk and general gpio functions
 *
 * @author Benjamen Lim
 * @author Huan Weng
 * @author Blake Strebel
 *
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_

#include "includes.h"

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
 * @brief Initializes all unused GPIO pins in High-Z state
 *
 * @param Void
 * @return Void
 */
extern void gpioInit(void); // inits all unused GPIO

/**
 * @brief Stops the processor for a given amount of time. This is an approximate time
 *
 * @param ms milliseconds
 * @return Void
 */
extern void delayMS(int ms); // stops processor for a given amount of time in ms, this is approximate.


extern void timeInit(void);
extern uint32_t getTime(void);
extern void timeInt(void);

#endif /* SYSTEM_H_ */
