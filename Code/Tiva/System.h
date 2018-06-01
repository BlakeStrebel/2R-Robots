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
 * Combine this function with a char buffer to process inputs from UART
 *
 * Example:
 *
 *		int max_length = 50;
 *		float kp, ki, kd;
 *		char buffer[max_length];
 *		UART0read(buffer,max_length);
 *		sscanf(buffer, "%f %f %f", kp, ki, kd); // get P, I, D values from user
 * 
 *
 * @param char* message pointer to a buffer
 * @param int maxLength the maximum length of the buffer
 * @return Void
 */
extern void UART0read(char * message, int maxLength);


/**
 * @brief Writes output to UART
 *
 * This function will write until it reaches the end of the char buffer '\0'. 
 * 
 * Example:
 *
 *		int max_length = 50;
 *		char buffer[max_length];
 *		sprintf(buffer,"kp: %f, ki: %f, kd: %f",kp, ki, kd);
 *		UART0write(buffer,max_length);
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

#endif /* SYSTEM_H_ */
