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
#include "driverlib/ssi.h"
#include "driverlib/systick.h"
#include "utils/uartstdio.h"
#include "driverlib/fpu.h"
#include "math.h"
#include "driverlib/adc.h"

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

extern void UART0IntPut(int value);

extern void UART0FloatPut(float value);

extern void UART0ArrayPut(int number, float * value);

extern float UART0FloatGet();

extern int UART0IntGet();



#endif /* R2R_H_ */
