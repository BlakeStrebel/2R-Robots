/**
 * @file r2r.c
 * @brief Main R2R library
 *
 * This file contains all the functions for the R2R Project
 *
 * @author Benjamen Lim
 * @author Huan Weng
 *
 */

#include "r2r.h"
#include "Encoder.h"
#include "Motor.h"
#include "PositionControl.h"
#include "CurrentControl.h"
#include "Utilities.h"


#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
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
#include "driverlib/fpu.h"

/*
 * This function does initialization for all the default connections
 *
 * Comes after:
 * -
 */
void r2rDefaultInit(void){
    sysInit(); // Init system clock and Master interrupt
    uartInit();
    encoderSPIInit();
    MotorSPIInit();
    motorInit(); // Set useful signal outputs.
    motorDriverInit(); // Send values to set up the motor for 3x PWM mode
    currentControlInit();
    positionControlInit();
    //timeInit();
    SysCtlDelay(ui32SysClock); // Wait for a second
    setMODE(ICALIB);    // Calibrate current offsets
}


/*
 * The system init function initiazes the system clock and the processor interrupts
 * Clock: 120Mhz
 *
 * Comes after:
 * -
 */
void sysInit(void){
    // Set the clock to run directly from the crystal at 120MHz.
    ui32SysClock = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                      SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480,
                                      120000000); // Use 25Mhz crystal and use PLL to accelerate to 120MHz
    // Enable FPU for calculation.
    FPUEnable();
}

/*
 * This function sets up the UART on UART0, PA0(RX) and PA1 (TX)
 * It is set up for 115200 baud 8N1. To send numbers and characters use
 * UARTprintf.
 */
void uartInit(void)
{
    // Enable GPIO port A which is used for UART0 pins.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configure UART0 pins on port A0 and A1.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    // Enable UART0 so that we can configure the clock.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Select the alternate (UART) function for these pins.
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    // Configure the UART for 115,200, 8-N-1 operation.
    // TODO: Change the baud rate to the highest. 256000? 128000? as Matlab mentions?
    UARTConfigSetExpClk(UART0_BASE, ui32SysClock, 115200,
                        UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                        UART_CONFIG_PAR_NONE);
}


void UART0IntPut(int value)
{
    uint8_t *byte;
    byte = &value;
    int i;
    for (i = 0; i < 4; i++)
        UARTCharPut(UART0_BASE, *byte++);
}

void UART0FloatPut(float value)
{
    uint8_t *byte;
    byte = &value;
    int i;
    for (i = 0; i < 4; i++)
        UARTCharPut(UART0_BASE, *byte++);
}

void UART0ArrayPut(int number, float * value)
{
    uint8_t *byte;
    byte = &value;
    int i;
    for (i = 0; i < number * 4; i++)
        UARTCharPut(UART0_BASE, *byte++);
}

float UART0FloatGet()
{
    char recev[4];
    float *getfloat;
    int i;
    for (i = 0; i < 4; i++)
        recev[i] = UARTCharGet(UART0_BASE);
    getfloat = &recev;
    return *getfloat;
}

int UART0IntGet()
{
    char recev[4];
    int *getint;
    int i;
    for (i = 0; i < 4; i++)
        recev[i] = UARTCharGet(UART0_BASE);
    getint = &recev;
    return *getint;
}







