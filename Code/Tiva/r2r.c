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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/fpu.h"
#include "driverlib/ssi.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/debug.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"


#include "r2r.h"
#include "System.h"
#include "Encoder.h"
#include "Motor.h"
#include "PositionControl.h"
#include "CurrentControl.h"
#include "Utilities.h"


// TODO: motor driver 2 cs pin is actually motor driver 1's CS pin

/*
 * This function does initialisation for all the default connections
 *
 * Comes after:
 * -
 */
void r2rDefaultInit(void)
{
    sysInit(); // Init system clock and Master interrupt
    uartInit();
    encoderSPIInit();

    MotorSPIinit();
    motorInit(); // Set useful signal outputs.
    motorDriverInit(); // Send values to set up the motor for 3x PWM mode
    currentControlInit();


    MotorTimerInit();
    timeInit();

    SysCtlDelay(ui32SysClock); // Wait for a second

    setMODE(ICALIB);    // Calibrate current offsets
}

/*
 * This function explicitly checks for safety conditions, and it uses a global variable run_program to turn off the motor
 * TODO: save last state to a permanent storage device and check it upon starting. If last state was some error state, user must clear the flag
 */
void safetyCheck(void){
    ;
}

/*
 * This function updates all the sensors that we are using. Usually runs once every loop.
 * TODO: Add sensor functions, add interrupts
 */
void sensorUpdate(void){
    //adcRead();
    encoderRead(1);
    encoderRead(2);
}

/*
 * This function sets up an I2C bus on I2C0, PB2 and PB3
 * Sets 400kbps
 */

void i2cInit(tI2CMInstance g_sI2CMSimpleInst){
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.
    // I2C data transfer rate set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);

    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;

    // Initialize the I2C master driver.
    I2CMInit(&g_sI2CMSimpleInst, I2C0_BASE, INT_I2C0, 0xff, 0xff, SysCtlClockGet());
}

void MCP9600init(){
    // See https://ncd.io/k-type-thermocouple-mcp9600-with-arduino/
    // Operations:
    // Set thermo config
    // write 0x05
    // write 0x00
    // Set device config
    // write 0x06
    // write 0x00
    //I2CMwrite(i2cints,0x64,0x00,1,data,count,callback,write)
}

void MCP9600ready(){
    // Write 0x04
    // wait
    // check if slave busy, if not read
    // return int
}

void MCP9600read(){
    // write 0x00
    // read to long int = 1
    // read to long int = 2
    // if 1 &0x80 == 0x80
    // 1 = 1& 0x7F
    //temp  = 1024 - (1 *16/ + 2/16)
}
/*********************************************** CURRENTLY NOT BEING IMPLEMENTED ********************************************/





/*
 * Reads the temperature
 */
uint32_t tempRead(void){
    uint32_t pui32ADC0Value[4];
    // uint32_t ui32TempValueC;
    //
    // Trigger the ADC conversion.
    //
    ADCProcessorTrigger(ADC0_BASE, 3);

    //
    // Wait for conversion to be completed.
    //
    while(!ADCIntStatus(ADC0_BASE, 3, false))
    {
    }
    //
    // Clear the ADC interrupt flag.
    //
    ADCIntClear(ADC0_BASE, 3);

    //
    // Read ADC Value.
    //
    ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value);
    //
    // Use non-calibrated conversion provided in the data sheet.  Make
    // sure you divide last to avoid dropout.
    //
    // Read internal processor temperature
    //ui32TempValueC = ((1475 * 1023) - (2250 * pui32ADC0Value[0])) / 10230;

    return 1;
}






