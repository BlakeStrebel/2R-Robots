/**
 * @file r2r.c
 * @brief
 *
 * This file contains the default initialisation function
 *
 * @author Benjamen Lim
 * @author Huan Weng
 *
 */

#include "includes.h"
#include "r2r.h"
#include "System.h"
#include "Encoder.h"
#include "Motor.h"
#include "PositionControl.h"
#include "CurrentControl.h"
#include "Adc.h"

void r2rDefaultInit(void){
    sysInit(); // initialize System Clock and master interrupt
    uartInit(); // initialize UART
    encoderSPIInit(); // initialize SPI for encoder
    MotorSPIinit(); // initialize SPI for motor
    pwmInit();
    motorInit(); // Set pins for motor driver
    motorDriverInit(); // Send values to set up the motor for 1x PWM mode
    //currentControlInit();
    adcInit();
    timeInit();

    //gpioInit(); // Init for general GPIO - set to input for safety
    //MotorTimerInit();

}









