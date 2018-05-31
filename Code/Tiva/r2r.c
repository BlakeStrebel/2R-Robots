/**
 * @file r2r.c
 * @brief
 *
 * This file contains the main function that calls all other initialization functions
 *
 * @author Benjamen Lim
 * @author Huan Weng
 *
 */


#include "r2r.h"


void r2rDefaultInit(void){
    sysInit(); // initialize System Clock and master interrupt
    uartInit(); // initialize UART
    encoderSPIInit(); // initialize SPI for encoder
    MotorSPIInit(); // initialize SPI for motor
    pwmInit();
    motorInit(); // Set pins for motor driver
    motorDriverInit(); // Send values to set up the motor for 1x PWM mode
    adcInit();
    //currentControlInit();

    //timeInit();

    //gpioInit(); // Init for general GPIO - set to input for safety
    //MotorTimerInit();

}









