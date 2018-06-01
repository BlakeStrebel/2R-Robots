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
<<<<<<< HEAD
    sysInit(); // initialize System Clock and master interrupt
    uartInit(); // initialize UART
    encoderSPIInit(); // initialize SPI for encoder
    motorInit(); // Set pins for motor driver
    motorDriverInit(); // Send values to set up the motor for 1x PWM mode
    adcInit();
    //currentControlInit();


    motorInit(); // Set useful signal outputs.
    motorDriverInit(); // Send values to set up the motor for 3x PWM mode
    //currentControlInit();

    //gpioInit(); // Init for general GPIO - set to input for safety
    //MotorTimerInit();

    timeInit();

    //gpioInit(); // Init for general GPIO - set to input for safety
    //MotorTimerInit();

}









