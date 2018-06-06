/**
 * @file r2r.c
 * @brief
 *
 * This file contains the main function that calls all other initialization functions
 *
 * @author Benjamen Lim
 * @author Huan Weng
 * @author Blake Strebel
 *
 */


#include "r2r.h"
#include "System.h"
#include "Encoder.h"
#include "Motor.h"
#include "PositionControl.h"
#include "CurrentControl.h"
#include "Utilities.h"


#define TIMER_6_FREQUENCY 10000
#define TIMER_7_FREQUENCY 1000



/*
 * This function does initialisation for all the default connections
 *
 * Comes after:
 * -
 */
void r2rDefaultInit(void)
{
    sysInit(); // initialize System Clock and master interrupt
    uartInit(); // initialize UART
    encoderSPIInit(); // initialize SPI for encoder
    //adcInit();
    motorInit(); // Set useful signal outputs.
    MotorSPIInit();
    motorDriverInit(); // Set up the motor for 3x PWM mode
    currentControlInit(); // Set up interrupts for current control 
    MotorTimerInit(); // Set up interrupts for motor control loop at 1kHz
    timeInit(); // general purpose tick tock timer, good for testing how long code takes.

    SysCtlDelay(ui32SysClock); // Wait for a second

    setMODE(ICALIB);    // Calibrate current offsets
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



void TIMER6IntHandler(void){
    ; // Add code here
}

void TIMER7IntHandler(void){
    ;
}


// This function sets up custom timers 6 and 7 for the user to use
void customTimersInit(){

     IntMasterDisable();

    // Enable the Timer0 peripheral.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER6); // Use timer 6
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER7); // Use timer 7

    // Configure Timer6A and Timer7A
    TimerConfigure(TIMER6_BASE, TIMER_CFG_PERIODIC);
    TimerConfigure(TIMER7_BASE, TIMER_CFG_PERIODIC);

    // Set the count time for timers
    TimerLoadSet(TIMER6_BASE, TIMER_A, ui32SysClock / TIMER_6_FREQUENCY-1); // Use timer 6A 1KHz (default)
    TimerLoadSet(TIMER7_BASE, TIMER_A, ui32SysClock / TIMER_7_FREQUENCY-1); // Use timer 7A 10KHz (default)

    // Setup the interrupts for the timer timeouts.
    IntEnable(INT_TIMER6A);
    IntEnable(INT_TIMER7A);
    TimerIntEnable(TIMER6_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER7_BASE, TIMER_TIMA_TIMEOUT);

    // Enable the timers.
    TimerEnable(TIMER6_BASE, TIMER_A);
    TimerEnable(TIMER7_BASE, TIMER_A);

  
    // Set the INT_TIMER6A interrupt priority to the 3rd lowest priority.
    IntPrioritySet(INT_TIMER6A,  0xB0);
    // Set the INT_TIMER7A interrupt priority to the 2nd lowest priority.
    IntPrioritySet(INT_TIMER7A,  0xC0);
    IntMasterEnable();

}










