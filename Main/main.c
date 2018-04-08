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
#include "utils/uartstdio.h"
#include "utils/uartstdio.c"
#include "r2r.h"

// Variables
uint32_t ledFlag = 1;

// Interrupts
// Update the STARTUP_CCS.C file when adding new interrupts!

void
Timer0IntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    TimerIntClear(TIMER0_BASE, TIMER_TIMB_TIMEOUT);

    //
    // Toggle the flag for the first timer.
    //
    ledFlag ^= 1;

    //
    // Use the flags to Toggle the LED for this timer
    //
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, ledFlag);

}




int main()
{


    // Init code
    r2rDefaultInit();
    UARTprintf("Init complete.");

    // Interrupts


    // Infinite loop

    // Variables
    float eint = 0;
    float eprev = 0;


    while(1){
        sensorUpdate(); // updates encoders

        // PID control loop
        float kp = 0.01;
        float kd = 0.1;
        float ki = 0.1;

        float desired_angle = 180;
        float current_angle = readMotor2Angle();
        float error = desired_angle - current_angle;

        //UARTprintf("Desired_angle: %d \t Current Angle: %d\n",desired_angle,(int)current_angle);

        eint = eint+error;
        // Integration clamp
        if(eint>300){
            eint=300;
        }
        else if(eint<-300){
            eint=-300;
        }
        float control = kp*error+kd*eprev+ki*eint;
        // Control signal clamp
        if (control>30){
            control = 30;
        }
        else if (control<-30){
            control = -30;
        }
        UARTprintf("Motor 2: %d \t Motor 2 angle: %d \t Motor 2 control: %d\n",readMotor2Raw(),(int)readMotor2Angle(),(int)control);
        motor2ControlPWM((int)control);
        eprev=error;



    }







    return(0);
}

