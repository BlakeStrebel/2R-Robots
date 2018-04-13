/*
* This code demonstrates how to use the motor control code with the R2R lib.
* 
* motor2controlPWM() takes a positive or negative value from the range of -320 to 320. 
* Braking is enabled when control signal is 0.
*/



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



int main()
{



    // Init code
    r2rDefaultInit(); // PWM set to a period of 320 clock cycles
    UARTprintf("Init complete.\n");

    // Interrupts

    // Start code
    UARTprintf("Turning forwards.\n");
    int numcount = 0;
    while(numcount<1000000){
        motor2ControlPWM(50); // forward
        numcount++;
    }
    UARTprintf("Turning backwards.\n")
    numcount = 0;
    while (numcount<1000000){
        motor2ControlPWM(-50); // reverse
        numcount++;
    }
    while(1){
         motor2ControlPWM(0); // stop
    }




    // Infinite loop







    return(0);
}

