/*
 * This example demonstrates the use of the sensorUpdate() function
 *
 * r2rdebug is SET TO 1
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
    r2rDefaultInit();

    // Interrupts

    while(1){
        sensorUpdate();
        UARTprintf("Motor 1 Angle: %d\t Motor 2 Angle: %d\t Motor 2 Relative Angle: %ld\n",
                   readMotor1Angle(),readMotor2Angle(),(long)readMotor2RelativeAngle());

        }



    







    return(0);
}

