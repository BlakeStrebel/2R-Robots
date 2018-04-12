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
// Variables
float eint = 0;
float eprev = 0;
// PID control loop
float kp = 0.01;
float kd = 0.1;
float ki = 0.1;

// Interrupts
// Update the STARTUP_CCS.C file when adding new interrupts!
/*
void
Timer0IntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Toggle the flag for the first timer.
    //
    //ledFlag ^= 1;
    if (ledFlag==1){
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0,0);
        ledFlag=0;
    }
    else {
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0,  GPIO_PIN_0);
        ledFlag=1;
    }


    //
    // Use the flags to Toggle the LED for this timer
    //


}
*/

int i;
float data[2000];

void
Timer1IntHandler(void)                          //This is the time interrupt
{

    //
    // Clear the timer interrupt.
    //
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    //Test
    //
    // Toggle the flag for the second timer.
    ledFlag ^= 1;
    //

    //
    // Use the flags to Toggle the LED for this timer
    //
    //GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_0, ledFlag);

    //ROM_UARTCharPutNonBlocking(UART0_BASE,
     //                          'c');


    //
    // Update the interrupt status.
    //
    IntMasterDisable();                                     //Should be at the top of this function?

    sensorUpdate(); // updates encoders



    float desired_angle = 60;
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
    //UARTprintf("Motor 2: %d \t Motor 2 angle: %d \t Motor 2 control: %d\n",readMotor2Raw(),(int)readMotor2Angle(),(int)control);
    motor2ControlPWM((int)control);
    eprev=error;

    data[i++] = current_angle;
    if (i < 500)
        IntMasterEnable();

}


int main()
{
    char traj[2000];
    char recev[12];
    uint8_t *array;
    float *getfloat;
    uint8_t x;


    // Init code
    r2rDefaultInit();
    //UARTprintf("Init complete.");

    // Interrupts
<<<<<<< HEAD
    int numcount = 0;
    while(numcount<1000000){
        motor2ControlPWM(50);
        numcount++;
    }
    while(1){
        motor2PWM(1);
    }

=======

   for (i = 0; i < 2000; i++){
       recev[i] = 0;
   }
>>>>>>> 6190cf45c1e3f5f3642c9de660cfffedaf1b0917
    // Infinite loop

   i = 0;
    while(1){
        while(UARTCharsAvail(UART0_BASE)){
            for (i = 0; i < 4 * 3; i++) {
                recev[i] = UARTCharGet(UART0_BASE);
            }
            getfloat = &recev;
            kp = *getfloat++;
            ki = *getfloat++;
            kd = *getfloat;
            GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_0, 1);
            UARTCharPut(UART0_BASE,'p');
            while(!UARTCharsAvail(UART0_BASE)){}
            //GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_0, 1);
            if (UARTCharGet(UART0_BASE) == 's'){
               GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_0, 0);
               IntMasterEnable();
               i = 0;
            }

        }
        if (i == 500){
            GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_0, 1);
            array = &data;
            for (i = 0; i < 500 * 4; i++){
                ROM_UARTCharPut(UART0_BASE, *array++);
            }
        }





    }







    return(0);
}

