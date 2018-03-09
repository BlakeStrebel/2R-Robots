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
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "r2r.h"


volatile uint8_t user_input_flag = 0;
volatile uint8_t get_user_char ='m';
volatile uint8_t more_input = 0;

uint8_t charBuf[100]={}; //global char array to store variables from the MATLAB user


// Sensor test code
// Reads 2 encoders
// This is a SSI sensor
void EncodersExampleTest (void){
    ;
}


void
UARTIntHandler(void)
{
    user_input_flag = 1;

    uint32_t ui32Status;

    ui32Status = UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    // We need to clear the flag otherwise it will not interrupt again


    //
    // Loop while there are characters in the receive FIFO.
    //
    int count = 0;
    while(UARTCharsAvail(UART0_BASE))
    {
        if (more_input == 0){
            // We are in command mode
            get_user_char = UARTCharGetNonBlocking(UART0_BASE);
        }
        else {
            // We are getting some input from user.
            while (count<4){
                char newchar = UARTCharGetNonBlocking(UART0_BASE);
                charBuf[count] = newchar;
                count++;
            }
        }
        /* while (count<4){
            char newchar = UARTCharGetNonBlocking(UART0_BASE);
            charBuf[count] = newchar;
            count++;
        }
        */
    }
    more_input = 0;
    UARTIntClear(UART0_BASE, ui32Status);

    /*
    float x = *(float *)&charBuf;
    x = x*2;

    recv = *(float *)&charBuf; // convert a char array to a float
    recv = recv+2000;
    valRecv = &recv; // do some operation on the float
    UARTSend(valRecv, 4); // send it back.
*/
}


int main()
    {
    user_input_flag = 1;
    //uint32_t myarray[4];
    int run_program =1 ; // flag to check if code should be running
    int arm_in_motion = 0; //flag to check if we are moving the arm
    uint32_t optionArray[4];

    // Init code
    sysInit();
    //uartInit();
    initConsole();
    // Enable interrupts of UART
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    adcInit();
    //menuInit();
    //spiInit();
    //motorInit();
    //i2cInit(g_sI2CMSimpleInst);



    // UARTprintf("Setting up...");

    int curr1;
    int curr2;
    int temp1;
    int temp2;
    sensorUpdate();

    while(run_program == 1){

        // Set global variable arm_in_motion when the arm is in motion to 1
        // If user ends the program, set run_program==0
        if (user_input_flag==1){
            //menuOptions(get_user_char);
            matlabMenu(get_user_char);
            user_input_flag = 1; // unset user input flag once all menu processing is complete
        }

        // Loop to run while the arm is in motion
        // When trajectory is complete, set arm_in_motion to 0
        while(arm_in_motion==1){

            sensorUpdate();
            //PIDUpdate();
            curr1 = currentRead1();
            curr2 = currentRead2();
            temp1 = tempRead1();
            temp2 = tempRead2();
            UARTprintf("The sensor readings for current are: %d %d temp: %d %d\r\n",curr1,curr2,temp1,temp2);

            UARTprintf("Printout");
            //delayMS(10);

            //safetyCheck();
        }

    }
    //shutdownAll();



    return(0);
}
