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
#include "utils/uartstdio.c"
#include "r2r.h"


volatile uint8_t user_input_flag = 0;
volatile uint8_t get_user_char ='m';
volatile uint8_t more_input = 0;

volatile float Kptest = 0;
volatile float Kdtest = 0;
volatile float Kitest = 0;

char charBuf[100]={}; //global char array to store variables from the MATLAB user


// Sensor test code
// Reads 2 encoders
// This is a SSI sensor




int main()
{


    // Init code
    r2rDefaultInit();


    // Enable interrupts of UART
    //IntEnable(INT_UART0);
    //UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);









    // UARTprintf("Setting up...");





    return(0);
}

