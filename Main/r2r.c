/*
 * r2r.c
 *
 *  Created on: Feb 21, 2018
 *      Author: Ben
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
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
#include "driverlib/fpu.h"
#include "driverlib/adc.h"
#include "driverlib/ssi.h"
#include "driverlib/pwm.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/debug.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h"



#include "r2r.h"

uint32_t adcArray[4]={0};


uint32_t ui32SysClock;

#define NUM_SSI_DATA            8
#define NUM_SSI_DATA2           5

// Data from motor driver 2
uint32_t pui32DataTx[NUM_SSI_DATA];
uint32_t pui32DataRx[NUM_SSI_DATA];
uint32_t ui32Index;

// TODO: motor driver 2 cs pin is actually motor driver 1's CS pin

// Data from encoder 2
uint32_t pui32DataTx2[NUM_SSI_DATA2];
uint32_t pui32DataRx2[NUM_SSI_DATA2];
uint32_t ui32Index2;

// Array to store encoder data
uint32_t encoderVal[2];



/*
 * This function does initialisation for all the default connections
 *
 * Comes after:
 * -
 */
void r2rDefaultInit(void){
    sysInit(); // Init system clock and Master interrupt
    uartInit(); // Init UART communication
    spiInit(); // Init SPI communication for motor driver 1 and 2, and encoder 1, and encoder 2
    motorInit(); // Set up PWM pins for motor driver, already includes pwmInit()
    motorDriverInit(); // Send values to set up the motor for 1x PWM mode
    //pwmInit();
    adcInit(); // Init for current and temperture
    gpioInit(); // Init for general GPIO - set to input for safety
    timerIntInit();

}

/*
 * This function stops the clock for a given amount of ms
 *
 * Comes after:
 * sysInit()
 */
void delayMS(int ms) {
    //SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ;
    SysCtlDelay( (ui32SysClock/(3*1000))*ms ) ;

}


/*
 * The system init function initiazes the system clock and the processor interrupts
 * Clock: 120Mhz
 *
 * Comes after:
 * -
 */
void sysInit(void){
    //
    // Set the clock to run directly from the crystal at 120MHz.
    //
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480), 120000000); // Use 25Mhz crystal and use PLL to accelerate to 120MHz
    IntMasterEnable();
}

/*
 * This function explicitly checks for safety conditions, and it uses a global variable run_program to turn off the motor
 * TODO: save last state to a permanent storage device and check it upon starting. If last state was some error state, user must clear the flag
 */
void safetyCheck(void){
    ;
}

/*
 * This function updates all the sensors that we are using. Usually runs once every loop.
 * TODO: Add sensor functions, add interrupts
 */
void sensorUpdate(void){
    //adcRead();
    encoderRead();
}



/*
 * This function sets up the timer interrupt
 *
 * Comes after:
 * - sysInit()

void timerIntInit(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0); // Use timer 0
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_B, ui32SysClock/4); // Use timer B // activate every 1/2 of a second 120/120/2 = 0.5s
    IntEnable(INT_TIMER0B);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMB_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_B);
    // TODO: GPIO is just for blinking purposes on the EK-TM4C129
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0); // LED PN0
    GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_0,GPIO_PIN_0); // Turn on the damn thing
}
 */
void timerIntInit(void){               // Set the clocking to run directly from the crystal at 120MHz.  So we need to decide the sysclock, maybe the largest
    //
    // Enable processor interrupts.
    //
    IntMasterEnable();

    //
    // Enable the peripherals used by this example.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);              //Not sure if we can use timer0 or timer1

    //
    // Configure the 32-bit periodic timer.
    //
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER1_BASE, TIMER_A, ui32SysClock/100);    //should be one second /100

    //
    // Setup the interrupts for the timer timeouts.
    //
    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Enable the timers.
    //
    TimerEnable(TIMER1_BASE, TIMER_A);


    //test
    //
    // Enable the GPIO port that is used for the on-board LEDs.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);

    //
    // Enable the GPIO pins for the LEDs (PN0 & PN1).
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_0);

    //
    // Disable processor interrupts.
    //
    IntMasterDisable();
}





/*
 *  This function sets up all the GPIO pins for unused and other pins that are used in other functions.
 *
 *  Comes after:
 *  -
 */
void gpioInit(void){
    // Unused pins, enable and set to inputs
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    GPIOPinTypeGPIOInput(GPIO_PORTP_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

}

/*
 * This function reads both encoder counts and stores them in an array
 *
 * Comes after:
 * spiInit();
 *
 * Writes to:
 * encoderVal array
 *
 * TODO: check the delay
 */
void encoderRead(void){
    pui32DataTx2[0] = 0x74; // "t" return readhead temperature
    pui32DataTx2[1] = 0x00; // Sends empty data so that the encoder will complete sending data. (40 bits in total)
    pui32DataTx2[2] = 0x00;
    pui32DataTx2[3] = 0x00;
    pui32DataTx2[4] = 0x00;
    while(SSIDataGetNonBlocking(SSI0_BASE, &pui32DataRx2[0])){
    }

    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_3,0x00);
    SysCtlDelay(10);

    for(ui32Index2 = 0; ui32Index2 < NUM_SSI_DATA2; ui32Index2++)
    {
       SSIDataPut(SSI0_BASE, pui32DataTx2[ui32Index2]);
    }
    while(SSIBusy(SSI0_BASE))
    {
    }

    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_3,GPIO_PIN_3);
    SysCtlDelay(1000);

    for(ui32Index2 = 0; ui32Index2 < NUM_SSI_DATA2; ui32Index2++)
    {
      pui32DataRx2[ui32Index2] &= 0x00FF;
      SSIDataGet(SSI0_BASE, &pui32DataRx2[ui32Index2]);
    }

    // The angle is the first 14 bits of the response.
    // TODO: check that the data RX2 is overwritten
    int num = pui32DataRx2[0]<<6;
    num = num | (pui32DataRx2[1]>>2);
    encoderVal[0] = num;

    // Read the other encoder
    while(SSIDataGetNonBlocking(SSI1_BASE, &pui32DataRx2[0]))
    {
    }
    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_1,0x00);
    SysCtlDelay(10);

    for(ui32Index2 = 0; ui32Index2 < NUM_SSI_DATA2; ui32Index2++)
    {
       SSIDataPut(SSI1_BASE, pui32DataTx2[ui32Index2]);
    }
    while(SSIBusy(SSI1_BASE)){
    }

    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_1,GPIO_PIN_1);
    SysCtlDelay(1000);

    for(ui32Index2 = 0; ui32Index2 < NUM_SSI_DATA2; ui32Index2++)
    {
      pui32DataRx2[ui32Index2] &= 0x00FF;
      SSIDataGet(SSI1_BASE, &pui32DataRx2[ui32Index2]);
    }
    // The angle is the first 14 bits of the response.
    num = pui32DataRx2[0]<<6;
    num = num | (pui32DataRx2[1]>>2);
    encoderVal[1] = num;
}


/*
 * This function sets up all the PWM pins and the gpio pins for the motor, may also optionally call SPI pins
 *
 * Comes after:
 * pwmInit()
 * sysInit()
 *
 */

void motorInit(void){
    // Setting up motor driver pins, directions PK0:1, PK2:2
       // Motor enable pins, PK1: 1, PK3: 2
       // Motor brake pins, PK4: 1, PK5: 2
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
       SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
       GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);
       // GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, ledTable[i]);
       // Pulled low on motor fault PK6:1, PK7: 2
       GPIOPinTypeGPIOInput(GPIO_PORTK_BASE, GPIO_PIN_6 | GPIO_PIN_7);
       // TODO: Cal, short all amplifier inputs together (why?)
       GPIOPinTypeGPIOOutput(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1 );
       pwmInit(); // PWM OUT 4 and PWM OUT 6, PF0 and PF2
       //set directions
       GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_0 | GPIO_PIN_2 , GPIO_PIN_0+GPIO_PIN_2);
       // turn on
       GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_1 |  GPIO_PIN_3 , GPIO_PIN_1+ GPIO_PIN_3);


       GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_0,GPIO_PIN_0); // dir for motor 2, set to forward
       GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_4,GPIO_PIN_4); // brake for motor 2, set HIGH so no braking

}

/*
 * This function sends SPI data over to the motor driver to setup the driver for 1x PWM mode
 * Comes after:
 * Motor power is supplied
 * motorInit()
 * spiInit()
 *
 * Note that the driver will NOT be set if motor power is not supplied because the driver will not be able to read the registers over SPI, even though it responds
 */

void motorDriverInit(void){
   while(SSIDataGetNonBlocking(SSI2_BASE, &pui32DataRx[0])){
   }
   pui32DataTx[0] = 0b1001000000000000; // read register 3
   pui32DataTx[1] = 0b0001000001000000; // set register 3, bit 6 and 5 to 10, option 3, 1x PWM mode
   pui32DataTx[2]=  0b1001000000000000; // read register 3
   //
   // Send 3 bytes of data.
   //
   GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_2,0x00);
   SysCtlDelay(1);
   for(ui32Index = 0; ui32Index < NUM_SSI_DATA; ui32Index++){
       GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_2,0x00); // Pull CS pin low
       SysCtlDelay(1);
       SSIDataPut(SSI2_BASE, pui32DataTx[ui32Index]); // Send data
       SysCtlDelay(1000); // wait (at least 50ns)
       GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_2,GPIO_PIN_2); // Being the CS pin high
       SSIDataGet(SSI2_BASE, &pui32DataRx[ui32Index]); // Get the data
   }
   while(SSIBusy(SSI2_BASE)){
   }

   GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_2,GPIO_PIN_2); // Make sure the pin is high
   //SysCtlDelay(1000);
   SysCtlDelay(1000);
   // Done
}

/*
 * Wrapper function that takes in a control signal for the 1xPWM mode
 * if control is zero = brake!
 */
void motor2ControlPWM(int control){
    if (control>0){
        // Positive direction
        GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_0,GPIO_PIN_0); // Set to HIGH  - forward
        GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_4,GPIO_PIN_4); // Set to HIGH - no braking
        motor2PWM(control);
    }
    else if (control<0) {
        // Negative direction
        GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_0,0);
        GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_4,GPIO_PIN_4);
        motor2PWM(-1*control);
    }
    else {
        GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_0,0);
        GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_4,0); // Set brake pin to low, brake!
    }
}


/*
 * Wrapper function to write a PWM value to the motor
 *
 * Comes after:
 * motorInit()
 * spiInit()
 * motorDriverInit()
 */

void motor1PWM(int pwmValue){
    // assuming PWM has been initialized.
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,pwmValue);

}

void motor2PWM(int pwmValue){
    // assuming PWM has been initialized.
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,pwmValue);
}

/*
 * Wrapper function to read the encoder's raw data (int), angle data (float) or optionally angle in radians (float)
 *
 * Comes after:
 * sensorUpdate()
 */
int readMotor2Raw(void){
    return encoderVal[1];
}
int readMotor1Raw(void){
    return encoderVal[0];
}

float readMotor1Angle(void){
    return ((float)readMotor1Raw()/16383.0)*360.0;
}
float readMotor2Angle(void){
    return ((float)readMotor2Raw()/16383.0)*360.0;
}
// TODO: include M_PI here
float readMotor1Rad(void){
    return readMotor1Raw()/16383*2*3.14;
}
float readMotor2Rad(void){
    return readMotor2Raw()/16383*2*3.14;
}
/*
 * Reads the motor driver status over SPI
 *
 * Comes after:
 * motorInit()
 * motorDriverInit()
 * spiInit()
 */
void readDriverStatus(void){
    while(SSIDataGetNonBlocking(SSI2_BASE, &pui32DataRx[0])){
    }
    pui32DataTx[0] = 0b1000000000000000; // read register 0
    pui32DataTx[1] = 0b1000100000000000; // read register 1
    pui32DataTx[2] = 0b1001000000000000; // read register 2
    pui32DataTx[3] = 0b1001100000000000; // read register 3
    pui32DataTx[4] = 0b1010000000000000; // read register 4
    pui32DataTx[5] = 0b1010100000000000; // read register 5
    pui32DataTx[6] = 0b1011000000000000; // read register 6
    // Last register is reserved
       //
       // Send 7 bytes of data.
       //
   GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_2,0x00);
   SysCtlDelay(1);
   for(ui32Index = 0; ui32Index < 7; ui32Index++){
       GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_2,0x00); // Pull CS pin low
       SysCtlDelay(1);
       SSIDataPut(SSI2_BASE, pui32DataTx[ui32Index]); // Send data
       SysCtlDelay(1000); // wait (at least 50ns)
       GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_2,GPIO_PIN_2); // Being the CS pin high
       SSIDataGet(SSI2_BASE, &pui32DataRx[ui32Index]); // Get the data
   }
   while(SSIBusy(SSI2_BASE)){
   }

   GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_2,GPIO_PIN_2); // Make sure the pin is high
   //SysCtlDelay(1000);
   SysCtlDelay(1000);
   // Done
   // Fault status register 1
   if (pui32DataRx[0]&(1<<0)){
       // Overcurrent on C low-side MOSFET
   }
   if (pui32DataRx[0]&(1<<1)){
       // Overcurrent on C high-side MOSFET
   }
   if (pui32DataRx[0]&(1<<2)){
       // Overcurrent on B low-side MOSFET
   }
   if (pui32DataRx[0]&(1<<3)){
       // Overcurrent on B high-side MOSFET
   }
   if (pui32DataRx[0]&(1<<4)){
       // Overcurrent on A low-side MOSFET
   }
   if (pui32DataRx[0]&(1<<5)){
       // Overcurrent on A high-side MOSFET
   }
   if (pui32DataRx[0]&(1<<6)){
       // Overtemperature
   }
   if (pui32DataRx[0]&(1<<7)){
       // Undervoltage lockout
   }
   if (pui32DataRx[0]&(1<<8)){
       // Gate drive fault
   }
   if (pui32DataRx[0]&(1<<9)){
       // Overcurrent fault
   }
   if (pui32DataRx[0]&(1<<10)){
       // mirrors nFault pin, logic OR of fault status registers
   }

   // Fault status register 2
   if (pui32DataRx[1]&(1<<0)){
       // Gate drive fault on C low-side MOSFET
   }
   if (pui32DataRx[1]&(1<<1)){
       // Gate drive fault on C high-side MOSFET
   }
   if (pui32DataRx[1]&(1<<2)){
       // Gate drive fault on B low-side MOSFET
   }
   if (pui32DataRx[1]&(1<<3)){
       // Gate drive fault on B high-side MOSFET
   }
   if (pui32DataRx[1]&(1<<4)){
       // Gate drive fault on A low-side MOSFET
   }
   if (pui32DataRx[1]&(1<<5)){
       // Gate drive fault on A high-side MOSFET
   }
   if (pui32DataRx[1]&(1<<6)){
       // Charge pump undervoltage fault
   }
   if (pui32DataRx[1]&(1<<7)){
       // Over temperature warning
   }
   if (pui32DataRx[1]&(1<<8)){
       // Overcurrent on C sense amplifier
   }
   if (pui32DataRx[1]&(1<<9)){
       // Overcurrent on B sense amplifier
   }
   if (pui32DataRx[1]&(1<<10)){
       // Overcurrent on A sense amplifier
   }
   // Register 3
   if (pui32DataRx[2]&(1<<0)){
         // Latched fault bit set
     }
     if (pui32DataRx[2]&(1<<1)){
         // Brake
         // ORed with brake pin input
         // All 3 low side MOSFETS set to 1x PWM mode
     }
     if (pui32DataRx[2]&(1<<2)){
         // Coast
         // All MOSFETS in Hi-Z state
     }
     if (pui32DataRx[2]&(1<<3)){
         // ORed with Dir pin input
         // Set direction
     }
     if (pui32DataRx[2]&(1<<4)){
         // Set asynchronous rectification in 1x PWM mode
         // Default is synchronous rectification
     }
     if (pui32DataRx[2]&(1<<5)){
         if (pui32DataRx[2]&(1<<6)){
            // Independent PWM mode
         }
         else {
            // 3X PWM mode
         }
     }
     else {
         if (pui32DataRx[2]&(1<<6)){
            // 1X PWM mode
         }
         else {
            // 6X PWM mode
         }
     }

     if (pui32DataRx[2]&(1<<7)){
         // OTW reported on nFAULT
         // default is not reported
     }
     if (pui32DataRx[2]&(1<<8)){
         // Gate drive fault if disabled
         // default is enabled
     }
     if (pui32DataRx[2]&(1<<9)){
         // Charge pump fault is disabled
         // default is enabled
     }
     if (pui32DataRx[2]&(1<<10)){
         // Reserved
     }
     // Register 4
     // Gate Drive HS Register
     // Register 5
     // Gate Drive LS Register
     // Register 6
     // OCP control Register
     // Register 7
     // CSA control Register
     if (pui32DataRx[6]&(1<<0)){
           // Sense OCP
         if (pui32DataRx[6]&(1<<1)){
             // Sense OCP 1V
             // default
         }
         else {
             // Sense OCP 0.5V
         }
       }

     else{

         if (pui32DataRx[6]&(1<<1)){
             // Sense OCP 0.75V
             // default
         }
         else {
             // Sense OCP 0.25V
         }
       }
       if (pui32DataRx[6]&(1<<2)){
           // shuts inputs to current sense amplifier C for offset calibration
           // Default: Normal current sense amplifier C operation
       }
       if (pui32DataRx[6]&(1<<3)){
           // shuts inputs to current sense amplifier B for offset calibration
           // Default: Normal current sense amplifier B operation
       }
       if (pui32DataRx[6]&(1<<4)){
           // shuts inputs to current sense amplifier A for offset calibration
           // Default: Normal current sense amplifier A operation
       }
       if (pui32DataRx[6]&(1<<5)){
           // Sense overcurrent fault is disabled
           // Default: sense overcurrent fault is enabled
       }
       if (pui32DataRx[6]&(1<<6)){
          if (pui32DataRx[6]&(1<<7)){
             // 40-V/V current sense amplifier gain
          }
          else {
             // 10-V/V current sense amplifier gain
          }
       }
       else {
           if (pui32DataRx[6]&(1<<7)){
              // 20-V/V current sense amplifier gain
              // default
           }
           else {
              // 5-V/V current sense amplifier gain
           }
       }

       if (pui32DataRx[6]&(1<<8)){
           // VDS_OCP for the low-side MOSFET is measured across SHx to SNx
           // default: VDS_OCP for the low-side MOSFET is measured across SHx to SPx
           //
       }
       if (pui32DataRx[6]&(1<<9)){
           // default: current sense amplifier refernce voltage is Vref divided by 2
           // 0: current sense amplifier refernce voltage is Vref (unidirectional mode)
       }
       if (pui32DataRx[6]&(1<<10)){
           // Current sense amplifier positive input is SHx
           // default current sense amplifier positive is SPx
       }

}

/*
 * Not being implemented
 */

int motorError(void){
   int motor1error = GPIOPinRead(GPIO_PORTK_BASE, GPIO_PIN_6);
   int motor2error = GPIOPinRead(GPIO_PORTK_BASE, GPIO_PIN_7);
   if (motor1error==0){
       // brake motor 1
       GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_4,GPIO_PIN_4);
       return 51;
   }
   else if (motor2error==0){
       // brake motor 2
       GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_5,GPIO_PIN_5);
       return 52;
   }
   else{
       return 50;
   }
}

/*
 * Control functions
 */
void PIDUpdate(void){
    ; // only update as control changes
}

void PIDCurrUpdate(void){
    ; // update the commanded current as fast as possible
}



/*
 * This function sets up the UART on UART0, PA0(RX) and PA1 (TX)
 * It is set up for 115200 baud 8N1. To send numbers and characters use
 * UARTprintf.
 */
void uartInit(void){   //There will be only UART0 and not interrupt
     //
     // Enable the GPIO Peripheral used by the UART.
     //
     ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

     //
     // Enable UART0.
     //
     ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

     //
     // Configure the GPIO pin muxing for the UART function.
     //
     GPIOPinConfigure(GPIO_PA0_U0RX);
     GPIOPinConfigure(GPIO_PA1_U0TX);

     //
     // Since GPIO A0 and A1 are used for the UART function, they must be
     // configured for use as a peripheral function (instead of GPIO).
     //
     GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
     //
     // Configure the UART for 115,200, 8-N-1 operation.
     // This function uses ui32SysClock to get the system clock
     // frequency.  This could be also be a variable or hard coded value
     // instead of a function call.                                              What does that mean?
     //
     UARTConfigSetExpClk(UART0_BASE, ui32SysClock, 115200,
                         (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                          UART_CONFIG_PAR_NONE));
}


/*
 * Sends values over UART using a char buffer
 */

void uartSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        UARTCharPut(UART0_BASE, *pui8Buffer++);
    }
}

/*
 * Sends an array over UART
 *
 * Comes after:
 * sysInit()
 * uartInit()
 */

void uartSendArray(){

}

/*
 * Receives an array over UART
 *
 * Comes after:
 * sysInit()
 * uartInit()
 */

void uartRecvArray(){

}


/*
 * This function sets up an I2C bus on I2C0, PB2 and PB3
 * Sets 400kbps
 */

void i2cInit(tI2CMInstance g_sI2CMSimpleInst){
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.
    // I2C data transfer rate set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);

    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;

    // Initialize the I2C master driver.
    I2CMInit(&g_sI2CMSimpleInst, I2C0_BASE, INT_I2C0, 0xff, 0xff, SysCtlClockGet());
}

/*
 * This function initilizes the SPI on SSI0, using PA2 (CLK), PA3(SS), PA4(RX), PA5(TX)
 */

void spiInit(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2); // SSI2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1); // SSI1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0); // SSI1
    //
    // For this example SSI0 is used with PortA[5:2].  The actual port and pins
    // used may be different on your part, consult the data sheet for more
    // information.  GPIO port A needs to be enabled so these pins can be used.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Motor
    GPIOPinConfigure(GPIO_PD3_SSI2CLK);
    GPIOPinConfigure(GPIO_PD1_SSI2XDAT0);
    GPIOPinConfigure(GPIO_PD0_SSI2XDAT1);

    // Encoder 1
    GPIOPinConfigure(GPIO_PA4_SSI0XDAT0);
    GPIOPinConfigure(GPIO_PA5_SSI0XDAT1);
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);

    // Encoder 2
    GPIOPinConfigure(GPIO_PE5_SSI1XDAT1);
    GPIOPinConfigure(GPIO_PE4_SSI1XDAT0);
    GPIOPinConfigure(GPIO_PB5_SSI1CLK);



    // Motor 1
    GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 |  GPIO_PIN_3); // SCK/MOSI/MISO (shared)
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE,GPIO_PIN_2); // CS
    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_2,GPIO_PIN_2); //Set CS to HIGH

    // Motor 2
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE,GPIO_PIN_3); // CS
    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_3,GPIO_PIN_3); //Set CS to HIGH

    // encoder 1
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_4 | GPIO_PIN_5); // MOSI/MISO
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2); // SCK
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE,GPIO_PIN_0); //CS

    // encoder 2
    GPIOPinTypeSSI(GPIO_PORTE_BASE, GPIO_PIN_5 | GPIO_PIN_4); // MOSI/MISO
    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_5); // SCK
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE,GPIO_PIN_1); //CS
    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_1,GPIO_PIN_1); // Set CS to HIGH

    SSIConfigSetExpClk(SSI2_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_1,
                           SSI_MODE_MASTER, 1000000, 16); // 16 bits for motor
    SSIConfigSetExpClk(SSI1_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_1,
                            SSI_MODE_MASTER, 1000000, 8); // 8 bits for encoder, note that we can't go above 16 bits using SPI Freescale mode
    SSIConfigSetExpClk(SSI0_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_1,
                                SSI_MODE_MASTER, 1000000, 8); // 8 bits for encoder, note that we can't go above 16 bits using SPI Freescale mode
    SSIEnable(SSI2_BASE);
    SSIEnable(SSI1_BASE);
    SSIEnable(SSI0_BASE);

}

/*
void spiInit(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);

        //
        // Configure the pin muxing for SSI0 functions on port A2, A3, A4, and A5.
        // This step is not necessary if your part does not support pin muxing.
        // TODO: change this to select the port/pin you are using.
        //

        GPIOPinConfigure(GPIO_PD3_SSI2CLK);
        GPIOPinConfigure(GPIO_PD2_SSI2FSS); // NC
        GPIOPinConfigure(GPIO_PL2_C0O); //motor 1
        GPIOPinConfigure(GPIO_PL3_C1O); // motor 2
        GPIOPinConfigure(GPIO_PD0_SSI2XDAT1);
        GPIOPinConfigure(GPIO_PD1_SSI2XDAT0);


        //
        // Configure the GPIO settings for the SSI pins.  This function also gives
        // control of these pins to the SSI hardware.  Consult the data sheet to
        // see which functions are allocated per pin.
        // The pins are assigned as follows:
        //      PA5 - SSI0Tx
        //      PA4 - SSI0Rx
        //      PA3 - SSI0Fss
        //      PA2 - SSI0CLK
        // TODO: change this to select the port/pin you are using.
        //
        GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_1 | GPIO_PIN_0 | GPIO_PIN_2 |
                       GPIO_PIN_3);
        SSIConfigSetExpClk(SSI2_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                               SSI_MODE_MASTER, 1000000, 8);
        SSIEnable(SSI2_BASE);

}*/


/*
 * This function sets up the pwm on pins PF1,PF2,PF3 which are avaliable on the TM4C123 Launchpad as RGB outputs.
 * TODO: Change PWM output pins
 */


void pwmInit(void){
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
    // Enable the peripherals used by this program.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    // SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);  //The Tiva Launchpad 123 has two modules (0 and 1). Module 1 covers the LED pins
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);  //The Tiva Launchpad 129 has one modules (0). Module 0 covers the motor pins
    //Configure PF0,PF2 Pins as PWM, note 129 M0, but 123 M1
    GPIOPinConfigure(GPIO_PF0_M0PWM0);
    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    //GPIOPinConfigure(GPIO_PF0_M1PWM4);
    //GPIOPinConfigure(GPIO_PF1_M1PWM6);
    //GPIOPinConfigure(GPIO_PF3_M1PWM7);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_2);
    // GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3);
    //Configure PWM Options
    //PWM_GEN_2 Covers M1PWM4 and M1PWM5
    //PWM_GEN_3 Covers M1PWM6 and M1PWM7 See page 207 4/11/13 DriverLib doc
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    // PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    // PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    // Set the Period (expressed in clock ticks)
    // To calculate the appropriate parameter
    // use the following equation: N = (1 / f) * SysClk.  Where N is the
    // function parameter, f is the desired frequency, and SysClk is the
    // system clock frequency.
    // In this case you get: (1 / 12500  Hz) * 4MHz = 320 cycles.  Note that
    // the maximum period you can set is 2^16 - 1.
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 320); // PF0
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, 320); // PF2
    //Set PWM duty-50% (Period /2)
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,0);
    // PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7,0);
    // Enable the PWM generator
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    // Turn on the Output pins
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_2_BIT , true);
    // PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);
}


/*
 * This function enables the ADC unit on the TM4C
 */
void adcInit(){
    // ADC MUX,
    // M1 ADC: M0 MSB, M1 LSB
    // M2 ADC: M2 MSP, M3 LSB
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    //GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    //
    // Select the analog ADC function for these pins.
    // Consult the data sheet to see which functions are allocated per pin.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0); // Current Sense 2
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1); // Current Sense 1
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2); // Temp Sense 2
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3); // Temp Sense 1
    //
    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a singal to start the
    // conversion.  Each ADC module has 4 programmable sequences, sequence 0
    // to sequence 3.  This example is arbitrarily using sequence 3.
    //
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE,0,0, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE,0,1, ADC_CTL_CH1);
    ADCSequenceStepConfigure(ADC0_BASE,0,2, ADC_CTL_CH2);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH3 | ADC_CTL_IE |
                                 ADC_CTL_END);
    //
    // Since sample sequence 3 is now configured, it must be enabled.
    //
    ADCSequenceEnable(ADC0_BASE, 0);
    //
    // Clear the interrupt status flag.  This is done to make sure the
    // interrupt flag is cleared before we sample.
    //
    ADCIntClear(ADC0_BASE, 0);
}

/*
 * Reads the ADC, accepts a pointer to a uin32_t array and reads into it
 */
void adcRead(void){
    int someint = 0;
    ADCProcessorTrigger(ADC0_BASE, 0);
    //
    // Wait for conversion to be completed.
    //
    //while(!ADCIntStatus(ADC0_BASE, 3, false))
    //{
    //}
    //
    // Clear the ADC interrupt flag.
    //
    ADCIntClear(ADC0_BASE, 0);

    //
    // Read ADC Value.
    // You will get current 2, current 1, temp 2, temp 1 in that order
    someint = ADCSequenceDataGet(ADC0_BASE, 0, adcArray);

}

/*
 * Wrapper functions for current and temperature readings
 */

uint32_t currentRead1(void){
    return adcArray[1];
}
uint32_t  currentRead2(void){
    return adcArray[0];
}
uint32_t  tempRead1(void){
    return adcArray[3];
}
uint32_t  tempRead2(void){
    return adcArray[2];
}


void MCP9600init(){
    // See https://ncd.io/k-type-thermocouple-mcp9600-with-arduino/
    // Operations:
    // Set thermo config
    // write 0x05
    // write 0x00
    // Set device config
    // write 0x06
    // write 0x00
    //I2CMwrite(i2cints,0x64,0x00,1,data,count,callback,write)
}

void MCP9600ready(){
    // Write 0x04
    // wait
    // check if slave busy, if not read
    // return int
}

void MCP9600read(){
    // write 0x00
    // read to long int = 1
    // read to long int = 2
    // if 1 &0x80 == 0x80
    // 1 = 1& 0x7F
    //temp  = 1024 - (1 *16/ + 2/16)
}
/*********************************************** CURRENTLY NOT BEING IMPLEMENTED ********************************************/





/*
 * Reads the temperature
 */
uint32_t tempRead(void){
    uint32_t pui32ADC0Value[4];
    // uint32_t ui32TempValueC;
    //
    // Trigger the ADC conversion.
    //
    ADCProcessorTrigger(ADC0_BASE, 3);

    //
    // Wait for conversion to be completed.
    //
    while(!ADCIntStatus(ADC0_BASE, 3, false))
    {
    }
    //
    // Clear the ADC interrupt flag.
    //
    ADCIntClear(ADC0_BASE, 3);

    //
    // Read ADC Value.
    //
    ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value);
    //
    // Use non-calibrated conversion provided in the data sheet.  Make
    // sure you divide last to avoid dropout.
    //
    // Read internal processor temperature
    //ui32TempValueC = ((1475 * 1023) - (2250 * pui32ADC0Value[0])) / 10230;

    return 1;
}



uint32_t currRead(void){
   uint32_t pui32ADC0Value[4];
   //
   // Trigger the ADC conversion.
   //
   ADCProcessorTrigger(ADC0_BASE, 3);

   //
   // Wait for conversion to be completed.
   //
   while(!ADCIntStatus(ADC0_BASE, 3, false))
   {
   }

   //
   // Clear the ADC interrupt flag.
   //
   ADCIntClear(ADC0_BASE, 3);

   //
   // Read ADC Value.
   //
   ADCSequenceDataGet(ADC0_BASE, 3, pui32ADC0Value);
   return 1;
}


