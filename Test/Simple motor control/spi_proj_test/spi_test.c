//*****************************************************************************
//
// spi_master.c - Example demonstrating how to configure SSI0 in SPI master
//                mode.
//
// Copyright (c) 2010-2016 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
//
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the
//   distribution.
//
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision 2.1.3.156 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#include "utils/uartstdio.h"
#include "utils/uartstdio.c"

//*****************************************************************************
//
//! \addtogroup ssi_examples_list
//! <h1>SPI Master (spi_master)</h1>
//!
//! This example shows how to configure the SSI0 as SPI Master.  The code will
//! send three characters on the master Tx then polls the receive FIFO until
//! 3 characters are received on the master Rx.
//!
//! This example uses the following peripherals and I/O signals.  You must
//! review these and change as needed for your own board:
//! - SSI0 peripheral
//! - GPIO Port A peripheral (for SSI0 pins)
//! - SSI0Clk - PA2
//! - SSI0Fss - PA3
//! - SSI0Rx  - PA4
//! - SSI0Tx  - PA5
//!
//! The following UART signals are configured only for displaying console
//! messages for this example.  These are not required for operation of SSI0.
//! - UART0 peripheral
//! - GPIO Port A peripheral (for UART0 pins)
//! - UART0RX - PA0
//! - UART0TX - PA1
//!
//! This example uses the following interrupt handlers.  To use this example
//! in your own application you must add these interrupt handlers to your
//! vector table.
//! - None.
//
//*****************************************************************************

//*****************************************************************************
//
// Number of bytes to send and receive.
//
//*****************************************************************************
#define NUM_SSI_DATA            3
#define NUM_SSI_DATA2           5

//******
// Global variables.
//******
volatile int desire_angle;

//*****************************************************************************
//
// This function sets up UART0 to be used for a console to display information
// as the example is running.
//
//*****************************************************************************
void PWMconfig(int period);

void motorPWM2(int pwmValue){
    // assuming PWM has been initialized.
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2,pwmValue);

}
void delayMS(int ms,int clock) {
    SysCtlDelay( (clock/(3*1000))*ms ) ;
}

void
UARTIntHandler(void)
{
    uint32_t ui32Status;
    char charBuf[4];
    int count = 0;
    //
    // Get the interrrupt status.
    //
    ui32Status = UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    // We need to clear the flag otherwise it will not interrupt again
    UARTIntClear(UART0_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //

    while(UARTCharsAvail(UART0_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //

        while (count<1){
            char newchar = UARTCharGetNonBlocking(UART0_BASE);
            charBuf[count] = newchar;
            count++;

        }
        desire_angle = atoi (charBuf);
        // adding one changes the letter, so you know that the value is incremented.
        //ROM_UARTCharPutNonBlocking(UART0_BASE,newchar+1);
        //char replyString[25]={};
        //sprintf(replyString,"Hello! You sent: %c\r\n",newchar);

        // Speed test 2/18/2018


        // Test 2/7/2018
        //array = &f;
        //UARTSend(array, 4);

        //Test 2/4/2018
        //array = &f;
        //array = data;
        //UARTSend(array,16);
        //UARTSend((uint8_t *) "R2R WelcomeHAHAHAHAHAHAHHHAHAHHHAHAHAHHAHAHAHAHAHAHAHAHAHHA \r\n",200);
        //Test 2/4/2018
        //UARTSend(array,15);



        // Another option that is blocking is:
        // UARTCharPut(UART0_BASE, 'r');


        //
        // Blink the LED to show a character transfer is occuring.
        //
        // GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

        //
        // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
        //
        // SysCtlDelay(SysCtlClockGet() / (1000 * 3));

        //
        // Turn off the LED
        //
        // GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

    }


}


void
InitConsole(void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    //
    // Enable UART0 so that we can configure the clock.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

//*****************************************************************************
//
// Configure SSI0 in master Freescale (SPI) mode.  This example will send out
// 3 bytes of data, then wait for 3 bytes of data to come in.  This will all be
// done using the polling method.
//
//*****************************************************************************
int
main(void)
{
    // References
    //https://e2e.ti.com/support/microcontrollers/tiva_arm/f/908/t/432569
    // https://gist.github.com/madcowswe/e360649b1f8075c042b794ac550f600b
    uint32_t ui32SysClock;


   volatile uint32_t pui32DataTx[NUM_SSI_DATA];
    uint32_t pui32DataRx[NUM_SSI_DATA];
    uint32_t ui32Index;

    volatile uint32_t pui32DataTx2[NUM_SSI_DATA2];
        uint32_t pui32DataRx2[NUM_SSI_DATA2];
        uint32_t ui32Index2;

    //
    // Set the clocking to run directly from the external crystal/oscillator.
    // TODO: The SYSCTL_XTAL_ value must be changed to match the value of the
    // crystal on your board.
    //
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                          SYSCTL_OSC_MAIN |
                                          SYSCTL_USE_OSC), 25000000);
#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)

#else
    //SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
    //               SYSCTL_XTAL_16MHZ);
#endif

    //
    // Set up the serial console to use for displaying messages.  This is
    // just for this example program and is not needed for SSI operation.
    //
    InitConsole();
    PWMconfig(700);
    motorPWM2(0);

    //
    // Display the setup on the console.
    //
    UARTprintf("SSI ->\n");
    UARTprintf("  Mode: SPI\n");
    UARTprintf("  Data: 16-bit\n\n");

    //
    // The SSI0 peripheral must be enabled for use.
    //
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);

    //
    // For this example SSI0 is used with PortA[5:2].  The actual port and pins
    // used may be different on your part, consult the data sheet for more
    // information.  GPIO port A needs to be enabled so these pins can be used.
    // TODO: change this to whichever GPIO port you are using.
    //
    // SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //
    // Configure the pin muxing for SSI0 functions on port A2, A3, A4, and A5.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //
    //GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    //GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    //GPIOPinConfigure(GPIO_PA4_SSI0XDAT0);
    //GPIOPinConfigure(GPIO_PA5_SSI0XDAT1);
    GPIOPinConfigure(GPIO_PD3_SSI2CLK);
    GPIOPinConfigure(GPIO_PD1_SSI2XDAT0);
    GPIOPinConfigure(GPIO_PD0_SSI2XDAT1);
    GPIOPinConfigure(GPIO_PE5_SSI1XDAT1);
    GPIOPinConfigure(GPIO_PE4_SSI1XDAT0);
    GPIOPinConfigure(GPIO_PB5_SSI1CLK);

    // Note that we are using PF2 here for motor 1, but this is only broken out on the 129, so we use it for our PWM


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
    //GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 |
    //               GPIO_PIN_2);
    GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 |  GPIO_PIN_3);

    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE,GPIO_PIN_2);
    GPIOPinTypeSSI(GPIO_PORTE_BASE, GPIO_PIN_5 | GPIO_PIN_4);

    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_5);
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE,GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_1,GPIO_PIN_1);

    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_2,GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE,GPIO_PIN_0 | GPIO_PIN_4);
    GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_0,GPIO_PIN_0); //dir
    GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_4,GPIO_PIN_4);

    IntMasterEnable();
    IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

    //
    // Configure and enable the SSI port for SPI master mode.  Use SSI0,
    // system clock supply, idle clock level low and active low clock in
    // freescale SPI mode, master mode, 1MHz SSI frequency, and 8-bit data.
    // For SPI mode, you can set the polarity of the SSI clock when the SSI
    // unit is idle.  You can also configure what clock edge you want to
    // capture data on.  Please reference the datasheet for more information on
    // the different SPI modes.
    //
    SSIConfigSetExpClk(SSI2_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_1,
                           SSI_MODE_MASTER, 1000000, 16);
    SSIConfigSetExpClk(SSI1_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_1,
                            SSI_MODE_MASTER, 1000000, 8);
#if defined(TARGET_IS_TM4C129_RA0) ||                                         \
    defined(TARGET_IS_TM4C129_RA1) ||                                         \
    defined(TARGET_IS_TM4C129_RA2)
    SSIConfigSetExpClk(SSI0_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, 1000000, 8);
#else
    //SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
    //                   SSI_MODE_MASTER, 1000000, 8);
#endif

    //
    // Enable the SSI0 module.
    //

    // SSIEnable(SSI0_BASE);
    SSIEnable(SSI2_BASE);
    SSIEnable(SSI1_BASE);

    //
    // Read any residual data from the SSI port.  This makes sure the receive
    // FIFOs are empty, so we don't read any unwanted junk.  This is done here
    // because the SPI SSI mode is full-duplex, which allows you to send and
    // receive at the same time.  The SSIDataGetNonBlocking function returns
    // "true" when data was returned, and "false" when no data was returned.
    // The "non-blocking" function checks if there is any data in the receive
    // FIFO and does not "hang" if there isn't.
    //
    while(SSIDataGetNonBlocking(SSI2_BASE, &pui32DataRx[0]))
    {
    }

    //
    // Initialize the data to send.
    //
    //uint8_t regAdr = 1;
    //uint16_t controlword = 0x8000 | (regAdr & 0x7) << 11; //MSbit =1 for read, address is 3 bits (MSbit is always 0), data is 11 bits
    //uint16_t recbuff = 0xbeef;

    pui32DataTx[0] = 0b1001000000000000;
    pui32DataTx[1] = 0b0001000001000000;
    pui32DataTx[2]=  0b1001000000000000;
    pui32DataTx[3]=  0b1000000000000000;

    //pui32DataTx[0] = 0x0B;  // read mode
    //pui32DataTx[1] = 0x00;  // register address to read
    //pui32DataTx[2] = 0x00;  // dummy byte to generate SCLK for reading 1 byte from MISO


    //pui32DataTx[0] = 0b100000000000000000;
    //
    // Display indication that the SSI is transmitting data.
    //
    UARTprintf("Sent:\n  ");

    //
    // Send 3 bytes of data.
    //
    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_2,0x00);
    SysCtlDelay(1);
    for(ui32Index = 0; ui32Index < NUM_SSI_DATA; ui32Index++)
    {
        //
        // Display the data that SSI is transferring.
        //
        //UARTprintf("'%c' ", pui32DataTx[ui32Index]);

        //
        // Send the data using the "blocking" put function.  This function
        // will wait until there is room in the send FIFO before returning.
        // This allows you to assure that all the data you send makes it into
        // the send FIFO.
        //
        //SSIDataPut(SSI2_BASE, controlword);
        // SSIDataPut(SSI2_BASE,recbuff);
        GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_2,0x00);
        SysCtlDelay(1);
        SSIDataPut(SSI2_BASE, pui32DataTx[ui32Index]);
        SysCtlDelay(1000);
        GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_2,GPIO_PIN_2);
        SSIDataGet(SSI2_BASE, &pui32DataRx[ui32Index]);
    }



    //
    // Wait until SSI0 is done transferring all the data in the transmit FIFO.
    //
    while(SSIBusy(SSI2_BASE))
    {
    }
    // SysCtlDelay(1);
    // GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_2,GPIO_PIN_2);
    //
    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_2,GPIO_PIN_2);
    SysCtlDelay(1000);

    //
    // Display indication that the SSI is receiving data.
    //
    UARTprintf("\nReceived:\n  ");

    //
    // Receive 3 bytes of data.
    //
    for(ui32Index = 0; ui32Index < NUM_SSI_DATA; ui32Index++)
    {
        //
        // Receive the data using the "blocking" Get function. This function
        // will wait until there is data in the receive FIFO before returning.
        //
        //
        // Since we are using 8-bit data, mask off the MSB.
        //
        pui32DataRx[ui32Index] &= 0x0FFF;

        //
        // Display the data that SSI0 received.
        //
        UARTprintf("'%c' ", pui32DataRx[ui32Index]);

    }
    motorPWM2(70);
    delayMS(1000,ui32SysClock);
    motorPWM2(1);
    GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_0|GPIO_PIN_4,GPIO_PIN_4);//turn dir off, keep brakes on
    delayMS(1000,ui32SysClock);
    motorPWM2(20);
    while(1){
        while(SSIDataGetNonBlocking(SSI1_BASE, &pui32DataRx[0]))
        {
        }
       pui32DataTx2[0] = 0x74;
       pui32DataTx2[1] = 0x00;
       pui32DataTx2[2] = 0x00;
       pui32DataTx2[3] = 0x00;
       pui32DataTx2[4] = 0x00;
       GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_1,0x00);
       SysCtlDelay(10);
       for(ui32Index2 = 0; ui32Index2 < NUM_SSI_DATA2; ui32Index2++)
           {
               //
               // Display the data that SSI is transferring.
               //
               //UARTprintf("'%c' ", pui32DataTx[ui32Index]);

               //
               // Send the data using the "blocking" put function.  This function
               // will wait until there is room in the send FIFO before returning.
               // This allows you to assure that all the data you send makes it into
               // the send FIFO.
               //
               //SSIDataPut(SSI2_BASE, controlword);
               // SSIDataPut(SSI2_BASE,recbuff);
               //GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_2,0x00);
               //SysCtlDelay(1);
               SSIDataPut(SSI1_BASE, pui32DataTx[ui32Index2]);
               //SysCtlDelay(1000);
               //GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_2,GPIO_PIN_2);


           }
           while(SSIBusy(SSI1_BASE))
          {
          }
          // SysCtlDelay(1);
          // GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_2,GPIO_PIN_2);
          //
          GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_1,GPIO_PIN_1);
          SysCtlDelay(1000);

          //
          // Display indication that the SSI is receiving data.
          //
          //UARTprintf("\nReceived:\n  ");

          //
          // Receive 5 bytes of data.
          //
          for(ui32Index2 = 0; ui32Index2 < NUM_SSI_DATA2; ui32Index2++)
          {
              //
              // Receive the data using the "blocking" Get function. This function
              // will wait until there is data in the receive FIFO before returning.
              //
              //
              //
              // Since we are using 8-bit data, mask off the MSB.
              //
              pui32DataRx2[ui32Index2] &= 0x00FF;
              SSIDataGet(SSI1_BASE, &pui32DataRx2[ui32Index2]);

              //
              // Display the data that SSI0 received.
              //
              //UARTprintf("'%d' ", pui32DataRx[ui32Index]);

          }
          int num = pui32DataRx2[0]<<6;
          //UARTprintf("%d ",num);
          num = num | (pui32DataRx2[1]>>2);
          float angle = (float)num/(16383)*360; // our current angle

          // let our desired angle be 0
          float kp = 1;
          float ki = 0;
          float e = 90 - angle;
          int control = e*kp;
          if (control > 20){
              motorPWM2(20);
              GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_0,GPIO_PIN_0);
          }
          else if (control <-20){
              GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_0|GPIO_PIN_4,GPIO_PIN_4);
              motorPWM2(20);
          }
          else if ((control > 0) & (control < 20)){
              motorPWM2(control);
              GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_0,GPIO_PIN_0);
          }
          else {
              GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_0|GPIO_PIN_4,GPIO_PIN_4);
              motorPWM2(-control);

          }

          //num = num/16383*360;

          UARTprintf("Desired angle: %d\n", desire_angle);
          UARTprintf("Current angle: %d\n", (int)angle);

          delayMS(1,ui32SysClock);
      }

    /*
    int i = 0;
    for(i=0;i<70;i++){
        motorPWM2(i);
        delayMS(10,ui32SysClock);
        GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_2,0x00);
        SysCtlDelay(1);
        SSIDataPut(SSI2_BASE, pui32DataTx[3]);
        SysCtlDelay(100);
        GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_2,GPIO_PIN_2);
    }
    */
    int motor_not_ready = 0;
    while(1){
        ;
    }
    //
    // Return no errors
    //
    return(0);
}

void PWMconfig(int period){

    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);


    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    // Set pin types
    GPIOPinTypePWM(GPIO_PORTF_BASE,GPIO_PIN_2);
    // PWM configuration
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);




    // Set the Period (expressed in clock ticks)
        // To calculate the appropriate parameter
        // use the following equation: N = (1 / f) * SysClk.  Where N is the
        // function parameter, f is the desired frequency, and SysClk is the
        // system clock frequency.
        // In this case you get: (1 / 25000  Hz) * 25MHz =  cycles.  Note that
        // the maximum period you can set is 2^16 - 1.
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, period); // PF0
        PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, period); // PF2
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
