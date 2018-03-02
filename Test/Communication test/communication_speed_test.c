//*****************************************************************************
// communication_speed_test.c
//
// Huan Weng
//
// This example is part of communication speed test
//
// It receives 20 float elements from Matlab via UART, does some calculation and sends them back.
//
// Use none for flow control and parity, 128000 for the baud rate, 8-n-1 mode
//
// This example uses the ROM_ prefix, which calls the TivaWare firmware in the
// ROM, which should make the program a little faster.
//
//*****************************************************************************

#define TARGET_IS_TM4C123_RB2
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

// New variables for interrupt. Are they good to be global?
char recev[80];
float data[20];
uint8_t *array;
float *getfloat;
int i;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// The UART interrupt handler.
//
// Note that the UART interrupt handler requires a definition in startup_ccs.c
//
//*****************************************************************************


void
UARTIntHandler(void)
{
    uint32_t ui32Status;

    //
    // Get the interrrupt status.
    //
    ui32Status = ROM_UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    // We need to clear the flag otherwise it will not interrupt again
    ROM_UARTIntClear(UART0_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    while(ROM_UARTCharsAvail(UART0_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //
        for (i = 0; i < 4 * 20; i++) {
            recev[i] = ROM_UARTCharGet(UART0_BASE);
        }
        getfloat = &recev;
        // adding one changes the letter, so you know that the value is incremented.
        //ROM_UARTCharPutNonBlocking(UART0_BASE,newchar+1);
        // Another option that is blocking is:
        // UARTCharPut(UART0_BASE, 'r');

        //Change something.
        for (i = 0; i < 20; i++) {
            data[i] = 0.1 + *getfloat++;
        }
        //Convert the variable types.
        array = &data;

        // Every time we decide to send a 4*5 array back.

        for (i = 0; i < 20 * 4; i++) {
            // What is the real difference between blocking and non-blocking? Blocking will not stick on this line.
            ROM_UARTCharPut(UART0_BASE, *array++);
        }



        //
        // Blink the LED to show a character transfer is occuring.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

        //
        // Delay for 1 millisecond.  Each SysCtlDelay is about 3 clocks.
        //
        SysCtlDelay(SysCtlClockGet() / (1000 * 3));

        //
        // Turn off the LED
        //
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);

    }
}

//*****************************************************************************
//
// Send a string to the UART.
//
//*****************************************************************************
void
UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
    //
    // Loop while there are more characters to send.
    //
    while(ui32Count--)
    {
        //
        // Write the next character to the UART.
        //
        ROM_UARTCharPut(UART0_BASE, *pui8Buffer++);
    }
}


int
main(void)
{
    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    // This is not necessary and can be removed.
    // ROM_FPUEnable();
    // ROM_FPULazyStackingEnable();

    //
    // Use PLL to set our time, and lock it to the external crystal (defined as 16Mhz)
    // PLL generates a 200Mhz clock no matter the input clock
    // Divide by 4 to get a final clock of 50Mhz
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Enable the GPIO pins for the LED (PF2).
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

    //
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable processor interrupts.
    //
    ROM_IntMasterEnable();

    //
    // Set GPIO A0 and A1 as UART pins.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Configure the UART for 115,200, 8-N-1 operation.
    //
    ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 128000,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    //
    // Enable the UART interrupt.
    //
    ROM_IntEnable(INT_UART0);
    ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

    //
    // Prompt for text to be entered.
    //
    //UARTSend((uint8_t *)"\033[2JEnter text: ", 16);

    //
    // Loop forever echoing data through the UART.
    //
    while(1)
    {
    }
}
