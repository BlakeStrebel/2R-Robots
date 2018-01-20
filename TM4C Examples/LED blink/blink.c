/*
 * This example blinks the LEDS on the launchpad in successive order using a table.
 * It shows how to enable basic digital write HIGH and LOWs to the pins
 */


// Create some LEDs for blinking
#define RED_LED   GPIO_PIN_1
#define BLUE_LED  GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3
#define SLOW

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

/**
 * main.c
 */
int main(void)
{

    //
    // Setup the system clock to run at 50 Mhz from PLL with crystal reference
    //
    SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|
                    SYSCTL_OSC_MAIN);

    //
    // Enable and wait for the port to be ready for access
    //

    // Port F for LEDs

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }

    //
    // Set the outputs to OUTPUT mode
    //

    // LEDs output
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED);


    // State table for state.
    uint8_t ledTable[6] = { RED_LED,
                            BLUE_LED,
                            GREEN_LED,
                            RED_LED+BLUE_LED,
                            GREEN_LED+BLUE_LED,
                            RED_LED+GREEN_LED};

    // Accelerate up so we can take advantage of a faster speed.
    int j = 0;
    for (j = 0;j<93;j=j+6){
        int i = 0;
        for(i = 0; i < 6; i++) {
                
                    GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, ledTable[i]);
                    SysCtlDelay(1000000-(j+i)*10000);

        }


    }// Breaks at j=96



    while(1) {
        // Commutation process: index into state table and apply that state to the LED port for a short time.
        // After the delay time, increment the state and repeat
        int i = 0;

        for(i = 0; i < 6; i++) {
            //bitmask to turn on LEDs
            GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, ledTable[i]);
            SysCtlDelay(40000); 
        }
    }

    return 0;
}
