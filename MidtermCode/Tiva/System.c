#include "System.h"

/*
 * The system init function initiazes the system clock and the processor interrupts
 * Clock: 120Mhz
 *
 * Comes after:
 * -
 */
void sysInit(void){
    // Enable and wait for the port to be ready for access
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION)) {;}

    //
    // Set the clock to run directly from the crystal at 120MHz.
    //
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN |
                                             SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480), 120000000); // Use 25Mhz crystal and use PLL to accelerate to 120MHz
}

/*
 * This function sets up the UART on UART0, PA0(RX) and PA1 (TX)
 * It is set up for 115200 baud 8N1. To send numbers and characters use
 * UARTprintf.
 */
void uartInit(void){
    // Enable GPIO port A which is used for UART0 pins.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configure UART0 pins on port A0 and A1.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    // Enable UART0 so that we can configure the clock.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Select the alternate (UART) function for these pins.
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE, ui32SysClock, 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
}

void UART0read(char * message, int maxLength)
{
    char data = 0;
    int complete = 0, num_bytes = 0;
    // loop until you get a '\r' or '\n'
    while (!complete)
    {
        data = UARTCharGet(UART0_BASE); // read char when it becomes available
        if ((data == '\n') || (data == '\r')) {
            complete = 1;
        }
        else
        {
            message[num_bytes] = data;
            ++num_bytes;
            // roll over if the array is too small
            if (num_bytes >= maxLength) {
                num_bytes = 0;
            }
        }
    }
    // end the string
    message[num_bytes] = '\0';
}

void UART0write(const char * string)
{
    while (*string != '\0') {
        UARTCharPut(UART0_BASE, *string); // send the data
        ++string;
    }
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
    GPIOPinTypeGPIOInput(GPIO_PORTP_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE,GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTP_BASE,GPIO_PIN_3,GPIO_PIN_3);
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

