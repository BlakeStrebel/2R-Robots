//*****************************************************************************
// Comments by Benjamen Lim
// You can get up to 1/4 times the speed of the microcontroller
//
// qei.c - Example to demonstrate QEI on Tiva Launchpad
// This setup uses QEI0 P6/PD7 
// You can also use QEI1 PC5/PC6 in which case you don't need the PD7 HWREG calls
// https://forum.43oh.com/topic/7170-using-harware-qei-on-tiva-launchpad/
// 
// When building new projects:
// [Left click on Project] -> Show Build Settings -> CSS Build, ARM Compiler, Include options -> add "C:\ti\TivaWare_C_Series-2.1.3.156" to path
// [Left click on Project] -> Show Build Settings -> CSS Build, ARM Linker, Include lib -> add "C:\ti\TivaWare_C_Series-2.1.3.156\driverlib\ccs\Debug\driverlib.lib" 
// [Right click on Project] -> Import, Import -> General, Import from File System -> "C:\ti\ivaWare_C_Series-2.1.3.156\utils", Select the ones you want
//
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/qei.h"




int main(void) {

    // Set the clocking to run directly from the crystal.
    SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

    // Enable QEI Peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); // We are using PD7 for PhB0, QEI module 0 phase B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);

    //Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    //Set Pins to be PHA0 and PHB0
    GPIOPinConfigure(GPIO_PD6_PHA0); // QEI module 0 phase A
    GPIOPinConfigure(GPIO_PD7_PHB0);

    //Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7. 
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 |  GPIO_PIN_7);

    //Disable peripheral and int before configuration
    QEIDisable(QEI0_BASE);
    QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);

    // Configure quadrature encoder, use an arbitrary top limit of 1000
    QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET  | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), 1000);

    // Enable the quadrature encoder.
    QEIEnable(QEI0_BASE);

    //Set position to a middle value so we can see if things are working
    QEIPositionSet(QEI0_BASE, 500);

    //Add qeiPosition as a watch expression to see the value inc/dec
    volatile int qeiPosition; // declare it in the loop so that we can see the value in the debugger

    while (1) //This is the main loop of the program
    {
        qeiPosition = QEIPositionGet(QEI0_BASE); 
        SysCtlDelay (1000);
    }
}
