//*****************************************************************************
//
// PID controller for TM4C123GXL Launchpad Example
//
// Written by: Benjamen Lim
//
// Combined PWM and QEI Example
//
// PWM pins PF1, and PF2
// QEI pins PD6, and PD7 (note PD7 has to be unlocked from NMI)
// Period set to 320 clock cycles
// Clock set to 50Mhz
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
#include "driverlib/pwm.h"


#define PERIOD 320

// Prototypes
void PWMconfig(int period);
void QEIconfig(void);

int main(void) {

    // Set the clock to 50Mhz
    SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

    PWMconfig(PERIOD);
    QEIconfig();

    // Begin

    volatile int currPosition=0;
    volatile int tempposition = 0; //Added on 1/31
    int refposition = 0;
    int dir = 0;
    int pwmout = 0;
    float Kp = 10;
    float Kd = 10;
    float Ki = 0;
    int error = 0;
    int eprev = 0;
    int ediv = 0;
    int eint = 0;
    int controlsig = 0;


    while (1)
    {

        //update position
        currPosition = currPosition + (QEIPositionGet(QEI0_BASE)-500);

        QEIPositionSet(QEI0_BASE, 500);


        // Compute position error
        error = (refposition - currPosition); // get current error
        eint = eint + error; // integrate up the error
        ediv = error-eprev; // no need to divide by time since you are multiplying by a constant.
        controlsig = Kp*error+Kd*ediv+Ki*eint;

        // Send control signal
        if (controlsig>PERIOD) {
            pwmout = PERIOD;
            dir = PERIOD; // forward full speed
        }
        else if(controlsig<-PERIOD) {
            dir = 1;
            pwmout = PERIOD; // backwards full speed
        }
        else if((controlsig<0) && (controlsig>-PERIOD)){
            pwmout = -controlsig; // backwards at controlsig
            dir = PERIOD;
        }
        else if ((controlsig>=0)&&(controlsig<PERIOD)){
            pwmout = controlsig; // forwards at controlsig
            dir = 1;
        }

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5,dir);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,pwmout);

        //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5,1);
        //PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,100);

        eprev = error; // update previous error

    }
}


void QEIconfig(){
    // QEI Config
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); // We are using PD7 for PhB0, QEI module 0 phase B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);

    //Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    //Set Pins to be PHA0 and PHB0
    GPIOPinConfigure(GPIO_PD6_PHA0); // QEI module 0 phase A
    GPIOPinConfigure(GPIO_PD7_PHB0);

    //Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7. I believe this sets the pull up and makes them inputs
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
}

void PWMconfig(int period){

    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);


    GPIOPinConfigure(GPIO_PF1_M1PWM5); //PF1
    GPIOPinConfigure(GPIO_PF2_M1PWM6); //PF2

    // Set pin types
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2);
    // PWM configuration
    PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, period);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, period);

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5,100);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,100);

    PWMGenEnable(PWM1_BASE, PWM_GEN_2);
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);

    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT , true);
}
