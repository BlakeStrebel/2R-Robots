#include "System.h"
#include "r2r.h"
#include "Current.h"
#include "Motor.h"
#include "Utilities.h"
#include <stdio.h>



void currentControlInit(void){

    IntMasterDisable();

    /*// Initialize ADC mux
    // M1 ADC: M0 MSB, M1 LSB
    // M2 ADC: M2 MSP, M3 LSB
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    //setADCMux(1,1);
    //setADCMux(1,2);*/

    // Enable the ADC0 Module
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

    // Wait for the ADC0 module to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) {;}

    // Configure the ADC to use PLL at 480 MHz divided by 24 to get an ADC clock of 20MHz
    // Note: this must be between 16 and 32 MHz for TM4C129x devices
    ADCClockConfigSet(ADC0_BASE,ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, 24);

    // Configure ADC pins
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0); // AIN3
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1); // AIN2
    //GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2); // AIN1

    // Configure the ADC to use 16x hardware averaging of ADC samples
    ADCHardwareOversampleConfigure(ADC0_BASE, 16);

    // Configure the ADC to use internal 3V reference
    //ADCReferenceSet(ADC0_BASE,ADC_REF_INT);

    // Enable the first sample sequencer to capture the value of channels 0-3 when
    // the processor trigger occurs
    ADCSequenceDisable(ADC0_BASE, 0);
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    //ADCSequenceStepConfigure(ADC0_BASE,0,0, ADC_CTL_CH1);
    ADCSequenceStepConfigure(ADC0_BASE,0,0, ADC_CTL_CH2);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH3 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 0);
    ADCIntClear(ADC0_BASE, 0);

    IntMasterEnable();
}

void AD0_read(uint32_t * adcArray)
{
    ADCProcessorTrigger(ADC0_BASE, 0);

    // Wait for conversion to be completed.
    while(!ADCIntStatus(ADC0_BASE, 0, false)){;}

    // Extract ADC values into array
    ADCSequenceDataGet(ADC0_BASE, 0, adcArray);

    char buffer[10];
    sprintf(buffer, "%d\r\n", adcArray[0]);
    UART0write(buffer);
    sprintf(buffer, "%d\r\n", adcArray[1]);
    UART0write(buffer);

    adcArray[0] = adcArray[0] * 5.7547431 - 11428.5712;
    adcArray[1] = adcArray[1] * 5.7547431 - 11428.5712;

    sprintf(buffer, "%d\r\n", adcArray[0]);
    UART0write(buffer);
    sprintf(buffer, "%d\r\n", adcArray[1]);
    UART0write(buffer);

    // Clear the ADC interrupt flag.
    ADCIntClear(ADC0_BASE, 0);
}

// Returns current in mA
int mA_read(void)
{
    static float Motor1_PhaseA = 0, Motor1_PhaseB = 0;
    static uint32_t counts[8]={0};
    static int current = 0;

    // Read current in ADC counts
    AD0_read(counts);

    // Convert counts to mA
    Motor1_PhaseA = abs(counts[0] * 5.7547431 - 11428.5712);
    Motor1_PhaseB = abs(counts[1] * 5.7547431 - 11428.5712);

    // Return current with maximum absolute value
    if (Motor1_PhaseA > Motor1_PhaseB)
    {
        current = Motor1_PhaseA;
    }
    else
    {
        current = Motor1_PhaseB;
    }

    return current;
}

void Timer2IntHandler(void)
{
    //static int Motor1Current = 0;

    //Motor1Current = adcRead();

    switch(getMODE())
    {
        case IDLE:
        {
            //set_motor_pwm(1, 0);
            //set_motor_pwm(2, 0);
        }
        case PWM:
        {
            break;  // Motor PWMs have been set externally
        }
        case ITEST:
        {

        }
        case HOLD:
        {

        }
        case TRACK:
        {

        }
    }
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
}

// Mux being used to read current values
void setADCMux(int motor,int number){
    switch(motor){
    case 1:
        switch(number){
        case 1:
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_0,0);
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_1,0);
            break;
        case 2:
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_0,GPIO_PIN_0);
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_1,0);
            break;
        case 3:
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_0,0);
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_1,GPIO_PIN_1);
            break;
        case 4:
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_0,GPIO_PIN_0);
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_1,GPIO_PIN_1);
            break;
        }
    case 2:
        switch(number){
        case 1:
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_2,0);
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_3,0);
            break;
        case 2:
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_2,GPIO_PIN_2);
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_3,0);
            break;
        case 3:
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_2,0);
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_3,GPIO_PIN_3);
            break;
        case 4:
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_2,GPIO_PIN_2);
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_3,GPIO_PIN_3);
            break;
        }
    }
}





