/**
 * @file r2r.c
 * @brief Main R2R library
 *
 * This file contains all the functions for the R2R Project
 *
 * @author Benjamen Lim
 * @author Huan Weng
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/fpu.h"
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


#include "r2r.h"
#include "System.h"
#include "Encoder.h"
#include "Motor.h"
#include "PositionControl.h"
#include "CurrentControl.h"


// TODO: motor driver 2 cs pin is actually motor driver 1's CS pin

/*
 * This function does initialisation for all the default connections
 *
 * Comes after:
 * -
 */
void r2rDefaultInit(void){
    sysInit(); // Init system clock and Master interrupt
    uartInit();
    encoderSPIInit();
    MotorSPIinit();
    pwmInit(); // PWM OUT 4 and PWM OUT 6, PF0 and PF2
    motorInit(); // Set useful signal outputs.
    motorDriverInit(); // Send values to set up the motor for 1x PWM mode
    currentControlInit();
    //gpioInit(); // Init for general GPIO - set to input for safety
    //MotorTimerInit();
    timeInit();
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

uint32_t adcArray[8]={0};

uint32_t currentArray[6]={0};
float filterArray[6]={0};



void adcInit(){

    // ADC MUX,
    // M1 ADC: M0 MSB, M1 LSB
    // M2 ADC: M2 MSP, M3 LSB
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    //setADCMux(1,1);
    //setADCMux(1,2);


    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    ADCClockConfigSet(ADC0_BASE,ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, 24); // set to 480 MHz / 24 = 20MHz sample rate
    ADCHardwareOversampleConfigure(ADC0_BASE, 4);
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
    // Enable sample sequence 3 with a processor signal trigger.  Sequence 1
    // will do a single sample when the processor sends a singal to start the
    // conversion.  Each ADC module has 4 programmable sequences, sequence 0
    // to sequence 1.  This example is arbitrarily using sequence 1.
    //

    ADCSequenceDisable(ADC0_BASE, 0);
    //ADCHardwareOversampleConfigure(ADC0_BASE,32); // oversample by 32 times, so 20/32 = 625ks/s
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    //ADCHardwareOversampleConfigure(ADC0_BASE,32);
    ADCSequenceStepConfigure(ADC0_BASE,0,0, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE,0,1, ADC_CTL_CH1);
    ADCSequenceStepConfigure(ADC0_BASE,0,2, ADC_CTL_CH2);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH3 | ADC_CTL_IE |
                                 ADC_CTL_END); // takes 4 samples, so 156ks/s
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


void filterValues(){
    int i;
    float A = 0.6;
    for(i=0;i<6;i++){
        filterArray[i]= filterArray[i]*A+currentArray[i]*(1-A);
    }
}

float convertCurrent(uint32_t current){
    return((((float)current/4096.0*3.3)-1.6)*7.142857);
}

void adcCurrentRead(){

    setADCMux(1,0);
    setADCMux(2,0);
    adcRead();
    currentArray[0]=currentRead1();
    currentArray[3]=currentRead2();
    setADCMux(1,1);
    setADCMux(2,1);
    adcRead();
    currentArray[1]=currentRead1();
    currentArray[4]=currentRead2();
    setADCMux(1,2);
    setADCMux(2,2);
    adcRead();
    currentArray[2]=currentRead1();
    currentArray[5]=currentRead2();

}

/*
 * Reads the ADC, accepts a pointer to a uin32_t array and reads into it
 */
void adcRead(void){
    ADCProcessorTrigger(ADC0_BASE, 0);
    //
    // Wait for conversion to be completed.
    //
    while(!ADCIntStatus(ADC0_BASE, 0, false))
    {
    }
    //
    // Clear the ADC interrupt flag.
    //
    ADCIntClear(ADC0_BASE, 0);

    //
    // Read ADC Value.
    // You will get current 2, current 1, temp 2, temp 1 in that order
    ADCSequenceDataGet(ADC0_BASE, 0, adcArray);

}

/*
 * Wrapper functions for current and temperature readings
 */

uint32_t currentRead1(void){ // 1.6 = 0
    return adcArray[2];
}
uint32_t  currentRead2(void){
    return adcArray[3];
}
int32_t  tempRead1(void){
    return (int32_t)(((float)adcArray[0]/4096.0*3.3)*200);
}
int32_t  tempRead2(void){
    return (int32_t)(((float)adcArray[1]/4096.0*3.3)*200);
}






