/*
 * adc.c
 *
 *  Created on: May 31, 2018
 *      Author: Ben
 */

#include "Adc.h"



uint32_t adcArray[8]={0};

uint32_t currentArray[6]={0};

float filterArray[6]={0};


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

