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
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "utils/uartstdio.h"



#include "r2r.h"

uint32_t adcArray[4]={0};
char user_select = 'a';
extern int run_program = 1; // this will be declared in the main loop
uint8_t last_input = 'm';
extern volatile uint8_t more_input;
extern volatile uint8_t charBuf[100];


/*
 *
 * The system init function initiazes the system clock and the processor interrupts (can we?)
 * Clock: 50Mhz
 */
void delayMS(int ms) {
    SysCtlDelay( (SysCtlClockGet()/(3*1000))*ms ) ;
}

void sysInit(void){
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                           SYSCTL_XTAL_16MHZ);
    IntMasterEnable();
}

/*
 * This function explicitly checks for safety conditions, and it uses a global variable run_program to turn off the motor
 * TODO: save last state to a permanent storage device and check it upon starting. If last state was some error state, user must clear the flag
 */
void safetyCheck(void){
    int errmsg;
    errmsg = motorError();
}

/*
 * This function updates all the sensors that we are using. Usually runs once every loop
 * TODO: Add sensor functions
 */
void sensorUpdate(void){
    adcRead();
}

/*
 * This function sets up the menu
 * It assumes that the uart is already open
 */

void menuHeader(uint8_t input){
    UARTprintf("\033[2J"); // Clear the screen
    UARTprintf("\033[0;0H"); // Send cursor to top of screen
    UARTprintf("R2R Arm debugging \r\n");
    UARTprintf("_________________ \r\n");
    UARTprintf("____  o------o __ \r\n");
    UARTprintf("_____         \\  \r\n");
    UARTprintf("__________     \\  \r\n");
    UARTprintf("___________     \\ \r\n");
    UARTprintf("Press 'm' to return to main menu \r\n");
    UARTprintf("Command: %c \r\n", input);
}

/*
 * This function is used to handle the interface from MATLAB
 *
 * It gets values specified by the user in MATLAB and processes them for use when the user presses run
 *
 * To use a custom function with MATLAB, case 'c' is provided.
 * Each get command must be preceded by a printf statement
 * It must end with the char 'z' to let MATLAB know that all the varibles have been sent
 */

void matlabMenu(char menuchar){
    //char menuchar = UARTCharGet(UART0_BASE);
    int count = 0;
    float* floatArray;
    char charArray[100];
    switch(menuchar){
    case 'a':
        count  = 0;
        while (count<4){
            char newchar = UARTCharGet(UART0_BASE);
            //char newchar = UARTCharGetNonBlocking(UART0_BASE);
            charBuf[count] = newchar;
            count++;
        }
        break;
    case 'b': // PID gains control
        count = 0;
        int i = 0;
        while(!UARTCharsAvail(UART0_BASE)){ // wait until chars are available
           ;
        }
        while (UARTCharsAvail(UART0_BASE)){ // read
           // process code here
           char newchar = UARTCharGet(UART0_BASE);
           //char newchar = UARTCharGetNonBlocking(UART0_BASE);
           charArray[count] = newchar;
           count++;
       }
        uint8_t *array;
        array = &charArray;
        for (i=0;i<count;i++){
            UARTCharPut(UART0_BASE,*array++);
        }
        UARTCharPut(UART0_BASE,'\r\n');
        /*
        floatArray = (float *)&charArray;

        float data[3];
        //float kp = floatArray[0];
        //float ki = floatArray[1];
        //float kd = floatArray[2];
        int i;
        for (i = 0;i<count;i++){
            data[i] = 0.1+*floatArray++;
        }
        uint8_t *array;
        array = &charArray;
        for (i = 0;i<3*4;i++){
            UARTCharPut(UART0_BASE,*array++);
        }*/

        // TODO: Add some confirmation, save to global variable?
        break;

    case 'c': // custom code
        UARTprintf("Please set gains for non linear system (kp ki kd):\r\n"); // send string
        //delayMS(1);
        UARTprintf("num\r\n"); // send string

        count  = 0;

        char newchar;
        while(!UARTCharsAvail(UART0_BASE)){
            ;
        }
        while (UARTCharsAvail(UART0_BASE)){
            // process code here
            char newchar = UARTCharGet(UART0_BASE);
            //char newchar = UARTCharGetNonBlocking(UART0_BASE);
            charArray[count] = newchar;
            count++;
        }
        floatArray = &charArray;
        //kp = floatArray[0];
        //ki = floatArray[1];
        //kd = floatArray[2];
        UARTprintf("Please set some other value for non linear system (abcd):\r\n");
        UARTprintf("s\r\n");
        count  = 0;
        while (count<4){
            newchar = UARTCharGet(UART0_BASE);
            //char newchar = UARTCharGetNonBlocking(UART0_BASE);
            charBuf[count] = newchar;
            count++;
        }
        UARTprintf("z\r\n");
        UARTprintf("z\r\n");
        // UARTCharPut(UART0_BASE, 'z');
        break;

    }


}


void menuInit(void){
    uint8_t buffer[100]={};
    sprintf(buffer,"\033[2J\r\nR2R Arm debugging \r\n");
    uartSend((uint8_t *)buffer,100);
    sprintf(buffer,"This is a test for the ADC\r\n");
    uartSend((uint8_t *)buffer,100);
    UARTprintf("R2R UART Menu interface\r\n");
}

/*
 * This function contains all the menu interactions with the user
 */

void menuOptions(uint8_t input){
    // Generate menu here
    menuHeader(input);

    /*uint8_t next_input;
    if (last_input == 'c'){
        next_input = input;
        input = 'c';
    }
    */
    switch(input){
    case 'm':
        UARTprintf("a: Check set values \r\n");
        UARTprintf("b: Set new values \r\n");
        UARTprintf("c: More options \r\n");
        UARTprintf("\r\nr: Start Arm \r\n");
        break;
    case 'a':
        UARTprintf("Values for control: \r\n");
        UARTprintf("%c %c %c %c",charBuf[0],charBuf[1],charBuf[2],charBuf[3]);
        break;
    case 'b':
        UARTprintf("Please type in your input: \r\n");
        int count = 0;
        while (count<4){
            char newchar = UARTCharGet(UART0_BASE);
            //char newchar = UARTCharGetNonBlocking(UART0_BASE);
            charBuf[count] = newchar;
            count++;
        }
        UARTprintf("\r\nOkay. Values set \r\n");

        //UARTprintf("You typed in %c",newchar);
        //more_input = 1;
        break;
    case 'c':
        UARTprintf("Type in more options: \r\n");
        //switch(next_input){
        //case 'a':
          //  UARTprintf("Nested menu option \r\n");
          //  input = 'a';
          //  break;
        //}
        break;
    default:
        UARTprintf("Command not recognized!\r\n");
        break;
    }
    last_input = input;

}

void processUserInput(void){
    switch (user_select) {
    case 'a': // do option 1
        UARTprintf("Beginning loaded options:");
        break;
    case 'b': // do option 2
        UARTprintf("Setting up constants kp, ki, kd:");
        // process everything that is in the buffer
        // error checking
        break;
    }


}

/*
 *  This function sets up all the GPIO pins for unused and other pins that are used in other functions. .
 */
void gpioInit(void){

    // Unused pins, enable and set to inputs
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    GPIOPinTypeGPIOInput(GPIO_PORTP_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);


}

/*
 * This function sets up all the PWM pins and the gpio pins for the motor, may also optionally call SPI pins
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

}

void motorPWM1(int pwmValue){
    // assuming PWM has been initialized.
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4,pwmValue);

}

void motorPWM2(int pwmValue){
    // assuming PWM has been initialized.
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6,pwmValue);
}

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
 * It is set up for 128000 baud 8N1
 */
void uartInit(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Configure the UART for 128,000, 8-N-1 operation.
    //
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 128000,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));

    //
    // Enable the UART interrupt.
    //
    //IntEnable(INT_UART0);
    //UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);

}


void
initConsole(void)
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
    UARTStdioConfig(0, 128000, 16000000);
}



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
    ;
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
    ;
}

/*
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
*/

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
