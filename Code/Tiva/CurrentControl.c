#include "System.h"
#include "r2r.h"
#include "CurrentControl.h"
#include "PositionControl.h"
#include "Motor.h"
#include "Utilities.h"
#include <stdio.h>

#define C1A 20
#define C1B 50
#define C2A 50
#define C2B 20

static volatile uint32_t COUNTS[8]={0};
static volatile int MOTOR1CURRENT = 0, MOTOR1REF = 0;
static volatile int MOTOR2CURRENT = 0, MOTOR2REF = 0;


static volatile int E1 = 0, Eint1 = 0, E2 = 0, Eint2 = 0, PWM1 = 0, PWM2 = 0;
static volatile float Kp = 2, Ki = 0.1;

void currentControlInit(void){

    IntMasterDisable();

    /*// Initialize ADC mux
    // M1 ADC: M0 MSB, M1 LSB
    // M2 ADC: M2 MSP, M3 LSB
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    //setADCMux(1,1);
    //setADCMux(1,2);*/

    // Peripheral Enable
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);

    // Wait for the ADC0 module to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) {;}

    // Configure the ADC to use PLL at 480 MHz divided by 20 to get an ADC clock of 24MHz
    // Note: this must be between 16 and 32 MHz for TM4C129x devices
    ADCClockConfigSet(ADC0_BASE,ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, 20);

    // Configure ADC pins
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0); // AIN3
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1); // AIN2
    //GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2); // AIN1
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3); // AIN0
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_4); // AIN9


    // Configure the ADC to use 4x hardware averaging of ADC samples (we can't sample fast enough for this)
    //ADCHardwareOversampleConfigure(ADC0_BASE, 2);

    // Configure the ADC to trigger at timer2 frequency
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_TIMER, 0);
    //ADCSequenceStepConfigure(ADC0_BASE,0,0, ADC_CTL_CH1);
    ADCSequenceStepConfigure(ADC0_BASE,0,0, ADC_CTL_CH2);
    ADCSequenceStepConfigure(ADC0_BASE,0,1, ADC_CTL_CH3);
    ADCSequenceStepConfigure(ADC0_BASE,0,2, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH9 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 0);
    IntPrioritySet(INT_ADC0SS0, 0x00);   // Set ADC interrupt to highest priority

    // Configure ADC timer
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    int samplePeriod = ui32SysClock/30000;
    TimerLoadSet(TIMER2_BASE, TIMER_A, samplePeriod-1);
    TimerControlTrigger(TIMER2_BASE, TIMER_A, true);
    TimerEnable(TIMER2_BASE, TIMER_A);

    // Enable Interrupts
    ADCIntEnable(ADC0_BASE, 0x00); // First priority
    IntEnable(INT_ADC0SS0);
    IntMasterEnable();

    //IntEnable(INT_TIMER2A);
    //TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
}

void CurrentControlIntHandler(void)
{
    static int i = 0, decctr = DECIMATION-1, ready = 1;

    ADCIntClear(ADC0_BASE, 0); // Clear interrupt  flag

    // verify interrupt
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);

    // Update Mux
    //updateMux();

    // Update Current Values
    AD0_read();
    counts_read();

    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0x00); //verify interrupt

    // Wait until we've muxed both phases
    if (ready)
    {
        //counts_read();
        switch(getMODE())
        {
            case IDLE:
            {
                // Zero control effort
                set_motor_pwm(MOTOR1, 0);
                set_motor_pwm(MOTOR2, 0);
                break;
            }
            case PWM:
            {
                // Motor PWMs have been set externally
                break;
            }
            case ITEST:
            {
                // Generate Reference Square Wave

                if (i == 0)
                {
                    MOTOR1REF = 100;
                    MOTOR2REF = 100;
                }
                else if (i % 150 == 0)
                {
                    MOTOR1REF = -MOTOR1REF;
                    MOTOR2REF = -MOTOR2REF;
                }

                if (i == getN())  // Done tracking when index equals number of samples
                {
                    setMODE(IDLE);
                    MOTOR1REF = 0;
                    MOTOR2REF = 0;
                    i = 0;
                    reset_current_error();
                }
                else
                {
                    PI_controller(MOTOR1, MOTOR1REF, MOTOR1CURRENT); // Track reference signal
                    PI_controller(MOTOR2, MOTOR2REF, MOTOR2CURRENT);

                    i++; //increment data

                    // Handle data decimation
                    decctr++;
                    if (decctr == DECIMATION)
                    {
                        buffer_write(MOTOR1REF, MOTOR2REF, MOTOR1CURRENT, MOTOR2CURRENT);   // Write current values to buffer
                        //buffer_write(COUNTS[0] - 2047 + C1A, COUNTS[1] - 2047 + C1B, COUNTS[2] - 2047 + C2A, COUNTS[3] - 2047 + C2B);
                        decctr = 0; // reset decimation counter
                    }

                }
                break;
            }
            case HOLD:
            {
                //PI_controller(MOTOR1, MOTOR1REF, MOTOR1CURRENT); // Track reference signal
                //PI_controller(MOTOR2, MOTOR2REF, MOTOR2CURRENT);
                break;
            }
            case TRACK:
            {
                //PI_controller(MOTOR1, MOTOR1REF, MOTOR1CURRENT); // Track reference signal
                //PI_controller(MOTOR2, MOTOR2REF, MOTOR2CURRENT);
                break;
            }
        }

        //ready = 0;
    }
    else
    {
        ready = 1;
    }
}

void PI_controller(int motor, int reference, int actual)
{
    if (motor == 1)
    {
        E1 = reference - actual;
        Eint1 = Eint1 + E1;
        PWM1 = Kp*E1 + Ki*Eint1;
        PWM1 = boundInt(PWM1, PWMPERIOD);
        motor1ControlPWM(PWM1);
    }
    else if (motor == 2)
    {
        E2 = reference - actual;
        Eint2 = Eint2 + E2;
        PWM2 = Kp*E2 + Ki*Eint2;
        PWM2 = boundInt(PWM2, PWMPERIOD);
        motor2ControlPWM(PWM2);
    }
}

void get_current_gains(void)   // provide position control gains
{
    char buffer[25];
    sprintf(buffer, "%f\r\n",Kp);
    UART0write(buffer);
    sprintf(buffer, "%f\r\n",Ki);
    UART0write(buffer);
}

void set_current_gains(void)   // recieve position control gains
{
    char buffer[25];
    float Kptemp, Kitemp;
    UART0read(buffer,25);                              // Store gains in buffer
    sscanf(buffer, "%f %f",&Kptemp, &Kitemp);   // Extract gains to temporary variables
    IntMasterDisable();                         // Disable interrupts briefly
    Kp = Kptemp;                                            // Set gains
    Ki = Kitemp;
    reset_current_error();
    IntMasterEnable();                          // Re-enable interrupts
}

void setCurrent(int motor, int u)
{
    if (motor == 1)
    {
        MOTOR1REF = u;
    }
    else if (motor == 2)
    {
        MOTOR2REF = u;
    }
}

void reset_current_error(void)
{
    E1 = 0; E2 = 0;
    Eint1 = 0; Eint2 = 0;
}

void counts_read(void)
{
    static int Motor1_A, Motor1_B, Motor2_A, Motor2_B;

    // Determine phase currents
    IntMasterDisable();
    Motor1_A = abs(COUNTS[0] - 2047 + C1A);
    Motor1_B = abs(COUNTS[1] - 2047 + C1B);
    Motor2_A = abs(COUNTS[2] - 2047 + C2A);
    Motor2_B = abs(COUNTS[3] - 2047 + C2B);
    IntMasterEnable();

    // Update current value and direction
    if (PWM1 >= 0)
    {
        MOTOR1CURRENT = maxInt(Motor1_A, Motor1_B);
    }
    else if (PWM1 < 0)
    {
        MOTOR1CURRENT = -1*maxInt(Motor1_A, Motor1_B);
    }
    else    // If there is no PWM, ignore reading and set current to 0
    {
        MOTOR1CURRENT = 0;
    }

    if (PWM2 >= 0)
    {
        MOTOR2CURRENT = maxInt(Motor2_A, Motor2_B);
    }
    else if (PWM2 < 0)
    {
        MOTOR2CURRENT = -1*maxInt(Motor2_A, Motor2_B);
    }
    else
    {
        MOTOR2CURRENT = 0;
    }
}

// Read AD0
void AD0_read(void)
{
    static uint32_t TEMP[8]={0};
    static int i = 0;
    // Extract ADC values into array
    ADCSequenceDataGet(ADC0_BASE, 0, TEMP);

    // Update Global COUNTS
    IntMasterDisable();
    for(i = 0; i < 8; i++)
    {
        COUNTS[i] = TEMP[i];
    }
    IntMasterEnable();

    // Clear the ADC interrupt flag.
    ADCIntClear(ADC0_BASE, 0);
}

// Print motor current in mA
// Offsets haven't been applied that isn't good
void get_mA(void)
{
    char buffer[50];
    int Motor1_A, Motor1_B, Motor2_A, Motor2_B;

    // Convert counts to mA
    IntMasterDisable();
    Motor1_A = abs(COUNTS[0] * 5.7547431 - 11428.5712);
    Motor1_B = abs(COUNTS[1] * 5.7547431 - 11428.5712);
    Motor2_A = abs(COUNTS[2] * 5.7547431 - 11428.5712);
    Motor2_B = abs(COUNTS[3] * 5.7547431 - 11428.5712);
    IntMasterEnable();

    sprintf(buffer, "%d %d %d %d\r\n", Motor1_A, Motor1_B, Motor2_A, Motor2_B);
    UART0write(buffer);
}

// Print motor current in counts
// Getting Weird behavior because of interrupts maybe?
void get_counts(void)
{
    char buffer[100];
    int Motor1_A, Motor1_B, Motor2_A, Motor2_B;

    IntMasterDisable();
    Motor1_A = COUNTS[0] - 2047 + C1A;
    Motor1_B = COUNTS[1] - 2047 + C1B;
    Motor2_A = COUNTS[2] - 2047 + C2A;
    Motor2_B = COUNTS[3] - 2047 + C2B;
    IntMasterEnable();

    sprintf(buffer, "%d %d %d %d\r\n", Motor1_A, Motor1_B, Motor2_A, Motor2_B);
    UART0write(buffer);
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
