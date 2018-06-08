#include "r2r.h"
#include "CurrentControl.h"
#include "PositionControl.h"
#include "Motor.h"
#include "Utilities.h"
#include <stdio.h>

#define SWITCHDELAY 30              // Number of Cycles to Delay updating current on state switch
#define MA_PER_COUNT 5.75474330357  // Conversion between ADC counts and mA

// Voltage offsets for the motor phases in counts
static volatile int M1_A_OFFSET = 0;
static volatile int M1_B_OFFSET = 0;
static volatile int M2_A_OFFSET = 0;
static volatile int M2_B_OFFSET = 0;

// Current sensing
static volatile uint32_t M1_A_COUNTS, M1_B_COUNTS, M2_A_COUNTS, M2_B_COUNTS;
static volatile int MOTOR1CURRENT = 0, MOTOR1REF = 0;
static volatile int MOTOR2CURRENT = 0, MOTOR2REF = 0;

// Current control data
static volatile int E1 = 0, Eint1 = 0, E2 = 0, Eint2 = 0;
static volatile float Kp = 5, Ki = 0.15;
static volatile int PWM1 = 0, PWM2 = 0;

// Initialize ADC0 to read phase currents for each motor using fixed frequency interrupt
void currentControlInit(void){

    IntMasterDisable();

    // Peripheral Enable
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);

    // Wait for the ADC0 module to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) {;}

    // Configure the ADC to use PLL at 480 MHz divided by 20 to get an ADC clock of 24MHz
    // Note: this must be between 16 and 32 MHz for TM4C129x devices
    ADCClockConfigSet(ADC0_BASE,ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, 20);

    // Configure the ADC to use 2x hardware averaging of ADC samples
    ADCHardwareOversampleConfigure(ADC0_BASE, 2);

    // Configure ADC pins
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0); // AIN3 (Motor 1 A)
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1); // AIN2 (Motor 1 B)
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2); // AIN1 (Motor 2 A)
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3); // AIN0 (Motor 2 B)

    // Configure the ADC to trigger at timer2 frequency
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_TIMER, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH3);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH2);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH1);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH0| ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 0);
    IntPrioritySet(INT_ADC0SS0, 0x40);   // Set ADC interrupt to second highest priority

    // Configure ADC timer
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    int samplePeriod = ui32SysClock/15000;  // Read ADC at 15,000Hz, current control runs at this frequency
    TimerLoadSet(TIMER2_BASE, TIMER_A, samplePeriod-1);
    TimerControlTrigger(TIMER2_BASE, TIMER_A, true);
    TimerEnable(TIMER2_BASE, TIMER_A);

    // Enable Interrupts
    ADCIntEnable(ADC0_BASE, 0x00);
    IntEnable(INT_ADC0SS0);
    IntMasterEnable();
}

void CurrentControlIntHandler(void)
{
    static int i = 0, decctr = 0;   // Indexing values
    static int m1_A_calib = 0, m1_B_calib = 0, m2_A_calib = 0, m2_B_calib = 0;  // Current sense calibration

    ADCIntClear(ADC0_BASE, 0); // Clear interrupt  flag

    AD0_read(0);    // Update ADC values
    counts_read();  // Update motor current from phase currents

    switch(getMODE())
    {
        case IDLE:
        {
            // Zero control effort
            motor1ControlPWM(0);
            motor2ControlPWM(0);
            reset_current_error();
            break;
        }
        case PWM:
        {
            // Motor PWMs have been set externally
            break;
        }
        case HOLD:
        {
            PI_controller(MOTOR1, MOTOR1REF, MOTOR1CURRENT); // Track reference signal
            PI_controller(MOTOR2, MOTOR2REF, MOTOR2CURRENT);
            break;
        }
        case TRACK:
        {
            PI_controller(MOTOR1, MOTOR1REF, MOTOR1CURRENT); // Track reference signal
            PI_controller(MOTOR2, MOTOR2REF, MOTOR2CURRENT);
            break;
        }
        case ICALIB:
        {
            if (i == 1000)  // Done sensing when index equals number of samples
            {
                // Find average deviation from zero current and apply as offset
                M1_A_OFFSET = -1*m1_A_calib/1000;
                M1_B_OFFSET = -1*m1_B_calib/1000;
                M2_A_OFFSET = -1*m2_A_calib/1000;
                M2_B_OFFSET = -1*m2_B_calib/1000;

                // Reset calibration sums
                m1_A_calib = 0;
                m1_B_calib = 0;
                m2_A_calib = 0;
                m2_B_calib = 0;

                setMODE(IDLE);
                i = 0;
            }
            else
            {
                i++; //increment data

                // Sum deviations from zero current
                m1_A_calib = m1_A_calib + M1_A_COUNTS - 2047;
                m1_B_calib = m1_B_calib + M1_B_COUNTS - 2047;
                m2_A_calib = m2_A_calib + M2_A_COUNTS - 2047;
                m2_B_calib = m2_B_calib + M2_B_COUNTS - 2047;
            }

            break;
        }
        case ISENSE:
        {
            if (i == getN())  // Done sensing when index equals number of samples
            {
                setMODE(IDLE);
                i = 0;
            }
            else
            {
                i++; //increment data

                // Handle data decimation
                decctr++;
                if (decctr == DECIMATION)
                {
                    buffer_write(MOTOR1REF, MOTOR1CURRENT, MOTOR2REF, MOTOR2CURRENT);
                    decctr = 0; // reset decimation counter
                }
            }
            break;
        }
        case ITEST:
        {
            // Generate Reference Square Wave
            if (i == 0)
            {
                MOTOR1REF = 200;    // 200 count amplitude (~1150mA)
                MOTOR2REF = 200;
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
                motor1ControlPWM(0);
                motor2ControlPWM(0);
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
                    buffer_write(MOTOR1REF, MOTOR1CURRENT, MOTOR2REF, MOTOR2CURRENT);
                    decctr = 0; // reset decimation counter
                }

            }
            break;
        }
        case ITRACK:
        {
            // Reference current has been set by client

            if (i == getN())  // Done sensing when index equals number of samples
            {
                setMODE(IDLE);
                i = 0;
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
                    buffer_write(MOTOR1REF, MOTOR1CURRENT, MOTOR2REF, MOTOR2CURRENT);
                    decctr = 0; // reset decimation counter
                }
            }
            break;
        }
    }
}

// Calculate motor pwm using current error
void PI_controller(int motor, int reference, int actual)
{
    if (motor == 1)
    {
        E1 = reference - actual;
        Eint1 = Eint1 + E1;
        PWM1 = Kp*E1 + Ki*Eint1;
        PWM1 = boundInt(PWM1, PWMPERIOD-1);
        motor1ControlPWM(PWM1);
    }
    else if (motor == 2)
    {
        E2 = reference - actual;
        Eint2 = Eint2 + E2;
        PWM2 = Kp*E2 + Ki*Eint2;
        PWM2 = boundInt(PWM2, PWMPERIOD-1);
        motor2ControlPWM(PWM2);
    }
}

// Print current control gains over UART
void get_current_gains(void)
{
    char buffer[25];
    sprintf(buffer, "%f\r\n",Kp);
    UART0write(buffer);
    sprintf(buffer, "%f\r\n",Ki);
    UART0write(buffer);
}

// Set current control gains over UART
void set_current_gains(void)
{
    char buffer[25];
    float Kptemp, Kitemp;
    UART0read(buffer,25);                       // Store gains in buffer
    sscanf(buffer, "%f %f",&Kptemp, &Kitemp);   // Extract gains to temporary variables
    IntMasterDisable();                         // Disable interrupts briefly
    Kp = Kptemp;                                // Set gains
    Ki = Kitemp;
    reset_current_error();
    IntMasterEnable();                          // Re-enable interrupts
}

// Set reference current for motor current controller
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

// Return motor current
int getCurrent(int motor)
{
    int current;
    if (motor == 1)
    {
        current = MOTOR1CURRENT;
    }
    else if (motor == 2)
    {
        current = MOTOR2CURRENT;
    }
    return current;
}

// Return motor PWM
int getPWM(int motor)
{
    int pwm;
    if (motor == 1)
    {
        pwm = PWM1;
    }
    else if (motor == 2)
    {
        pwm = PWM2;
    }
    return pwm;
}

// Reset error being used for current control
void reset_current_error(void)
{
    E1 = 0; E2 = 0;
    Eint1 = 0; Eint2 = 0;
}

// Process phase voltages to update motor currents in counts
void counts_read(void)
{
    static int M1_A, M1_B, M2_A, M2_B;
    static int M1_current_prev = 0, M2_current_prev = 0;
    static int32_t M1_hallstate = 0, M1_hallstate_prev = 0, M1_hallstate_cntr = 0;
    static int32_t M2_hallstate = 0, M2_hallstate_prev = 0, M2_hallstate_cntr = 0;

    // Offset phase currents to be centered at zero using calibration data
    IntMasterDisable();
    M1_A = (int)M1_A_COUNTS - 2047 + M1_A_OFFSET;
    M1_B = (int)M1_B_COUNTS - 2047 + M1_B_OFFSET;
    M2_A = (int)M2_A_COUNTS - 2047 + M2_A_OFFSET;
    M2_B = (int)M2_B_COUNTS - 2047 + M2_B_OFFSET;
    IntMasterEnable();

    // Update motor 1 current and direction
    M1_hallstate = getmotor1HALLS();
    switch (M1_hallstate)
    {
        case M1_HALLSTATE_0:
            MOTOR1CURRENT = -1*M1_B;
            break;
        case M1_HALLSTATE_1:
            MOTOR1CURRENT = -1*M1_A;
            break;
        case M1_HALLSTATE_2:
            MOTOR1CURRENT = M1_B;
            break;
        case M1_HALLSTATE_3:
            MOTOR1CURRENT = M1_B;
            break;
        case M1_HALLSTATE_4:
            MOTOR1CURRENT = M1_A;
            break;
        case M1_HALLSTATE_5:
            MOTOR1CURRENT = M1_A;
            break;
    }

    // Smooth voltage spikes when commutation state changes
    if (M1_hallstate != M1_hallstate_prev)
    {
        // wait SWITCHDELAY control cycles before updating current
        if (M1_hallstate_cntr < SWITCHDELAY)
        {
            MOTOR1CURRENT = M1_current_prev;
            M1_hallstate_cntr++;
        }
        else
        {
            M1_hallstate_prev = M1_hallstate;
            M1_hallstate_cntr = 0;
        }
    }

    M1_current_prev = MOTOR1CURRENT;    // Update previous current value

    // Update motor 2 current and direction
    M2_hallstate = getmotor2HALLS();
    switch (M2_hallstate)
    {
        case M2_HALLSTATE_0:
            MOTOR2CURRENT = -1*M2_B;
            break;
        case M2_HALLSTATE_1:
            MOTOR2CURRENT = -1*M2_A;
            break;
        case M2_HALLSTATE_2:
            MOTOR2CURRENT = M2_B;
            break;
        case M2_HALLSTATE_3:
            MOTOR2CURRENT = M2_B;
            break;
        case M2_HALLSTATE_4:
            MOTOR2CURRENT = M2_A;
            break;
        case M2_HALLSTATE_5:
            MOTOR2CURRENT = M2_A;
            break;
    }

    // Smooth voltage spikes when commutation state changes
    if (M2_hallstate != M2_hallstate_prev)
    {
        // wait SWITCHDELAY control cycles before updating current
        if (M2_hallstate_cntr < SWITCHDELAY)
        {
            MOTOR2CURRENT = M2_current_prev;
            M2_hallstate_cntr++;
        }
        else
        {
            M2_hallstate_prev = M2_hallstate;
            M2_hallstate_cntr = 0;
        }
    }

    M2_current_prev = MOTOR2CURRENT;    // Update previous current value
}

// Read AD0 containing raw phase voltages measured across shunt resistor
void AD0_read(int mux)
{
    static uint32_t TEMP[8]={0};

    // Extract ADC values into array
    ADCSequenceDataGet(ADC0_BASE, 0, TEMP);

    // Update Global COUNTS
    IntMasterDisable();
    M1_A_COUNTS = TEMP[0];
    M1_B_COUNTS = TEMP[1];
    M2_A_COUNTS = TEMP[2];
    M2_B_COUNTS = TEMP[3];
    IntMasterEnable();
}

// Print motor current in mA
void get_mA(void)
{
    char buffer[10];
    int M1_current_mA, M2_current_mA;

    // Convert counts to mA
    IntMasterDisable();
    M1_current_mA = MOTOR1CURRENT*MA_PER_COUNT;
    M2_current_mA = MOTOR2CURRENT*MA_PER_COUNT;
    IntMasterEnable();

    sprintf(buffer, "%d %d\r\n", M1_current_mA, M2_current_mA);
    UART0write(buffer);
}

// Print motor current in counts
void get_counts(void)
{
    char buffer[10];
    int M1_current_counts, M2_current_counts;

    IntMasterDisable();
    M1_current_counts = MOTOR1CURRENT;
    M2_current_counts = MOTOR2CURRENT;
    IntMasterEnable();

    sprintf(buffer, "%d\r\n", M1_current_counts);
    UART0write(buffer);
    sprintf(buffer, "%d\r\n", M2_current_counts);
    UART0write(buffer);
}

// Mux being used to read current values, this function is only necessary on the PCB
void setADCMux(int motor,int number){
    switch(motor){
    case 1:
        switch(number){
        case 0:
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_0,0);
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_1,0);
            break;
        case 1:
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_0,GPIO_PIN_0);
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_1,0);
            break;
        case 2:
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_0,0);
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_1,GPIO_PIN_1);
            break;
        case 3:
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_0,GPIO_PIN_0);
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_1,GPIO_PIN_1);
            break;
        }
    case 2:
        switch(number){
        case 0:
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_2,0);
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_3,0);
            break;
        case 1:
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_2,GPIO_PIN_2);
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_3,0);
            break;
        case 2:
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_2,0);
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_3,GPIO_PIN_3);
            break;
        case 3:
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_2,GPIO_PIN_2);
            GPIOPinWrite(GPIO_PORTM_BASE,GPIO_PIN_3,GPIO_PIN_3);
            break;
        }
    }
}
