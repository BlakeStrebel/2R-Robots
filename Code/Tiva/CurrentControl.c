#include "r2r.h"
#include "CurrentControl.h"
#include "PositionControl.h"
#include "Motor.h"
#include "Utilities.h"
#include <stdio.h>

#define SWITCHDELAY 30              // Number of Cycles to Delay updating current on state switch
#define MA_PER_COUNT 5.75474330357  // Conversion between ADC counts and mA

// Voltage offsets for the motor phases in counts
static volatile int motorOffsetA[3] = {0};
static volatile int motorOffsetB[3] = {0};

// Current sensing
static volatile uint32_t motorCountsA[3] = {0};
static volatile uint32_t motorCountsB[3] = {0};

static volatile int actualCurrent[3] = {0};
static volatile int refCurrent[3] = {0};

// Current control data
static volatile int currentE[3] = {0};
static volatile int currentEint[3] = {0};
static volatile float Kp[3] = {0, 5, 5};
static volatile float Ki[3] = {0, 0.15, 0.15};

static volatile int motorPWM[3] = {0};

// Initialize ADC0 to read phase currents for each motor using fixed frequency interrupt
void currentControlInit(void)
{
    IntMasterDisable();

    // Peripheral Enable
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);

    // Configure the ADC to use PLL at 480 MHz divided by 20 to get an ADC clock of 20MHz
    // Note: this must be between 16 and 32 MHz for TM4C129x devices
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, 24);

    // Configure the ADC to use 2x hardware averaging of ADC samples
    //ADCHardwareOversampleConfigure(ADC0_BASE, 2);

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
    ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH0 | ADC_CTL_IE |
                                              ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 0);

    IntPrioritySet(INT_ADC0SS0, 0x40);   // Set ADC interrupt to second highest priority

    // Configure ADC timer
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);

    // Read ADC at 2,000Hz, current control runs at this frequency
    TimerLoadSet(TIMER2_BASE, TIMER_A, ui32SysClock / 2000);
    TimerControlTrigger(TIMER2_BASE, TIMER_A, true);
    TimerEnable(TIMER2_BASE, TIMER_A);

    // Enable Interrupts
    //ADCIntEnable(ADC0_BASE, 0x00);
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
            motorControlPWM(1, 0);
            motorControlPWM(2, 0);
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
            currentPIController(1, refCurrent[1], actualCurrent[1]); // Track reference signal
            currentPIController(2, refCurrent[2], actualCurrent[2]);
            break;
        }
        case TRACK:
        {
            currentPIController(1, refCurrent[1], actualCurrent[1]); // Track reference signal
            currentPIController(2, refCurrent[2], actualCurrent[2]);
            break;
        }
        case ICALIB:
        {
            if (i >= 1000)  // Done sensing when index equals number of samples
            {
                // Find average deviation from zero current and apply as offset
                motorOffsetA[1] = -1*m1_A_calib/1000;
                motorOffsetB[1] = -1*m1_B_calib/1000;
                motorOffsetA[2] = -1*m2_A_calib/1000;
                motorOffsetB[2] = -1*m2_B_calib/1000;

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
                m1_A_calib = m1_A_calib + motorCountsA[1] - 2047;
                m1_B_calib = m1_B_calib + motorCountsB[1] - 2047;
                m2_A_calib = m2_A_calib + motorCountsA[2] - 2047;
                m2_B_calib = m2_B_calib + motorCountsB[2] - 2047;
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
                    //buffer_write(refCurrent[1], actualCurrent[1], refCurrent[2], actualCurrent[2]);
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
                refCurrent[1] = 200;    // 200 count amplitude (~1150mA)
                refCurrent[2] = 200;
            }
            else if (i % 150 == 0)
            {
                refCurrent[1] = -refCurrent[1];
                refCurrent[2] = -refCurrent[2];
            }

            if (i == getN())  // Done tracking when index equals number of samples
            {
                setMODE(IDLE);
                refCurrent[1] = 0;
                refCurrent[2] = 0;
                motorControlPWM(1, 0);
                motorControlPWM(2, 0);
                i = 0;
                reset_current_error();
            }
            else
            {
                currentPIController(1, refCurrent[1], actualCurrent[1]); // Track reference signal
                currentPIController(2, refCurrent[2], actualCurrent[2]);

                i++; //increment data

                // Handle data decimation
                decctr++;
                if (decctr == DECIMATION)
                {
                    //buffer_write(refCurrent[1], actualCurrent[1], refCurrent[2], actualCurrent[2]);
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
                currentPIController(1, refCurrent[1], actualCurrent[1]); // Track reference signal
                currentPIController(2, refCurrent[2], actualCurrent[2]);

                i++; //increment data

                // Handle data decimation
                decctr++;
                if (decctr == DECIMATION)
                {
                    //buffer_write(refCurrent[1], actualCurrent[1], refCurrent[2], actualCurrent[2]);
                    decctr = 0; // reset decimation counter
                }
            }
            break;
        }
    }
}

// Calculate motor pwm using current error
void currentPIController(int motor, int reference, int actual)
{
    currentE[motor] = reference - actual;
    currentEint[motor] = currentEint[motor] + currentE[motor];
    motorPWM[motor] = Kp[motor]*currentE[motor] + Ki[motor]*currentEint[motor];
    motorPWM[motor] = boundInt(motorPWM[motor], PWMPERIOD);
    motorControlPWM(motor, motorPWM[motor]);
}

// Print current control gains over UART
void get_current_gains(int motor)
{
    char buffer[25];
    sprintf(buffer, "%f\r\n",Kp[motor]);
    //UART0write(buffer);
    sprintf(buffer, "%f\r\n",Ki[motor]);
    //UART0write(buffer);
}

// Set current control gains over UART
void set_current_gains(int motor)
{
    char buffer[25];
    float Kptemp, Kitemp;
    //UART0read(buffer,25);                       // Store gains in buffer
    sscanf(buffer, "%f %f",&Kptemp, &Kitemp);   // Extract gains to temporary variables
    IntMasterDisable();                         // Disable interrupts briefly
    Kp[motor] = Kptemp;                                // Set gains
    Ki[motor] = Kitemp;
    reset_current_error();
    IntMasterEnable();                          // Re-enable interrupts
}

// Set reference current for motor current controller
void setCurrent(int motor, int u)
{
    refCurrent[motor] = u;
}

// Return motor current
int getCurrent(int motor)
{
    return actualCurrent[motor];
}

// Return motor PWM
int getPWM(int motor)
{
    return motorPWM[motor];
}

// Reset error being used for current control
void reset_current_error(void)
{
    currentE[1] = 0;
    currentE[2] = 0;
    currentEint[1] = 0;
    currentEint[2] = 0;
}

// Process phase voltages to update motor currents in counts
void counts_read(void)
{
    static int M1_A, M1_B, M2_A, M2_B;
    static int M1_current_prev = 0, M2_current_prev = 0;
    static int M1_hallstate = 0, M1_hallstate_prev = 0, M1_hallstate_cntr = 0;
    static int M2_hallstate = 0, M2_hallstate_prev = 0, M2_hallstate_cntr = 0;

    // Offset phase currents to be centered at zero using calibration data
    IntMasterDisable();
    M1_A = (int)motorCountsA[1] - 2047 + motorOffsetA[1];
    M1_B = (int)motorCountsB[1] - 2047 + motorOffsetB[1];
    M2_A = (int)motorCountsA[2] - 2047 + motorOffsetA[2];
    M2_B = (int)motorCountsB[2] - 2047 + motorOffsetB[2];
    IntMasterEnable();

    // Update motor 1 current and direction
    M1_hallstate = getmotorHALLS(1);
    switch (M1_hallstate)
    {
        case (GPIO_PIN_0 | GPIO_PIN_1):
            actualCurrent[1] = -1*M1_B;
            break;
        case GPIO_PIN_0:
            actualCurrent[1] = -1*M1_A;
            break;
        case (GPIO_PIN_0 | GPIO_PIN_2):
            actualCurrent[1] = M1_B;
            break;
        case GPIO_PIN_2:
            actualCurrent[1] = M1_B;
            break;
        case (GPIO_PIN_1 | GPIO_PIN_2):
            actualCurrent[1] = M1_A;
            break;
        case GPIO_PIN_1:
            actualCurrent[1] = M1_A;
            break;
    }

    // Smooth voltage spikes when commutation state changes
    if (M1_hallstate != M1_hallstate_prev)
    {
        // wait SWITCHDELAY control cycles before updating current
        if (M1_hallstate_cntr < SWITCHDELAY)
        {
            actualCurrent[1] = M1_current_prev;
            M1_hallstate_cntr++;
        }
        else
        {
            M1_hallstate_prev = M1_hallstate;
            M1_hallstate_cntr = 0;
        }
    }

    M1_current_prev = actualCurrent[1];    // Update previous current value

    // Update motor 2 current and direction
    M2_hallstate = getmotorHALLS(2);
    switch (M2_hallstate)
    {
        case (GPIO_PIN_0 | GPIO_PIN_1):
            actualCurrent[2] = -1*M2_B;
            break;
        case GPIO_PIN_0:
            actualCurrent[2] = -1*M2_A;
            break;
        case (GPIO_PIN_0 | GPIO_PIN_2):
            actualCurrent[2] = M2_B;
            break;
        case GPIO_PIN_2:
            actualCurrent[2] = M2_B;
            break;
        case (GPIO_PIN_1 | GPIO_PIN_2):
            actualCurrent[2] = M2_A;
            break;
        case GPIO_PIN_1:
            actualCurrent[2] = M2_A;
            break;
    }

    // Smooth voltage spikes when commutation state changes
    if (M2_hallstate != M2_hallstate_prev)
    {
        // wait SWITCHDELAY control cycles before updating current
        if (M2_hallstate_cntr < SWITCHDELAY)
        {
            actualCurrent[2] = M2_current_prev;
            M2_hallstate_cntr++;
        }
        else
        {
            M2_hallstate_prev = M2_hallstate;
            M2_hallstate_cntr = 0;
        }
    }

    M2_current_prev = actualCurrent[2];    // Update previous current value
}

// Read AD0 containing raw phase voltages measured across shunt resistor
void AD0_read(int mux)
{
    static uint32_t TEMP[8]={0};

    // Extract ADC values into array
    ADCSequenceDataGet(ADC0_BASE, 0, TEMP);

    // Update Global COUNTS
    IntMasterDisable();
    motorCountsA[1] = TEMP[0];
    motorCountsB[1] = TEMP[1];
    motorCountsA[2] = TEMP[2];
    motorCountsB[2] = TEMP[3];
    IntMasterEnable();
}

// Print motor current in mA
void get_mA(void)
{
    char buffer[10];
    int M1_current_mA, M2_current_mA;

    // Convert counts to mA
    IntMasterDisable();
    M1_current_mA = actualCurrent[1]*MA_PER_COUNT;
    M2_current_mA = actualCurrent[2]*MA_PER_COUNT;
    IntMasterEnable();

    sprintf(buffer, "%d %d\r\n", M1_current_mA, M2_current_mA);
    //UART0write(buffer);
}

// Print motor current in counts
void get_counts(void)
{
    char buffer[10];
    int M1_current_counts, M2_current_counts;

    IntMasterDisable();
    M1_current_counts = actualCurrent[1];
    M2_current_counts = actualCurrent[2];
    IntMasterEnable();

    sprintf(buffer, "%d %d\r\n", M1_current_counts, M2_current_counts);
    //UART0write(buffer);
}

