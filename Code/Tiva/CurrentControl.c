#include "System.h"
#include "r2r.h"
#include "CurrentControl.h"
#include "PositionControl.h"
#include "Motor.h"
#include "Utilities.h"
#include "Adc.h"
#include <stdio.h>

// Voltage offsets for the motor phases in counts
#define M1_A_OFFSET 25
#define M1_B_OFFSET 36
#define M2_A_OFFSET 12
#define M2_B_OFFSET 63

static volatile uint32_t M1_A_COUNTS, M1_B_COUNTS, M2_A_COUNTS, M2_B_COUNTS;
static volatile int MOTOR1CURRENT = 0, MOTOR1REF = 0;
static volatile int MOTOR2CURRENT = 0, MOTOR2REF = 0;

static volatile int E1 = 0, Eint1 = 0, E2 = 0, Eint2 = 0, PWM1 = 0, PWM2 = 0;
static volatile float Kp = 4, Ki = 0.1;

void currentControlInit(void){

    IntMasterDisable();

    // Initialize ADC mux
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    setADCMux(1,0); // Initialize mux to sample Phase A
    setADCMux(2,0);

    // Peripheral Enable
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    //GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0); Used to time ADC
    //GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, GPIO_PIN_0);

    // Wait for the ADC0 module to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)) {;}

    // Configure the ADC to use PLL at 480 MHz divided by 20 to get an ADC clock of 24MHz
    // Note: this must be between 16 and 32 MHz for TM4C129x devices
    ADCClockConfigSet(ADC0_BASE,ADC_CLOCK_SRC_PLL | ADC_CLOCK_RATE_FULL, 20);

    // Configure ADC pins

    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0); // AIN3 (Motor 2)
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1); // AIN2 (Motor 1)


    // Configure the ADC to use 4x hardware averaging of ADC samples

    ADCHardwareOversampleConfigure(ADC0_BASE, 4);


    // Configure the ADC to trigger at timer2 frequency
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_TIMER, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH2);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH3| ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 0);
    IntPrioritySet(INT_ADC0SS0, 0x00);   // Set ADC interrupt to highest priority

    // Configure ADC timer
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    int samplePeriod = ui32SysClock/60000;  // Read ADC at 60,000Hz, current control runs at half this frequency
    TimerLoadSet(TIMER2_BASE, TIMER_A, samplePeriod-1);
    TimerControlTrigger(TIMER2_BASE, TIMER_A, true);
    TimerEnable(TIMER2_BASE, TIMER_A);

    // Enable Interrupts
    ADCIntEnable(ADC0_BASE, 0x00); // First priority
    IntEnable(INT_ADC0SS0);
    IntMasterEnable();
}

void CurrentControlIntHandler(void)
{
    static int i = 0, decctr = 0, ready = 0, mux = 0;

    ADCIntClear(ADC0_BASE, 0); // Clear interrupt  flag

    // Update ADC values for muxed phase
    AD0_read(mux);

    // Update Mux for the next read
    mux = !mux;
    setADCMux(1,mux);
    setADCMux(2,mux);

    // Wait until we've sampled both phases to do control
    if (ready)
    {
        counts_read();  // Update motor current from phase currents

        switch(getMODE())
        {
            case IDLE:
            {
                // Zero control effort
                //set_motor_pwm(MOTOR1, 0);
                //set_motor_pwm(MOTOR2, 0);
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
                        //buffer_write(M1_A_COUNTS - 2047 + M1_A_OFFSET, M1_B_COUNTS - 2047 + M1_B_OFFSET, M2_A_COUNTS - 2047 + M2_A_OFFSET, M2_B_COUNTS - 2047 + M2_B_OFFSET);

                        //buffer_write(M2_A_COUNTS - 2047 + M2_A_OFFSET, M2_B_COUNTS - 2047 + M2_B_OFFSET, MOTOR2CURRENT, MOTOR2CURRENT);   // Write current values to buffer

                        buffer_write(MOTOR1REF, MOTOR1CURRENT, MOTOR2REF, MOTOR2CURRENT);

                        //buffer_write(COUNTS[0] - 2047 , COUNTS[1] - 2047, COUNTS[2] - 2047, COUNTS[3] - 2047);
                        //buffer_write(COUNTS[0] - 2047 + M1_A_OFFSET, COUNTS[1] - 2047 + M1_B_OFFSET, getmotor1HALLS(), 0);
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
    }

    ready = !ready;

}

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
    static int M1_A, M1_B, M2_A, M2_B;
    static int M1_current_prev = 0, M2_current_prev = 0;
    static int32_t M1_hallstate = 0, M1_hallstate_prev = 0, M1_hallstate_cntr = 0;
    static int32_t M2_hallstate = 0, M2_hallstate_prev = 0, M2_hallstate_cntr = 0;

    // Offset phase currents to be centered at zero
    IntMasterDisable();
    M1_A = (int)M1_A_COUNTS - 2047 + M1_A_OFFSET;
    M1_B = (int)M1_B_COUNTS - 2047 + M1_B_OFFSET;
    M2_A = (int)M2_A_COUNTS - 2047 + M2_A_OFFSET;
    M2_B = (int)M2_B_COUNTS - 2047 + M2_B_OFFSET;
    IntMasterEnable();

    // Update motor 1 current and direction
    M1_hallstate = getmotor1HALLS();
    if (M1_hallstate == M1_HALLSTATE_1 || M1_hallstate == M1_HALLSTATE_2 || M1_hallstate == M1_HALLSTATE_3)
    {
        if (M1_A > M1_B)
        {
            MOTOR1CURRENT = maxInt(abs(M1_A),abs(M1_B));    // Positive current (CW)

        }
        else
        {
            MOTOR1CURRENT = -1*maxInt(abs(M1_A),abs(M1_B)); // Negative current (CCW)
        }
    }
    else
    {
        if (M1_A > M1_B)
        {
            MOTOR1CURRENT = -1*maxInt(abs(M1_A),abs(M1_B)); // Negative current (CCW)
        }
        else
        {
            MOTOR1CURRENT = maxInt(abs(M1_A),abs(M1_B));    // Positive current (CW)
        }
    }

    // Add some hysteresis to smooth current spikes when commutation state changes
    if (M1_hallstate != M1_hallstate_prev)
    {
        // wait 5 control cycles before updating current
        if (M1_hallstate_cntr < 5)
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
    if (M2_hallstate == M2_HALLSTATE_1 || M2_hallstate == M2_HALLSTATE_2 || M2_hallstate == M2_HALLSTATE_3)
    {
        if (M2_A > M2_B)
        {
            MOTOR2CURRENT = maxInt(abs(M2_A),abs(M2_B));    // Positive current (CW)

        }
        else
        {
            MOTOR2CURRENT = -1*maxInt(abs(M2_A),abs(M2_B)); // Negative current (CCW)
        }
    }
    else
    {
        if (M2_A > M2_B)
        {
            MOTOR2CURRENT = -1*maxInt(abs(M2_A),abs(M2_B)); // Negative current (CCW)
        }
        else
        {
            MOTOR2CURRENT = maxInt(abs(M2_A),abs(M2_B));    // Positive current (CW)
        }
    }

    // Add some hysteresis to smooth current spikes when commutation state changes
    if (M2_hallstate != M2_hallstate_prev)
    {
        // wait 5 control cycles before updating current
        if (M2_hallstate_cntr < 5)
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

// Read AD0
void AD0_read(int mux)
{
    static uint32_t TEMP[8]={0};

    // Extract ADC values into array
    ADCSequenceDataGet(ADC0_BASE, 0, TEMP);

    if (mux == 0)   // mux 1 reads Phase A
    {
        // Update Global COUNTS
        IntMasterDisable();
        M1_A_COUNTS = TEMP[0];
        M2_A_COUNTS = TEMP[1];
        IntMasterEnable();
    }
    else            // mux 2 reads Phase B
    {
        // Update Global COUNTS
        IntMasterDisable();
        M1_B_COUNTS = TEMP[0];
        M2_B_COUNTS = TEMP[1];
        IntMasterEnable();
    }
}

// Print motor current in mA
void get_mA(void)
{
    /*

     char buffer[50];
    int M1_A, M1_B, M2_A, M2_B;

    // Convert counts to mA
    IntMasterDisable();
    M1_A = abs(COUNTS[0] * 5.7547431 - 11428.5712);
    M1_B = abs(COUNTS[1] * 5.7547431 - 11428.5712);
    M2_A = abs(COUNTS[2] * 5.7547431 - 11428.5712);
    M2_B = abs(COUNTS[3] * 5.7547431 - 11428.5712);
    IntMasterEnable();

    sprintf(buffer, "%d %d %d %d\r\n", M1_A, M1_B, M2_A, M2_B);
    UART0write(buffer);
*/
}

// Print motor current in counts
void get_counts(void)
{
    char buffer[100];
    int M1_A, M1_B, M2_A, M2_B;

    IntMasterDisable();
    M1_A = M1_A_COUNTS - 2047 + M1_A_OFFSET;
    M1_B = M1_B_COUNTS - 2047 + M1_B_OFFSET;
    M2_A = M2_A_COUNTS - 2047 + M2_A_OFFSET;
    M2_B = M2_B_COUNTS - 2047 + M2_B_OFFSET;
    IntMasterEnable();

    sprintf(buffer, "%d %d %d %d\r\n", M1_A, M1_B, M2_A, M2_B);
    UART0write(buffer);
}

