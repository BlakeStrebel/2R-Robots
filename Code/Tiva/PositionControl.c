#include "r2r.h"
#include "PositionControl.h"
#include "CurrentControl.h"
#include "Utilities.h"
#include "Encoder.h"
#include "Motor.h"
#include <math.h>

// PID gains
static volatile float Kp = 0;
static volatile float Ki = 0;
static volatile float Kd = 0;

// Control info
static volatile control_data_t M1;
static volatile control_data_t M2;
static volatile control_error E1;
static volatile control_error E2;

// Decogging state
static volatile int DECOGGING = 0;

/*
 * This function sets up the timer interrupt used for motor control
 *
 * Comes after:
 * - sysInit()
 */
void MotorTimerInit(void){
    setMODE(IDLE);
    IntMasterDisable();
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER1_BASE, TIMER_A, ui32SysClock/1000-1);    // Set control frequency to 1kHz
    IntPrioritySet(INT_TIMER1A, 0xE0);                          // Lowest priority
    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER1_BASE, TIMER_A);
    IntMasterEnable();

    E1.u = 0;
    E2.u = 0;
}

void get_position_gains(void)   // Provide position control gains
{
    char buffer[25];
    sprintf(buffer, "%f\r\n",Kp);
    UART0write(buffer);
    sprintf(buffer, "%f\r\n",Ki);
    UART0write(buffer);
    sprintf(buffer, "%f\r\n",Kd);
    UART0write(buffer);
}

void set_position_gains(void)   // Receive position control gains
{
    char buffer[25];
    float Kptemp, Kitemp, Kdtemp;
    UART0read(buffer,25);                              // Store gains in buffer
    sscanf(buffer, "%f %f %f",&Kptemp, &Kitemp, &Kdtemp);   // Extract gains to temporary variables
    IntMasterDisable();                         // Disable interrupts briefly
    Kp = Kptemp;                                            // Set gains
    Ki = Kitemp;
    Kd = Kdtemp;
    reset_controller_error();
    IntMasterEnable();                          // Re-enable interrupts
}

// Set new desired angle for motor
// takes angle in counts
void set_desired_angle(int angle, int motor)
{
    IntMasterDisable();
    if (motor == 1)
    {
        E1.desired = angle;
    }
    else if (motor == 2)
    {
        E2.desired = angle;
    }
    IntMasterEnable();
}

int get_desired_angle(int motor)
{
    int angle;
    if (motor == 1)
    {
        angle = E1.desired;
    }
    else if (motor == 2)
    {
        angle = E2.desired;
    }

    return angle;
}

void load_position_trajectory(int motor)      // Load trajectory for tracking
{
    int i, n, data;
    char buffer[10];
    setN();         // Receive number of samples from client
    n = getN();     // Determine number of samples

    for (i = 0; i < n; i++)
    {
        UART0read(buffer,10);           // Read reference position from client
        sscanf(buffer,"%d",&data);      // Store position in data
        write_refPos(data, i, motor);   // Write data to reference position array
    }
}

void Timer1IntHandler(void)
{
    static int decctr = 0;  // counter for data decimation
    static int i = 0;   // trajectory index

    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT); // clear the interrupt flag

    encoderRead(); // update encoder values
    E1.actual = readMotor1RawRelative(); // read positions
    E2.actual = readMotor2RawRelative();
    E1.raw = readMotor1Raw();
    E2.raw = readMotor2Raw();

    switch(getMODE())
    {
        case IDLE:
        {
            break;
        }
        case PWM:
        {
            break;
        }
        case ITEST:
        {
            break;
        }
        case HOLD:
        {
            PID_Controller(E1.desired, E1.actual, MOTOR1);    // motor1 control
            //PID_Controller(E2.desired, E2.actual, MOTOR2);    // motor2 control TODO: here
            break;

        }
        case TRACK:
        {
            if (i == getN())    // Done tracking when index equals number of samples
            {
                i = 0; // reset index
                setMODE(IDLE);  // Hold final position, Could set PWM to zero instead
            }
            else
            {
                E1.desired = get_refPos(i, 1);
                E2.desired = get_refPos(i, 2);
                PID_Controller(E1.desired, E1.actual, 1);    // motor1 control
                //PID_Controller(E2.desired, E2.actual, 2);    // motor2 control TODO: test

                i++;    // increment index

                // Handle data decimation
                decctr++;
                if (decctr == DECIMATION)
                {
                    buffer_write(E1.actual, E2.actual, E1.u, E2.u);   //TODO: uncomment this
                    //buffer_write(E1.actual, E1.actual, E1.u, getPWM(MOTOR1)); //getCurrent(MOTOR1)s
                    decctr = 0; // reset decimation counter
                }
            }
            break;
        }
    }
}

// Calculate control effort and set value to control motor
void PID_Controller(int reference, int actual, int motor)
{
    if (motor == 1)
    {
        E1.Enew = reference - actual;               // Calculate error
        E1.Eint = E1.Eint + E1.Enew;                // Calculate intergral error
        E1.Edot = E1.Enew - E1.Eold;                // Calculate derivative error
        E1.Eold = E1.Enew;                          // Update old error
        E1.u = Kp*E1.Enew + Ki*E1.Eint + Kd*E1.Edot;   // Calculate effort

        if (DECOGGING)                              // Add decogging control
        {
            E1.u = E1.u + decog_motor(E1.raw, 1);
        }

        E1.u = boundInt(E1.u, 1500);
        setCurrent(motor, E1.u);
    }
    else if (motor == 2)
    {
        E2.Enew = reference - actual;               // Calculate error
        E2.Eint = E2.Eint + E2.Enew;                // Calculate intergral error
        E2.Edot = E2.Enew - E2.Eold;                // Calculate derivative error
        E2.Eold = E2.Enew;                          // Update old error
        E2.u = Kp*E2.Enew + Ki*E2.Eint + Kd*E2.Edot;   // Calculate effort

        if (DECOGGING)                              // Add decogging control
        {
            E2.u = E2.u + decog_motor(E2.raw, 2);
        }

        E2.u = boundInt(E2.u, 1500);
        setCurrent(motor, E2.u);
    }
}

// Reset error for all of the motors
void reset_controller_error(void)
{
    E1.Eold = 0;
    E1.Enew = 0;
    E1.Eint = 0;
    E1.Edot = 0;

    E2.Eold = 0;
    E2.Enew = 0;
    E2.Eint = 0;
    E2.Edot = 0;
}

float decog_motor(int x, int motor)
{
    float u = 0;
    if (motor == 1)
    {
        u = 0;
    }
    else if (motor == 2)
    {
        u = 20.4732*cos(0.00921288*x - 2.68484) + 14.1249*cos(0.00460644*x - 1.3871) + 21.0256*cos(0.00307096*x - 2.93648); // + 66.7217
    }

    return u;
}

void setDecogging(void) // Turn motor decogging on/off
{
    char buffer[10]; int decog;
    sscanf(buffer,"%d",&decog);
    DECOGGING = decog;
}
