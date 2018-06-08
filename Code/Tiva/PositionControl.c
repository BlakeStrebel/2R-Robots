#include "r2r.h"
#include "PositionControl.h"
#include "CurrentControl.h"
#include "Utilities.h"
#include "Encoder.h"
#include "Motor.h"
#include "math.h"

#define M_PI 3.1415926

// PID gains
static volatile float Kp = 0.5;
static volatile float Ki = 0;
static volatile float Kd = 0.5;

// PID gains
static volatile PID_gains PID[3];

// Control info
static volatile control_error E[3];

// Decogging state
static volatile int DECOGGING = 0;

/*
 * This function sets up the timer interrupt used for motor control
 *
 * Comes after:
 * - sysInit()
 */
void positionControlInit(void){
    setMODE(IDLE);
    IntMasterDisable();
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER1_BASE, TIMER_A, ui32SysClock / 1000);    // Set control frequency to 1kHz
    IntPrioritySet(INT_TIMER1A, 0xE0);                          // Lowest priority
    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER1_BASE, TIMER_A);
    IntMasterEnable();

}

// Set new desired angle for motor
// takes angle in counts
void set_desired_angle(int angle, int motor)
{
    IntMasterDisable();
    E[motor].desired = angle;
    IntMasterEnable();
}

int get_desired_angle(int motor)
{
    return E[motor].desired;
}

void Timer1IntHandler(void)
{
    static int decctr = 0;  // counter for data decimation
    static int i = 0;   // trajectory index

    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT); // clear the interrupt flag

    encoderRead(1); // update encoder values
    encoderRead(2); // update encoder values

    switch(getMODE())
    {
        case PID1:
            PID_Controller(E[1].traj[i] / 2 / M_PI * 16384, readMotorRawRelative(1), 1, 0);    // motor1 control
        case READ1:
            if (i >= getN())
            {
                i = 0;
                setMODE(IDLE);
            }
            decctr++;
            if (decctr == DECIMATION)
            {
                UART0FloatPut(readMotorRadRelative(1));
                decctr = 0; // reset decimation counter
            }
            i++;
            break;

        case PID2:
            PID_Controller(E[2].traj[i] / 2 / M_PI * 16384, readMotorRawRelative(2) - readMotorRawRelative(1), 2, 0);    // motor2 control
        case READ2:
            if (i >= getN())
            {
                i = 0;
                setMODE(IDLE);
            }
            decctr++;
            if (decctr == DECIMATION)
            {
                UART0FloatPut(readMotorRadRelative(2) - readMotorRadRelative(1));
                decctr = 0; // reset decimation counter
            }
            i++;
            break;

        case PIDb:
            PID_Controller(E[1].traj[i] / 2 / M_PI * 16384, readMotorRawRelative(1), 1, 0);    // motor1 control
            PID_Controller(E[2].traj[i] / 2 / M_PI * 16384, readMotorRawRelative(2) - readMotorRawRelative(1), 2, 0);    // motor2 control
        case READb:
            if (i >= getN())
            {
                i = 0;
                setMODE(IDLE);
            }
            decctr++;
            if (decctr == DECIMATION)
            {
                UART0FloatPut(readMotorRadRelative(1));
                UART0FloatPut(readMotorRadRelative(2) - readMotorRadRelative(1));
                decctr = 0; // reset decimation counter
            }
            i++;
            break;


/*

        case HOLD:
        {
            PID_Controller(E[1].desired, E[1].actual, MOTOR1);    // motor1 control
            PID_Controller(E[2].desired, E[2].actual, MOTOR2);    // motor2 control
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
                E[1].desired = get_refPos(i, 1);
                E[2].desired = get_refPos(i, 2);
                PID_Controller(E[1].desired, E[1].actual, MOTOR1);    // motor1 control
                PID_Controller(E[2].desired, E[2].actual, MOTOR2);    // motor2 control

                i++;    // increment index

                // Handle data decimation
                decctr++;
                if (decctr == DECIMATION)
                {
                    buffer_write(E[1].actual, E[2].actual, E[1].u, E[2].u);
                    decctr = 0; // reset decimation counter
                }
            }
            break;
        }
        */
    }
}

// Calculate control effort and set value to control motor
void PID_Controller(int reference, int actual, int motor, int default_gains)
{
    static float u;

    E[motor].Enew = reference - actual;               // Calculate error
    E[motor].Eint = E[motor].Eint + E[motor].Enew;                // Calculate intergral error
    E[motor].Edot = E[motor].Enew - E[motor].Eold;                // Calculate derivative error
    E[motor].Eold = E[motor].Enew;                          // Update old error

    if (default_gains)
        u = defaultKp[motor] * E[motor].Enew + defaultKi[motor] * E[motor].Eint + defaultKd[motor] * E[motor].Edot;   // Calculate effort
    else
        u = PID[motor].Kp * E[motor].Enew + PID[motor].Ki * E[motor].Eint + PID[motor].Kd * E[motor].Edot;   // Calculate effort

    if (DECOGGING)                              // Add decogging control
        u += decog_motor(readMotorCounts(motor), motor);

    // Max effort
    u = boundInt(u, 2047);    // Bound max/min effort
    setCurrent(motor, u);        // Set desired motor current
}

// Reset error for all of the motors
void reset_controller_error(void)
{
    E[1].Eold = 0;
    E[1].Enew = 0;
    E[1].Eint = 0;
    E[1].Edot = 0;

    E[2].Eold = 0;
    E[2].Enew = 0;
    E[2].Eint = 0;
    E[2].Edot = 0;
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

void setDecogging(int decog) // Turn motor decogging on/off
{
    DECOGGING = decog;
}

void loadPositionTrajectory(int motor)
{
    int i;
    for (i = 0; i < getN(); i++)
        E[motor].traj[i] = UART0FloatGet();
}

void setPositionPID(int motor)   // recieve position control gains
{
    PID[motor].Kp = UART0FloatGet();
    PID[motor].Ki = UART0FloatGet();
    PID[motor].Kd = UART0FloatGet();
}
