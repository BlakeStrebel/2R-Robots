#include "PositionControl.h"
#include "System.h"
#include "CurrentControl.h"
#include "Utilities.h"
#include "Encoder.h"
#include "Motor.h"
#include <math.h>

// PID gains
static volatile PID_gains PID[3];

static volatile float Kp = 0;
static volatile float Ki = 0;
static volatile float Kd = 0;


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
<<<<<<< HEAD:Code/Tiva/Control.c
void timerIntInit(void)
{
    setMODE(IDLE);
    IntMasterDisable();

    // Enable the Timer0 peripheral.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); // Use timer 1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2); // Use timer 2

    // Configure Timer1A and Timer2A
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);

    // Set the count time for timers
    TimerLoadSet(TIMER1_BASE, TIMER_A, ui32SysClock / 1000); // Use timer 1A 1KHz.
    TimerLoadSet(TIMER2_BASE, TIMER_A, ui32SysClock / 5000); // Use timer 2A 5KHz.

    // Setup the interrupts for the timer timeouts.
    IntEnable(INT_TIMER1A);
    IntEnable(INT_TIMER2A);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    // Enable the timers.
    TimerEnable(TIMER1_BASE, TIMER_A);
    TimerEnable(TIMER2_BASE, TIMER_A);

    // Set the INT_TIMER1A interrupt priority to the lowest priority.
    IntPrioritySet(INT_TIMER1A, 0xE0);
    // Set the INT_TIMER2A interrupt priority to the highest priority.
    IntPrioritySet(INT_TIMER2A, 0);

    IntMasterEnable();

void MotorTimerInit(void){
    setMODE(IDLE);
    IntMasterDisable();
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER1_BASE, TIMER_A, ui32SysClock/1000-1);    // Set control frequency to 1kHz
    IntPrioritySet(INT_TIMER1A, 0x40); // Second highest priority
    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER1_BASE, TIMER_A);
    IntMasterEnable();

    E1.u = 0;
    E2.u = 0;
}

void get_position_gains(void)   // provide position control gains
{
    //char buffer[25];
    //sprintf(buffer, "%f\r\n",Kp);
    //UART0write(buffer);
    //sprintf(buffer, "%f\r\n",Ki);
    //UART0write(buffer);
    //sprintf(buffer, "%f\r\n",Kd);
    //UART0write(buffer);
}

void set_position_gains(void)   // recieve position control gains
{
    char buffer[25];
    float Kptemp, Kitemp, Kdtemp;
    UART0read(buffer,25);                              // Store gains in buffer
    sscanf(buffer, "%f %f %f",&Kptemp, &Kitemp, &Kdtemp);   // Extract gains to temporary variables
    IntMasterDisable();                         // Disable interrupts briefly
    PID[1].Kp = Kptemp;                                            // Set gains
    PID[1].Ki = Kitemp;
    PID[1].Kd = Kdtemp;
    reset_controller_error();
    IntMasterEnable();                          // Re-enable interrupts
}

void setPositionPID(int motor)   // recieve position control gains
{
    PID[motor].Kp = UART0FloatGet();
    PID[motor].Ki = UART0FloatGet();
    PID[motor].Kd = UART0FloatGet();
    reset_controller_error();
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

int get_motor_pwm(int motor)
{
    //return E[motor].u;
    return 0;
}

void set_motor_pwm(int motor, int value)    // TODO this function is repetative, delete it
{
    if (motor == 1)
    {
        motor1ControlPWM(value);
        E1.u = value;
    }
    else if (motor == 2)
    {
        motor2ControlPWM(value);
        E2.u = value;
    }
}

void Timer1IntHandler(void)
{
    static int decctr = 0;  // counter for data decimation
    static int i = 0;   // trajectory index


    encoderRead(1); // update encoder values
    encoderRead(2); // update encoder values


    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT); // clear the interrupt flag

    encoderRead(); // update encoder values
    E1.actual = readMotor1RawRelative(); // read positions
    E2.actual = readMotor2RawRelative();
    E1.raw = readMotor1Raw();
    E2.raw = readMotor2Raw();

    switch(getMODE())
    {
        case IDLE:
            break;
        case PID1:
            PID_Controller(E[1].traj[i] / 2 / M_PI * 16384, readMotorRawRelative(1), 1, 0);    // motor1 control
        case READ1:
            decctr++;
            if (decctr == DECIMATION)
            {
                UART0FloatPut(2 * M_PI * readMotorRawRelative(1) / 16384);
                decctr = 0; // reset decimation counter
            }
            i++;
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
            PID_Controller(E1.desired, E1.actual, 1);    // motor1 control
            PID_Controller(E2.desired, E2.actual, 2);    // motor2 control
            break;
        case PID2:
            PID_Controller(E[2].traj[i] / 2 / M_PI * 16384, readMotorRawRelative(2) - readMotorRawRelative(1), 2, 0);    // motor2 control
        case READ2:
            decctr++;
            if (decctr == DECIMATION)
            {
                UART0FloatPut(2 * M_PI * (readMotorRawRelative(2) - readMotorRawRelative(1)) / 16384);
                decctr = 0; // reset decimation counter
            }
            i++;
            break;
        case PIDb:
            //GPIOPinWrite(GPIO_PORTP_BASE,GPIO_PIN_3,GPIO_PIN_3);
            PID_Controller(E[1].traj[i] / 2 / M_PI * 16384, readMotorRawRelative(1), 1, 0);    // motor1 control
            PID_Controller(E[2].traj[i] / 2 / M_PI * 16384, readMotorRawRelative(2) - readMotorRawRelative(1), 2, 0);    // motor2 control
        case READb:
            decctr++;
            if (decctr == DECIMATION)
            {
                UART0FloatPut(2 * M_PI * readMotorRawRelative(1) / 16384);
                UART0FloatPut(2 * M_PI * (readMotorRawRelative(2) - readMotorRawRelative(1)) / 16384);
                decctr = 0; // reset decimation counter
            }
            i++;
            //GPIOPinWrite(GPIO_PORTP_BASE,GPIO_PIN_3,GPIO_PIN_3);
            break;
        case HOLD:
            PID_Controller(E[1].desired, readMotorRawRelative(1), 1, 1);    // motor1 control
            PID_Controller(E[2].desired, readMotorRawRelative(2), 2, 1);    // motor2 control
            break;
    }
    if (i >= getN())
    {
        i = 0;
        //GPIOPinWrite(GPIO_PORTP_BASE,GPIO_PIN_3,0);
        setMODE(IDLE);
    }
}

// Calculate control effort and set pwm value to control motor
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

    // Convert mA to counts, bound

    // Max effort
    if (u > 9600)
        u = 9600;
    else if (u < -9600)
        u = -9600;
    motorControlPWM(motor, u);

    if (u > 2047)
    {
        u = 2047;
    }
    else if (u < -2047)
    {
        u = -2047;
    }

    set_motor_pwm(motor, u);
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

