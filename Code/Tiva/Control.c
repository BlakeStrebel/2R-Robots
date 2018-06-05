#include "System.h"
#include "Control.h"
#include "Utilities.h"
#include "Encoder.h"
#include "Motor.h"
#include <math.h>

// PID gains
static volatile float Kp[3] = {0.0};
static volatile float Ki[3] = {0.0};
static volatile float Kd[3] = {0.0};
static volatile float dKp[3] = {0.0};
static volatile float dKi[3] = {0.0};
static volatile float dKd[3] = {0.0};
static volatile int t = 0;

// Control info
//static volatile control_data_t M1;
//static volatile control_data_t M2;
static volatile control_error E[3];

// Decogging state
static volatile int DECOGGING = 0;
/*
 * This function sets up the timer interrupt used for motor control
 *
 * Comes after:
 * - sysInit()
 */
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

    E[1].u = 0;
    E[2].u = 0;
}

void get_position_gains(void)   // provide position control gains
{
    char buffer[25];
    sprintf(buffer, "%f\r\n",Kp);
    UART0write(buffer);
    sprintf(buffer, "%f\r\n",Ki);
    UART0write(buffer);
    sprintf(buffer, "%f\r\n",Kd);
    UART0write(buffer);
}

void set_position_gains(void)   // recieve position control gains
{
    char buffer[25];
    float Kptemp, Kitemp, Kdtemp;
    UART0read(buffer,25);                              // Store gains in buffer
    sscanf(buffer, "%f %f %f",&Kptemp, &Kitemp, &Kdtemp);   // Extract gains to temporary variables
    IntMasterDisable();                         // Disable interrupts briefly
    Kp[1] = Kptemp;                                            // Set gains
    Ki[1] = Kitemp;
    Kd[1] = Kdtemp;
    reset_controller_error();
    IntMasterEnable();                          // Re-enable interrupts
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
    return E[motor].u;
}

void set_motor_pwm(int motor, int value)
{
    motorControlPWM(motor, value);
    E[motor].u = value;
}


void
Timer2IntHandler(void)
{
    //adcRead();
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
}


void
Timer1IntHandler(void)
{
    static int decctr = 0;  // counter for data decimation
    static int i = 0;   // trajectory index

    encoderRead(1); // update encoder values
    encoderRead(2); // update encoder values
    E[1].actual = readMotorRawRelative(1); // read positions
    E[2].actual = readMotorRawRelative(2);
    switch(getMODE())
    {
        case IDLE:
            break;
        case PID1:
            PID_Controller(E[1].traj[i], E[1].actual, 1);    // motor1 control
        case READ1:
            decctr++;
            if (decctr == DECIMATION)
            {
                UART0FloatPut(2 * M_PI * readMotorRawRelative(1) / 16383);
                decctr = 0; // reset decimation counter
            }
            i++;
            break;
        case PID2:
            PID_Controller(E[2].traj[i], E[2].actual, 2);    // motor2 control
        case READ2:
            decctr++;
            if (decctr == DECIMATION)
            {
                UART0FloatPut(2 * M_PI * readMotorRawRelative(2) / 16383);
                decctr = 0; // reset decimation counter
            }
            i++;
            break;
        case PIDb:
            //GPIOPinWrite(GPIO_PORTP_BASE,GPIO_PIN_3,GPIO_PIN_3);
            PID_Controller(E[1].traj[i], E[1].actual, 1);    // motor1 control
            PID_Controller(E[2].traj[i], E[2].actual, 2);    // motor2 control
        case READb:
            decctr++;
            if (decctr == DECIMATION)
            {
                UART0FloatPut(2 * M_PI * readMotorRawRelative(1) / 16383);
                UART0FloatPut(2 * M_PI * readMotorRawRelative(2) / 16383);
                decctr = 0; // reset decimation counter
            }
            i++;
            //GPIOPinWrite(GPIO_PORTP_BASE,GPIO_PIN_3,GPIO_PIN_3);
            break;
        case HOLD:
            PID_Controller(E[1].desired, E[1].actual, 1);    // motor1 control
            PID_Controller(E[2].desired, E[2].actual, 2);    // motor2 control
            break;
    }
    if (i >= t)
    {
        i = 0;
        //GPIOPinWrite(GPIO_PORTP_BASE,GPIO_PIN_3,0);
        setMODE(IDLE);
    }

    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT); // clear the interrupt flag
}

// Calculate control effort and set pwm value to control motor
void PID_Controller(int reference, int actual, int motor)
{
    static float u;

    E[motor].Enew = reference - actual;               // Calculate error
    E[motor].Eint = E[motor].Eint + E[motor].Enew;                // Calculate intergral error
    E[motor].Edot = E[motor].Enew - E[motor].Eold;                // Calculate derivative error
    E[motor].Eold = E[motor].Enew;                          // Update old error
    u = Kp[motor]*E[motor].Enew + Ki[motor]*E[motor].Eint + Kd[motor]*E[motor].Edot;   // Calculate effort

    if (DECOGGING)                              // Add decogging control
        u += decog_motor(readMotorCounts(motor), motor);

    // Max effort
    if (u > 9600)
    {
        u = 9600;
    }
    else if (u < -9600)
    {
        u = -9600;
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

void setDecogging(void) // Turn motor decogging on/off
{
    char buffer[10]; int decog;
    sscanf(buffer,"%d",&decog);
    DECOGGING = decog;
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

void setTime(int time)
{
    t = time;
}

void loadTrajectory(int motor)
{
    int i;
    for (i = 0; i < t; i++)
        E[motor].traj[i] = UART0FloatGet();
}

void set_position_PID(int motor)   // recieve position control gains
{
    Kp[motor] = UART0FloatGet();
    Ki[motor] = UART0FloatGet();
    Kd[motor] = UART0FloatGet();
    reset_controller_error();
}
