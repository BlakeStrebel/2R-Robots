#include "System.h"
#include "Control.h"
#include "Utilities.h"
#include "Encoder.h"
#include "Motor.h"

// PID gains
static volatile float Kp = 2;
static volatile float Ki = 0;
static volatile float Kd = 0;

// Control info
static volatile control_data_t M1;
static volatile control_data_t M2;
static volatile control_error E1;
static volatile control_error E2;

/*
 * This function sets up the timer interrupt used for motor control
 *
 * Comes after:
 * - sysInit()
 */
void timerIntInit(void){
    setMODE(IDLE);
    IntMasterDisable();
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1); // Use timer 1
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER1_BASE, TIMER_A, ui32SysClock/1000); // Use timer B // activate every 1/2 of a second 120/120/2 = 0.5s
    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER1_BASE, TIMER_A);
    // TODO: GPIO is just for blinking purposes on the EK-TM4C129
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0); // LED PN0
    GPIOPinWrite(GPIO_PORTN_BASE,GPIO_PIN_0,GPIO_PIN_0); // Turn on the damn thing
    IntMasterEnable();
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


void
Timer1IntHandler(void)
{
    static int decctr = 0;  // counter for data decimation
    static int i = 0;   // trajectory index

    encoderRead(); // update encoder values
    switch(getMODE())
    {
        case IDLE:
        {
            // do nothing
            break;
        }
        case HOLD:
        {
            E1.actual = readMotor1RawRelative(); // read positions
            E2.actual = readMotor2RawRelative();
            PID_Controller(E1.desired, E1.actual, 1);    // motor1 control
            PID_Controller(E2.desired, E2.actual, 2);    // motor2 control
            break;

        }
        case TRACK:
        {
            if (i == getN())    // Done tracking when index equals number of samples
            {
                i = 0; // reset index
                setMODE(HOLD);  // Hold final position, Could set PWM to zero instead
            }
            else
            {
                E1.actual = readMotor1RawRelative(); // actual read positions
                E2.actual = readMotor2RawRelative();
                E1.desired = get_refPos(i, 1);
                E2.desired = get_refPos(i, 2);
                E1.u = PID_Controller(E1.desired, E1.actual, 1);    // motor1 control
                E2.u = PID_Controller(E2.desired, E2.actual, 2);    // motor2 control

                i++;    // increment index

                // Handle data decimation
                decctr++;
                if (decctr == DECIMATION)
                {
                    buffer_write(E1.actual, E2.actual, E1.u, E2.u);
                    decctr = 0; // reset decimation counter
                }
            }
            break;
        }
    }

    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT); // clear the interrupt flag
}

// Calculate control effort and set pwm value to control motor
float PID_Controller(int reference, int actual, int motor)
{
    static float u;

    if (motor == 1)
    {
        E1.Enew = reference - actual;               // Calculate error
        E1.Eint = E1.Eint + E1.Enew;                // Calculate intergral error
        E1.Edot = E1.Enew - E1.Eold;                // Calculate derivative error
        E1.Eold = E1.Enew;                          // Update old error
        u = Kp*E1.Enew + Ki*E1.Eint + Kd*E1.Edot;   // Calculate effort
    }
    else if (motor == 2)
    {
        E2.Enew = reference - actual;               // Calculate error
        E2.Eint = E2.Eint + E2.Enew;                // Calculate intergral error
        E2.Edot = E2.Enew - E2.Eold;                // Calculate derivative error
        E2.Eold = E2.Enew;                          // Update old error
        u = Kp*E2.Enew + Ki*E2.Eint + Kd*E2.Edot;   // Calculate effort
    }

    // Max effort
    if (u > 1800)
    {
        u = 1800;
    }
    else if (u < -1800)
    {
        u = -1800;
    }

    // Set new control effort
    if (motor == 1)
    {
        motor1ControlPWM(u);
    }
    else if(motor == 2)
    {
        motor2ControlPWM(u);
    }

    return u;
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

