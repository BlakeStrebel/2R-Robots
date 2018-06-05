#include "r2r.h"
#include "System.h"
#include "Encoder.h"
#include "Motor.h"
#include "PositionControl.h"
#include "Utilities.h"
#include "CurrentControl.h"

#define BUF_SIZE 50

// The error routine that is called if the driver library encounters an error.
/*
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif
*/

int
main(void)
{
    // Init code
    r2rDefaultInit();
    char buffer[BUF_SIZE];

    setMotorZero(1);
    setMotorZero(2);

    //Just for test
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE,GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTP_BASE,GPIO_PIN_3,0);

    motorControlPWM(1, 0);
    motorControlPWM(2, 0);


    while(1)
    {
        while(UARTCharsAvail(UART0_BASE))
            switch (UARTCharGet(UART0_BASE))
            {
                case 1: // Calibrate
                    setMotorZero(1);
                    setMotorZero(2);
                    break;
                case 2: // Read Motor 1 position.
                    UART0FloatPut(2 * M_PI * readMotorRawRelative(1) / 16383);
                    break;
                case 3: // Read Motor 2 position.
                    UART0FloatPut(2 * M_PI * (readMotorRawRelative(2) - readMotorRawRelative(1)) / 16383);
                    break;
                case 4: // Read two motors' positions.
                    UART0FloatPut(2 * M_PI * readMotorRawRelative(1) / 16383);
                    UART0FloatPut(2 * M_PI * (readMotorRawRelative(2) - readMotorRawRelative(1)) / 16383);
                    break;
                case 5: // Rotate Motor 1
                    motorControlPWM(1, (int)(96 * UART0FloatGet()));
                    motorControlPWM(2, 0);
                    break;
                case 6: // Rotate Motor 2
                    motorControlPWM(1, 0);
                    motorControlPWM(2, (int)(96 * UART0FloatGet()));
                    break;
                case 7: // Rotate both motors
                    motorControlPWM(1, (int)(96 * UART0FloatGet()));
                    motorControlPWM(2, (int)(96 * UART0FloatGet()));
                    break;
                case 8:
                    motorControlPWM(1, 0);
                    motorControlPWM(2, 0);
                    break;
                case 9: // Set reading time.
                    setN(UART0IntGet());
                    break;
                case 10: // Read Motor 1 position continuously.
                    setMODE(READ1);
                    break;
                case 11: // Read Motor 2 position continuously.
                    setMODE(READ2);
                    break;
                case 12: // Read both motors' positions continuously.
                    setMODE(READb);
                    break;
                case 13: // Set initial position.
                    set_desired_angle((int)(UART0FloatGet() / 2 / M_PI * 16383), 1);
                    set_desired_angle((int)(UART0FloatGet() / 2 / M_PI * 16383), 2);
                    //GPIOPinWrite(GPIO_PORTP_BASE,GPIO_PIN_3,GPIO_PIN_3);
                    break;
                case 14: // Load Motor 1 trajectory.
                    setN(UART0IntGet());
                    loadPositionTrajectory(1);
                    break;
                case 15: // Load Motor 2 trajectory.
                    setN(UART0IntGet());
                    loadPositionTrajectory(2);
                    break;
                case 16: // Load two motors' trajectories.
                    setN(UART0IntGet() / 2);
                    loadPositionTrajectory(1);
                    loadPositionTrajectory(2);
                    break;
                case 17: // Set Motor 1 position control PID.
                    setPositionPID(1);
                    break;
                case 18: // Set Motor 2 position control PID.
                    setPositionPID(2);
                    break;
                case 19: // Set two motors' position control PIDs.
                    setPositionPID(1);
                    setPositionPID(2);
                    break;
                case 20: // Motor 1 PID control.
                    setMODE(PID1);
                    break;
                case 21: // Motor 2 PID control.
                    setMODE(PID2);
                    break;
                case 22: // Two motors' PID control.
                    setMODE(PIDb);
                    break;

                case 26:
                    setMODE(IDLE);
                    break;
                case 27:
                    setMODE(HOLD);
                case 28:
                    setDecogging(1);
                case 29:
                    setDecogging(0);
            }

    }

/*
    // Loop Forever
    while(1)
    {
        UART0read(buffer, BUF_SIZE); // Expect next character to be a menu command

        switch (buffer[0]) {
               case 'a':    // Read Raw Encoder Values
               {
                   int a1, a2;
                   a1 = readMotor1Raw();
                   a2 = readMotor2Raw();
                   sprintf(buffer, "%d\r\n", a1);
                   UART0write(buffer);
                   sprintf(buffer, "%d\r\n", a2);
                   UART0write(buffer);
                   break;
               }
               case 'b':    // Read Relative Encoder Angle
               {
                   int b1, b2;
                   b1 = readMotor1RawRelative();
                   b2 = readMotor2RawRelative();
                   sprintf(buffer, "%d\r\n", b1);
                   UART0write(buffer);
                   sprintf(buffer, "%d\r\n", b2);
                   UART0write(buffer);
                   break;
               }
               case 'c':    // Set Motor PWM
               {
                   int p1, p2;
                   UART0read(buffer,BUF_SIZE);
                   sscanf(buffer, "%d %d", &p1, &p2);
                   setMODE(PWM);
                   motor1ControlPWM(p1);
                   motor2ControlPWM(p2);
                   break;
               }
               case 'd':    // Get Motor PWM
               {
                   int pwm1, pwm2;
                   pwm1 = getmotor1PWM();
                   pwm2 = getmotor2PWM();
                   sprintf(buffer, "%d\r\n",pwm1);
                   UART0write(buffer);
                   sprintf(buffer, "%d\r\n",pwm2);
                   UART0write(buffer);
                   break;
               }
               case 'e':    // Get Desired Angle
               {
                   int d1, d2;
                   d1 = get_desired_angle(1);
                   d2 = get_desired_angle(2);
                   sprintf(buffer, "%d\r\n", d1);
                   UART0write(buffer);
                   sprintf(buffer, "%d\r\n", d2);
                   UART0write(buffer);
                   break;
               }
               case 'f':    // Set Desired Angle
               {
                   char buffer[25]; int d1, d2;
                   UART0read(buffer,25);
                   sscanf(buffer, "%d %d", &d1, &d2);
                   set_desired_angle(d1, 1);
                   set_desired_angle(d2, 2);
                   break;
               }
               case 'g':    // Get Motor Mode
               {
                   int mode = getMODE();
                   sprintf(buffer, "%d\r\n",mode);
                   UART0write(buffer);
                   break;
               }
               case 'h':    // Set Position Gains
               {
                   set_position_gains();
                   break;
               }
               case 'i':    // Get Position Gains
               {
                   get_position_gains();
                   break;
               }
               case 'j':    // Position Hold Mode
               {
                   setMODE(HOLD);
                   break;
               }
               case 'k':    // Load Position Trajectory
               {
                   load_position_trajectory(1);
                   load_position_trajectory(2);
                   break;
               }
               case 'l':    // Execute Position Trajectory
               {
                   setMODE(TRACK);
                   send_data();
                   break;
               }
               case 'm':   // free moving arm
               {
                   set_position_gains();
                   load_position_trajectory(1);
                   load_position_trajectory(2);
                   break;
               }
               case 'n':   // Hello User test
               {
                   set_position_gains();
                   load_position_trajectory(1);
                   load_position_trajectory(2);
                   setMODE(TRACK);
                   send_data();
                   break;
               }
               case 'o':
               {
                   setDecogging();
               }
               case 'r': // zero encoders
               {
                   setMotorZero(1);
                   setMotorZero(2);
                   break;

               }
               case 'q':    // Motor Off
               {
                   motor1ControlPWM(0);
                   motor2ControlPWM(0);
                   setMODE(IDLE);
                   break;
               }
               case '1':    // Read Phase Currents in Counts
               {
                   get_counts();
                   break;
               }
               case '2':    // Read Current in mA
               {
                   get_mA();
                   break;
               }
               case '3':
               {
                   setNclient(5000);
                   setMODE(ISENSE);
                   send_data();
                   break;
               }
               case '4':
               {
                   setNclient(1000);
                   setMODE(ITEST);
                   send_data();
                   break;
               }
               case '5':
               {
                   set_current_gains();
                   break;
               }
               case '6':
               {
                   get_current_gains();
                   break;
               }
               case '7':
               {
                   setMODE(ICALIB);
               }
               default:
               {
                   break;// Don't do anything
               }
           }
    }
*/
}
