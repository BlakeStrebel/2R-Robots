#include "r2r.h"
#include "System.h"
#include "Encoder.h"
#include "Motor.h"
#include "Control.h"
#include "Utilities.h"

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
                    UART0FloatPut(2 * M_PI * readMotorRawRelative(2) / 16383);
                    break;
                case 4: // Read two motors' positions.
                    UART0FloatPut(2 * M_PI * readMotorRawRelative(1) / 16383);
                    UART0FloatPut(2 * M_PI * readMotorRawRelative(2) / 16383);
                    break;

                    // Are these necessary?
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
                    break;



                case 9: // Set reading time.
                    setTime(UART0IntGet());
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
                    setTime(UART0IntGet());
                    loadTrajectory(1);
                    break;
                case 15: // Load Motor 2 trajectory.
                    setTime(UART0IntGet());
                    loadTrajectory(2);
                    break;
                case 16: // Load two motors' trajectories.
                    setTime(UART0IntGet() / 2);
                    loadTrajectory(1);
                    loadTrajectory(2);
                    break;
                case 17: // Set Motor 1 position control PID.
                    set_position_PID(1);
                    break;
                case 18: // Set Motor 2 position control PID.
                    set_position_PID(2);
                    break;
                case 19: // Set two motors' position control PIDs.
                    set_position_PID(1);
                    set_position_PID(2);
                    GPIOPinWrite(GPIO_PORTP_BASE,GPIO_PIN_3,GPIO_PIN_3);
                    break;
                case 20: // Motor 1 PID control.
                    setMODE(PID1);
                    break;
                case 21: // Motor 2 PID control.
                    setMODE(PID2);
                    break;
                case 22: // Two motors' PID control.
                    setMODE(PIDb);
                    GPIOPinWrite(GPIO_PORTP_BASE,GPIO_PIN_3,0);
                    //GPIOPinWrite(GPIO_PORTP_BASE,GPIO_PIN_3,GPIO_PIN_3);
                    break;


                    //These should be all float instead of int.
                case 23:
                    set_motor_pwm(1, (int)(UART0FloatGet() * 96));
                    break;
                case 24:
                    set_motor_pwm(2, (int)(UART0FloatGet() * 96));
                    break;
                case 25:
                    set_motor_pwm(1, (int)(UART0FloatGet() * 96));
                    set_motor_pwm(2, (int)(UART0FloatGet() * 96));
                    break;
                case 26:
                    setMODE(IDLE);
                    break;
                case 27:
                    setMODE(HOLD);
                case 28:
                    setDecogging();
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
                   a1 = readMotorRaw(1);
                   a2 = readMotorRaw(2);
                   sprintf(buffer, "%d\r\n", a1);
                   UART0write(buffer);
                   sprintf(buffer, "%d\r\n", a2);
                   UART0write(buffer);
                   break;
               }
               case 'b':    // Read Relative Encoder Angle
               {
                   int b1, b2;
                   b1 = readMotorRawRelative(1);
                   b2 = readMotorRawRelative(2);
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
                   set_motor_pwm(1, p1);
                   set_motor_pwm(2, p2);
                   break;
               }
               case 'd':    // Get Motor PWM
               {
                   int pwm1, pwm2;
                   pwm1 = get_motor_pwm(1);
                   pwm2 = get_motor_pwm(2);
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
                   setMODE(IDLE);
                   set_motor_pwm(1, 0);
                   set_motor_pwm(2, 0);
               }
               case '1':
               {
                   break;
               }
               case '2':
               {
                   break;
               }
               default:
               {
                   break;// Don't do anything
               }
           }
    }
*/
}
