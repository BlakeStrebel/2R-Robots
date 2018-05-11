#include "r2r.h"
#include "System.h"
#include "Encoder.h"
#include "Motor.h"
#include "Control.h"
#include "Utilities.h"

#define BUF_SIZE 50

// The error routine that is called if the driver library encounters an error.
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif


int
main(void)
{
    // Init code
    r2rDefaultInit();

    char buffer[BUF_SIZE];

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

                   motor1ControlPWM(p1);
                   motor2ControlPWM(p2);
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
                   zeroMotor1RawRelative();
                   zeroMotor2RawRelative();
                   break;

               }
               case 'q':    // Motor Off
               {
                   setMODE(IDLE);
                   motor1ControlPWM(0);
                   motor2ControlPWM(0);
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
}
