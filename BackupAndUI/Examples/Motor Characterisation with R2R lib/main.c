/**
 * @file main.c
 * @brief Characterization of the motor, use with PUTTY, 8N1 Mode, 115200
 *
 * @author Benjamen Lim
 * @author Huan Weng
 *
 */


#include "r2r.h"

extern int r2rdebug;
float eint = 0;
float eprev = 0;
// PID control loop
float kp = 0.01;
float kd = 0.1;
float ki = 0.1;
float offset_angle;

int main()
{
    r2rdebug = 1; //set debug mode for PUTTY output
    int motorTested = 1; // 1 for motor 1, else 2 for motor 2

    // Init code
    r2rDefaultInit();

    // tuning code
    // This code tunes motor
    int i=0;
    float desired_angle = -60;
    float current_angle;


    while(i<1000){
         sensorUpdate(); // updates encoders

         if(motorTested==1){
             current_angle = readMotor1AngleRelative() - offset_angle;
         }

         if(motorTested==2){
             current_angle = readMotor2AngleRelative() - offset_angle;
         }


         float error = desired_angle - current_angle;


         eint = eint+error;
         // Integration clamp
         if(eint>300){
             eint=300;
         }
         else if(eint<-300){
             eint=-300;
         }
         float control = kp*error+kd*eprev+ki*eint;
         // Control signal clamp
         if (control>1800){
             control = 1800;
         }
         else if (control<-1800){
             control = -1800;
         }
         //UARTprintf("Motor 2: %d \t Motor 2 angle: %d \t Motor 2 control: %d\n",readMotor2Raw(),(int)readMotor2Angle(),(int)control);
         if(motorTested==2){
             motor2ControlPWM((int)control);
         }
         else if(motorTested==1){
             motor1ControlPWM((int)control);
         }
         eprev=error;
         i++;

        //motor1PWM(60);
        //motor2PWM(40);

    }
    UARTprintf("Desired_angle: %d \t Current Angle: %d\n",desired_angle,(int)current_angle);
    motor1PWM(1);
    motor2PWM(1);





    return(0);
}

