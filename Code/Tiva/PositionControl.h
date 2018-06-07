/**
 * @file PositionControl.h
 * @brief control header
 *
 * This file contains position control functions
 *
 * @author Benjamen Lim
 * @author Huan Weng
 * @author Blake Strebel
 *
 */

#ifndef POSITION_CONTROL_H_
#define POSITION_CONTROL_H_


/**
* This struct contains the PID data
*/

static const float defaultKp[3] = {0, 2, 2};
static const float defaultKi[3] = {0, 0, 0};
static const float defaultKd[3] = {0, 0, 0};

typedef struct {                          // Define data structure containing control data
    float Kp;
    float Ki;
    float Kd;
} PID_gains;

// TODO in setup set these to zero

typedef struct {                          // Define data structure containing control data
    int Enew;
    int Eold;
    int Eint;
    int Edot;
    int desired;

    float traj[10000];

} control_error;


extern void MotorTimerInit(void);

extern void Timer1IntHandler(void);


//void positioncontrol_setup(void);                   // Setup position control module
/**
* @brief Sets position gains on the Tiva microcontroller by reading from UART
*
* @param Void
* @return Void
*/
void setPositionGains(void);                      // Set position gains

/**
* @brief Gets position gains on the Tiva microcontroller by reading to UART
*
* @param Void
* @return Void
*/
void getPositionGains(void);                      // Get position gains


/**
* @brief Gets desired angle for a given motor
*
* @param motor reads the desired angle for the given motor
* @return Void
*/
int getDesiredAngle(int motor);

/**
* @brief Sets desired angle for a given motor
*
* @param motor specifies the desired angle for the given motor
* @return Void
*/
void setDesiredAngle(int angle, int motor);


/**
* @brief Resets controller error on both controllers to be zero
*
* @param Void
* @return Void
*/
void resetControllerError(void);                  // Reset the error on both controllers to be zero


/**
* @brief Loads position trajectory for a given motor over UART
*
* @param motor specifies the trajectory that is being assigned from UART
* @return Void
*/
void loadPositionTrajectory(int motor);                // Load desired position trajectory from client

/**
* @brief Returns the set pwm on a given motor
*
* @param int a given motor
* @return int the pwm on the motor
*/
int getMotorPWM(int motor);


/**
* @brief Calculates pwm to send to the motor driver
*
* @param reference the reference angle
* @param actual the commanded angle
* @param motor the specified motor
* @return Void
*/
void PID_Controller(int reference, int actual, int motor, int default_gains);

/**
* @brief Motor decogging to smooth output of motor
*
* Motor decogging function that smooths out the output from the motor by adding a 
* term to the control signal to compensate for the cogging in the BLDC motor. 
* Values are hardcoded to default r2r motors.
*
* Example:
*
*     u = u + decog_motor(current_Angle_Radians, MOTOR_1);
*
* @param x current angle in radians
* @param motor the motor to decog
* @return the modified control signal
*/
float decogMotor(int x, int motor);


/**
* @brief Turns on motor decogging
*
*   
* @param int 1 turns on decogging, 0 turns it off
* @return Void
*/
void setDecogging(int decog);



void setPositionPID(int motor);


#endif /* POSITION_CONTROL_H_ */
