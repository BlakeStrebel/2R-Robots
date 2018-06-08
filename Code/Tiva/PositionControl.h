/**
 * @file Control.h
 * @brief control header
 *
 * This file contains the pwm functions
 *
 * @author Benjamen Lim
 * @author Huan Weng
 * @author Blake Strebel
 *
 */

#ifndef POSITION_CONTROL_H_
#define POSITION_CONTROL_H_


static const float defaultKp[3] = {0, 2, 2};
static const float defaultKi[3] = {0, 0, 0};
static const float defaultKd[3] = {0, 0, 0};


typedef struct {                          // Define data structure containing control data
    float Kp;
    float Ki;
    float Kd;
} PID_gains;

// TODO in setup set these to zero
// TODO in setup set these to zero
typedef struct {                          // Define data structure containing control data
    int Enew;
    int Eold;
    int Eint;
    int Edot;
    int desired;
    float traj[10000];
} control_error;

extern void positionControlInit(void);

extern void Timer1IntHandler(void);


/**
* @brief Gets desired angle for a given motor
*
* @param motor reads the desired angle for the given motor
* @return Void
*/
int get_desired_angle(int motor);

/**
* @brief Sets desired angle for a given motor
*
* @param motor specifies the desired angle for the given motor
* @return Void
*/
void set_desired_angle(int angle, int motor);

/**
* @brief Resets desired position to origin
*
* @param Void
* @return Void
*/
void reset_pos(void);                               // Reset desired position to origin (0 um)

/**
* @brief Resets controller error on both controllers to be zero
*
* @param Void
* @return Void
*/
void reset_controller_error(void);                  // Reset the error on both controllers to be zero

/**
* @brief Calculates pwm to send to the motor drivr
*
* @param reference the reference angle
* @param actual the commanded angle
* @param motor the specified motor
* @return Void
*/
void PID_Controller(int reference, int actual, int motor, int default_gains);

float decog_motor(int x, int motor);

void setDecogging(int decog);

void loadPositionTrajectory(int motor);

void setPositionPID(int motor);

#endif /* POSITION_CONTROL_H_ */
