/**
 * @file PositionControl.h
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

typedef struct {
    int Enew;   /**< Error between desired and actual angle */
    int Eold;   /**< Error from previous control cycle */
    int Eint;   /**< Summation of error over all control cycles */
    int Edot;   /**< Difference between new and old error */
    int desired;/**< Desired angle in counts */
    int actual; /**< Actual angle in counts (relative) */
    int raw;    /**< Absolute angle in counts */
    int u;      /**< Control effort */
} control_error; // Define data structure containing control data

/**
* @brief This function sets up the timer interrupt for position control
*
* @param Void
* @return Void
*/
extern void MotorTimerInit(void);

/**
* @brief The interrupt handler for position control
*
* @param Void
* @return Void
*/
extern void Timer1IntHandler(void);

/**
* @brief Sets position gains on the Tiva microcontroller by reading from UART
*
* @param Void
* @return Void
*/
void set_position_gains(void);

/**
* @brief Gets position gains on the Tiva microcontroller by reading to UART
*
* @param Void
* @return Void
*/
void get_position_gains(void);

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
* @brief Resets desired position to 0 counts
*
* @param Void
* @return Void
*/
void reset_pos(void);

/**
* @brief Resets controller error on both controllers to be zero
*
* @param Void
* @return Void
*/
void reset_controller_error(void);


/**
* @brief Loads position trajectory for a given motor over UART
*
* @param motor specifies the trajectory that is being assigned from UART
* @return Void
*/
void load_position_trajectory(int motor);

/**
* @brief Calculates pwm to send to the motor drivr
*
* @param reference the reference angle
* @param actual the commanded angle
* @param motor the specified motor
* @return Void
*/
void PID_Controller(int reference, int actual, int motor);

/**
* @brief Motor decogging to smooth output of motor
*
* Motor decogging function that smoothes out the output from the motor by adding a
* term to the control signal to compensate for the cogging in the BLDC motor. 
* Equations are fit using PositionDataAnalyze.m MATLAB script.
*
* Example:
*
*     u = u + decog_motor(current_Angle_Radians, MOTOR_1);
*
* @param x current angle in counts
* @param motor the motor to decog
* @return control signal modifier based on position
*/
int decog_motor(int x, int motor);

/**
* @brief Turn on/off decogging by setting global variable
*
*   
* @param void
* @return Void
*/
void setDecogging(void);

#endif /* POSITION_CONTROL_H_ */
