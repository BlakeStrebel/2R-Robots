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

// TODO in setup set these to zero
typedef struct {                          // Define data structure containing control data
    int Enew;
    int Eold;
    int Eint;
    int Edot;
    int desired;
    int actual;
    int raw;
    int u;
} control_error;

/**
* @brief This function sets up the timer interrupt for poosition control
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


//void positioncontrol_setup(void);                   // Setup position control module
/**
* @brief Sets position gains on the Tiva microcontroller by reading from UART
*
* @param Void
* @return Void
*/
void set_position_gains(void);                      // Set position gains

/**
* @brief Gets position gains on the Tiva microcontroller by reading to UART
*
* @param Void
* @return Void
*/
void get_position_gains(void);                      // Get position gains


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
* @brief Loads position trajectory for a given motor over UART
*
* @param motor specifies the trajectory that is being assigned from UART
* @return Void
*/
void load_position_trajectory(int motor);                // Load desired position trajectory from client


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
int decog_motor(int x, int motor);


/**
* @brief Receives position gains over UART, intended for use in a menu
*
*   
* @param int motor
* @return Void
*/
void setDecogging(void);

#endif /* POSITION_CONTROL_H_ */
