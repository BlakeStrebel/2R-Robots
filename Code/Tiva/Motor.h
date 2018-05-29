/**
 * @file Motor.h
 * @brief utilities header
 *
 * This file contains the motor functions
 *
 * @author Benjamen Lim
 * @author Huan Weng
 * @author Blake Strebel
 *
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#define MOTOR_SPINNING_TOO_FAST 10
#define MOTOR_TOO_HOT 11




extern void motorSafetyCheck(void);

/**
 * @brief Initializes the SPI channels for the motor drivers
 *
 * Initializes SSI1 ~ 4 and their corresponding GPIO pins
 *
 * @param Void
 * @return Void
 */
extern void MotorSPIInit(void);

/**
* @brief Initializes the GPIO pins for the motor
* 
* Initializes the direction, brake, and halt pins for the motor in 1x PWM mode
* 
* @param Void
* @return Void
*/
extern void motorInit(void);

/**
 * @brief Initializes the SPI commands to set the motor's configuration for 1x PWM mode.
 * @param Void
 * @return Void
 */
extern void motorDriverInit(void);


/**
 * @brief Initializes PWM
 *
 * Initialize PWM on PF0 (motor 1) and PG0 (motor 2), and the default period if 320 SysClk
 * cycles.
 *
 * @param Void
 * @return Void
 */
extern void pwmInit(void);


/**
 * @brief Sets the PWM value to motors
 *
 * @param The motor number
 * @param pwmValue The pwmValue ranges from 0 to the pwm period. The default pwm period is 320 SysClk cycles
 * @return Void
 *
 */
extern void motorPWM(int motor_number, int pwmValue);

/**
 * @brief Sets the PWM value and direction to motors
 *
 * @param The motor number
 * @param control The pwmValue ranges from -maximum pwm period to the positive period. The default pwm period is 320 SysClk cycles
 * @return Void
 *
 */
extern void motorControlPWM(int motor_number, int control);

/**
 * @brief Brakes motors
 *
 * @param The motor number
 * @return Void
 */
extern void motorBrake(int motor_number);

/**
 * @brief Checks the error pins and returns a value based on the state of the pin
 *
 * Returns 0 on no error, 1 on motor 1 error, and 2 on motor 2 error
 *
 * @param Void 
 * @return Void
 *
 */
extern int motorError(void); 



#endif /* MOTOR_H_ */
