/**
 * @file Motor.h
 *
 * This file contains all motor-related functions, such as initialization and setting speeds
 *
 * @author Benjamen Lim
 * @author Huan Weng
 * @author Blake Strebel
 *
 */

#ifndef MOTOR_H_
#define MOTOR_H_

// error messages
#define MOTOR_SPINNING_TOO_FAST 10
#define MOTOR_TOO_HOT 11
#define PWMPERIOD 4000

// limits
#define TEMP_LIMIT 50 // in degrees C
#define MAX_SPEED 10 // in rev/s


/**
 * Brakes the motor and kills the pwm signal by pulling down the enable line.
 *
 *
 * @param Void
 * @return Void
 */
extern void shutdownNow(void);

/**
 * Function to check for all safety limits and kills the motors if a fault is detected. 
 * Function must be called each time
 *
 *
 * @param Void
 * @return Void
 */
extern void motorSafetyCheck(void);

/**
 * @brief Initializes the SPI channels for the motor drivers
 *
 * Initializes SSI1~2 and their corresponding GPIO pins
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
 * Initialize PWM on PF0 (motor 1) and PG0 (motor 2), and the default period if 9600 SysClk
 * cycles or 12500Hz
 *
 * @param Void
 * @return Void
 */
extern void pwmInit(void);


/**
 * @brief Sets the PWM value to motor 1
 *
 * @param pwmValue The pwmValue ranges from 0 to the pwm period. The default pwm period is 320 SysClk cycles
 * @return Void
 *
 */
extern void motor1PWM(int pwmValue);
/**
 * @brief Sets the PWM value to motor 1
 *
 * @param pwmValue The pwmValue ranges from 0 to the pwm period. The default pwm period is 320 SysClk cycles
 * @return Void
 *
 */
extern void motor2PWM(int pwmValue);
/**
 * @brief Sets the PWM value and direction to motor 1
 *
 * @param control The pwmValue ranges from -maximum pwm period to the positive period. The default pwm period is 320 SysClk cycles
 * @return Void
 *
 */
extern void motor1ControlPWM(int control);
/**
 * @brief Sets the PWM value and direction to motor 2
 *
 * @param control The pwmValue ranges from -maximum pwm period to the positive period. The default pwm period is 9600 SysClk cycles
 * @return Void
 *
 */
extern void motor2ControlPWM(int control);

/**
 * @brief Brakes motor 1
 *
 * @param Void
 * @return Void
 */
extern void motor1Brake(void);

/**
 * @brief Brakes motor 2
 *
 * @param Void
 * @return Void
 */
extern void motor2Brake(void);



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