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

#define PWMPERIOD 4000

// MOTOR 1 HALL SENSOR PINS
#define M1H_PERIPH SYSCTL_PERIPH_GPIOM
#define M1H_PORT GPIO_PORTM_BASE // IF YOU CHANGE THIS PORT YOU MUST CHANGE THE INT HANDLER IN tm4c1294ncpdt_startup_css.c
#define M1H_PORT_INT INT_GPIOM
#define M1H_PIN_A GPIO_PIN_0
#define M1H_PIN_B GPIO_PIN_1
#define M1H_PIN_C GPIO_PIN_2
#define M1H_PINS (M1H_PIN_A | M1H_PIN_B | M1H_PIN_C)

// MOTOR 1 COMMUTATION DATA
#define M1_HALLSTATE_0 (M1H_PIN_A | M1H_PIN_B)
#define M1_HALLSTATE_1 M1H_PIN_A
#define M1_HALLSTATE_2 (M1H_PIN_A | M1H_PIN_C)
#define M1_HALLSTATE_3 M1H_PIN_C
#define M1_HALLSTATE_4 (M1H_PIN_B | M1H_PIN_C)
#define M1_HALLSTATE_5 M1H_PIN_B

// MOTOR 1 CONTROL PINS
#define M1_INL_PERIPH_A SYSCTL_PERIPH_GPIOP
#define M1_INL_PORT_A GPIO_PORTP_BASE
#define M1_INL_PIN_A GPIO_PIN_0
#define M1_INL_PERIPH_B SYSCTL_PERIPH_GPIOP
#define M1_INL_PORT_B GPIO_PORTP_BASE
#define M1_INL_PIN_B GPIO_PIN_1
#define M1_INL_PERIPH_C SYSCTL_PERIPH_GPIOP
#define M1_INL_PORT_C GPIO_PORTP_BASE
#define M1_INL_PIN_C GPIO_PIN_2

// MOTOR 2 HALL SENSOR PINS
#define M2H_PERIPH SYSCTL_PERIPH_GPION
#define M2H_PORT GPIO_PORTN_BASE // IF YOU CHANGE THIS PORT YOU MUST CHANGE THE INT HANDLER IN tm4c1294ncpdt_startup_css.c
#define M2H_PORT_INT INT_GPION
#define M2H_PIN_A GPIO_PIN_0
#define M2H_PIN_B GPIO_PIN_1
#define M2H_PIN_C GPIO_PIN_2
#define M2H_PINS (M2H_PIN_A | M2H_PIN_B | M2H_PIN_C)

// MOTOR 2 CONTROL PINS
#define M2_INL_PERIPH_A SYSCTL_PERIPH_GPIOM
#define M2_INL_PORT_A GPIO_PORTM_BASE
#define M2_INL_PIN_A GPIO_PIN_5
#define M2_INL_PERIPH_B SYSCTL_PERIPH_GPIOM
#define M2_INL_PORT_B GPIO_PORTM_BASE
#define M2_INL_PIN_B GPIO_PIN_6
#define M2_INL_PERIPH_C SYSCTL_PERIPH_GPIOM
#define M2_INL_PORT_C GPIO_PORTM_BASE
#define M2_INL_PIN_C GPIO_PIN_7

// MOTOR 2 COMMUTATION DATA
#define M2_HALLSTATE_0 (M2H_PIN_A | M2H_PIN_B)
#define M2_HALLSTATE_1 M2H_PIN_A
#define M2_HALLSTATE_2 (M2H_PIN_A | M2H_PIN_C)
#define M2_HALLSTATE_3 M2H_PIN_C
#define M2_HALLSTATE_4 (M2H_PIN_B | M2H_PIN_C)
#define M2_HALLSTATE_5 M2H_PIN_B


/**
 * @brief Hall sensor interrupt for motor 1
 *
 * This function is used to determine the commutation on motor 1
 *
 * @param Void
 * @return Void
 *
 */
extern void M1HIntHandler(void);


/**
 * @brief Hall sensor interrupt for motor 2
 *
 * This function is used to determine the commutation on motor 2
 *
 * @param Void
 * @return Void
 *
 */
extern void M2HIntHandler(void);

/**
 * @brief Commutate motor 1 with a given pwm
 *
 * The state of the commutation is determined by the hall sensing
 *
 *
 * @param State of 1 to 6
 * @param control the pwm for the motor  
 * @return Void
 *
 */
void motor1Commutate(int State, int control);

/**
 * @brief Commutate motor 2 with a given pwm
 *
 * The state of the commutation is determined by the hall sensing
 *
 * @param State of 1 to 6
 * @param control the pwm for the motor  
 * @return Void
 *
 */
void motor2Commutate(int State, int control);


/**
 * @brief Sets the PWM value to motor 1
 *
 * @param pwm1 the pwm output on phase A
 * @param pwm2 the pwm output on phase B
 * @param pwm3 the pwm output on phase C
 * @return Void
 *
 */
extern void motor1PWM(int pwm1, int pwm2, int pwm3);

/**
 * @brief Sets the PWM value to motor 2
 *
 * @param pwm1 the pwm output on phase A
 * @param pwm2 the pwm output on phase B
 * @param pwm3 the pwm output on phase C
 * @return Void
 *
 */
extern void motor2PWM(int pwm1, int pwm2, int pwm3);


/**
 * @brief Sets the PWM value and direction to motor 1
 *
 * This function handles all the commutation for motor 1. This is done via hall sensing to determine the position the BLDC motor is in.
 * 
 * Example:
 *		motor2ControlPWM(200); // sets the motor speed to 5% of default, clockwise
 *		delayMS(100); //delay for 100 ms
 *		motor2ControlPWM(-200); // sets motor speed to 5% of default, counter clockwise.
 *
 * @param control The pwmValue ranges from -maximum pwm period to the positive period. The default pwm period is 4000, 30kHz.
 * @return Void
 *
 */
extern void motor1ControlPWM(int control);

/**
 * @brief Sets the PWM value and direction to motor 2
 *
 * This function handles all the commutation for motor 2. This is done via hall sensing to determine the position the BLDC motor is in.
 *
 * Example:
 *		motor2ControlPWM(200); // sets the motor speed to 5% of default, clockwise
 *		delayMS(100); //delay for 100 ms
 *		motor2ControlPWM(-200); // sets motor speed to 5% of default, counter clockwise.
 *		
 *
 * @param control The pwmValue ranges from -maximum pwm period to the positive period. The default pwm period is 4000, 30kHz.
 * @return Void
 *
 */
extern void motor2ControlPWM(int control);



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
 * @brief Commutation table for motor 1
 *
 * This function contains the commutation table for motor 1 and sets the control pins for each phase accordingly.
 *
 * @param int a phase A 
 * @param int b phase B 
 * @param int c phase C 
 * @return Void
 *
 */
void M1_INL_WRITE(int a, int b, int c);

/**
 * @brief Commutation table for motor 2
 *
 * This function contains the commutation table for motor 2 and sets the control pins for each phase accordingly.
 *
 * @param int a phase A 
 * @param int b phase B 
 * @param int c phase C 
 * @return Void
 *
 */
void M2_INL_WRITE(int a, int b, int c);


/**
* @brief Returns the current PWM of motor 1
* 
* @param Void
* @return int pwm of motor
*/
int getmotor1PWM(void);


/**
* @brief Returns the current PWM of motor 2
* 
* @param Void
* @return int pwm of motor
*/
int getmotor2PWM(void);

/**
* @brief Returns the current state of the halls for motor 1
*
* It returns 32 bit number representing the state of the motor as a bit array
* It is compared to the HALLSTATE definitions to determine the current state. 
* 
* @param Void
* @return int32_t the state of the motor in a bit array
*/
int32_t getmotor1HALLS(void);


/**
* @brief Returns the current state of the halls for motor 2
*
* It returns 32 bit number representing the state of the motor as a bit array
* It is compared to the HALLSTATE definitions to determine the current state. 
* 
* @param Void
* @return int32_t the state of the motor in a bit array
*/
int32_t getmotor2HALLS(void);


/**
 * @brief Initializes the SPI channels for the motor drivers
 *
 * Initializes SSI1 ~ 4 and their corresponding GPIO pins
 *
 * @param Void
 * @return Void
 */
extern void MotorSPIinit(void);



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

#endif /* MOTOR_H_ */
