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

extern void M1HIntHandler(void);
extern void M2HIntHandler(void);

void motorCommutate(int motor, int state, int control);

extern void motorPWMSet(int motor, int pwm1, int pwm2, int pwm3);

extern void motorControlPWM(int motor, int control);

extern void motorInit(void);

void motorINLWrite(int motor, int a, int b, int c);

int getmotorPWM(int motor);

int getmotorHALLS(int motor);



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
