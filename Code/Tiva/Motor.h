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
#define PWMPERIOD 4000


/* MOTOR 1 HALL SENSOR PINS */
#define M1H_PERIPH SYSCTL_PERIPH_GPIOL
#define M1H_PORT GPIO_PORTL_BASE // IF YOU CHANGE THIS PORT YOU MUST CHANGE THE INT HANDLER IN tm4c1294ncpdt_startup_css.c
#define M1H_PIN_A GPIO_PIN_0
#define M1H_PIN_B GPIO_PIN_1
#define M1H_PIN_C GPIO_PIN_2
#define M1H_PINS (M1H_PIN_A | M1H_PIN_B | M1H_PIN_C)

/* MOTOR 1 CONTROL PINS */
#define M1_INL_PERIPH SYSCTL_PERIPH_GPIOP
#define M1_INL_PORT GPIO_PORTP_BASE
#define M1_INL_PIN_A GPIO_PIN_0
#define M1_INL_PIN_B GPIO_PIN_1
#define M1_INL_PIN_C GPIO_PIN_2
#define M1_INL_PINS (M1_INL_PIN_A | M1_INL_PIN_B | M1_INL_PIN_C)

/* MOTOR 1 COMMUTATION DATA */
#define M1_HALLSTATE_0 (M1H_PIN_A | M1H_PIN_B)
#define M1_HALLSTATE_1 M1H_PIN_A
#define M1_HALLSTATE_2 (M1H_PIN_A | M1H_PIN_C)
#define M1_HALLSTATE_3 M1H_PIN_C
#define M1_HALLSTATE_4 (M1H_PIN_B | M1H_PIN_C)
#define M1_HALLSTATE_5 M1H_PIN_B

/* MOTOR 1 OUTPUT STATES */
#define M1_INL_OUTPUT_0 (M1_INL_PIN_B | M1_INL_PIN_C)
#define M1_INL_OUTPUT_1 (M1_INL_PIN_A | M1_INL_PIN_C)
#define M1_INL_OUTPUT_2 (M1_INL_PIN_A | M1_INL_PIN_B)
#define M1_INL_BREAK 0x00

/* MOTOR 2 HALL SENSOR PINS */
#define M2H_PERIPH SYSCTL_PERIPH_GPION
#define M2H_PORT GPIO_PORTN_BASE // IF YOU CHANGE THIS PORT YOU MUST CHANGE THE INT HANDLER IN tm4c1294ncpdt_startup_css.c
#define M2H_PIN_A GPIO_PIN_0
#define M2H_PIN_B GPIO_PIN_1
#define M2H_PIN_C GPIO_PIN_2
#define M2H_PINS (M2H_PIN_A | M2H_PIN_B | M2H_PIN_C)

/* MOTOR 2 CONTROL PINS */
#define M2_INL_PERIPH SYSCTL_PERIPH_GPIOQ
#define M2_INL_PORT GPIO_PORTQ_BASE
#define M2_INL_PIN_A GPIO_PIN_0
#define M2_INL_PIN_B GPIO_PIN_2
#define M2_INL_PIN_C GPIO_PIN_3
#define M2_INL_PINS (M2_INL_PIN_A | M2_INL_PIN_B | M2_INL_PIN_C)

/* MOTOR 2 COMMUTATION DATA */
#define M2_HALLSTATE_0 (M2_PIN_A | M2_PIN_B)
#define M2_HALLSTATE_1 M2_PIN_A
#define M2_HALLSTATE_2 (M2_PIN_A | M2_PIN_C)
#define M2_HALLSTATE_3 M2_PIN_C
#define M2_HALLSTATE_4 (M2_PIN_B | M2_PIN_C)
#define M2_HALLSTATE_5 M2_PIN_B

/* MOTOR 2 OUTPUT STATES */
#define M2_INL_OUTPUT_0 (M2_INL_PIN_B | M2_INL_PIN_C)
#define M2_INL_OUTPUT_1 (M2_INL_PIN_A | M2_INL_PIN_C)
#define M2_INL_OUTPUT_2 (M2_INL_PIN_A | M2_INL_PIN_B)
#define M2_INL_BREAK 0x00


extern void M1HIntHandler(void);
extern void M2HIntHandler(void);

extern void motor1PWM(int pwm1, int pwm2, int pwm3);

extern void motor2PWM(int pwm1, int pwm2, int pwm3);

extern void motor1ControlPWM(int control);

extern void motor2ControlPWM(int control);



extern void motorInit(void);






extern void motorSafetyCheck(void);

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





extern float readMotor2Speed(void);
extern float readMotor1Speed(void);


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
