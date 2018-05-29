/**
 * @file Encoder.h
 * @brief Encoder header
 *
 * This file contains the encoder functions
 *
 * @author Benjamen Lim
 * @author Huan Weng
 * @author Blake Strebel
 *
 */

#ifndef ENCODER_H_
#define ENCODER_H_


typedef struct {
    int position_count;
    int velocity_count;
    int zero_count;
    int previous_count;
    int continuous_count;
} encoder_states;



/**
 * @brief Initializes the SPI channels for the motor drivers, encoders, and RAM.
 *
 * Initializes SSI1 ~ 4 and their corresponding GPIO pins
 *
 * @param Void
 * @return Void
 */
extern void encoderSPIInit(void);

/**
 * @brief Updates value in sensor array
 *
 * This function sends a 't' and 40 pulses via SPI to the encoder and reads out the encoder's reply to
 * the encoderVal array
 *
 * @param The motor number
 * @return Void
 *
 */
extern void encoderRead(int motor_number);

/**
 * @brief Reads value of motor encoders and sets the relative angle to zero
 *
 * @param The motor number
 * @return Void
 *
 */
extern void zeroMotorRawRelative(int motor_number);


/**
 * @brief Reads value in encoderVal array for motors and converts it to an angle
 *
 * @param The motor number
 * @return The angle as a float
 *
 */
extern float readMotorAngle(int motor_number);

/**
 * @brief Reads value in encoderVal array for motors
 *
 * The range for the read angle is 0 to 16383
 *
 * @param The motor number
 * @return The angle as an int
 *
 */
extern int readMotorRaw(int motor_number);

/**
 * @brief Reads value in encoderVal array for motors and keeps track of multiturn counts
 *
 * This function reads out the angle and keeps track of it for multiple turns. The value
 * ranges from -2,147,483,648 to 2,147,483,648. Each turn is 16383 counts
 *
 * @param The motor number
 * @return The counts as an int
 *
 */
extern int readMotorRawRelative(int motor_number);

/**
 * @brief Reads value in encoderVal array for motors and keeps track of multiturn angles
 *
 * This function reads out the angle and keeps track of it for multiple turns.
 *
 * @param The motor number
 * @return The angle as an int
 *
 */
extern float readMotorAngleRelative(int motor_number);

/**
 * @brief Reads the motors' speed in terms of revolutions/sec from the encoder
 *
 * @param The motor number
 * @return float the speed of the motor in rev/s
 */
extern float readMotorSpeed(int motor_number);

extern float readMotorRadRelative(int motor_number);

extern float readMotorRad(int motor_number);

#endif /* ENCODER_H_ */
