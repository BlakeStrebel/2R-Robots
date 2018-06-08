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

#include "math.h"

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
 * @param Void
 * @return Void
 *
 */
extern void encoderRead(int motor);

/**
 * @brief Reads value of one motor encoder and sets the relative angle to zero
 *
 * @param Void
 * @return Void
 *
 */
extern void setMotorZero(int motor);

/**
 * @brief Reads value for one motor
 *
 * The range for the read angle is 0 to 16383
 *
 * @param Void
 * @return The angle as an int
 *
 */
extern int readMotorRaw(int motor);

/**
 * @brief Reads value for one motor and keeps track of multiturn counts
 *
 * This function reads out the angle and keeps track of it for multiple turns. The value
 * ranges from -2,147,483,648 to 2,147,483,648. Each turn is 16383 counts
 *
 * @param Void
 * @return The counts as an int
 *
 */
extern int readMotorRawRelative(int motor);


extern float readMotorRadRelative(int motor_number);

extern float readMotorRad(int motor_number);

extern int readMotorCounts(int motor_number);

#endif /* ENCODER_H_ */
