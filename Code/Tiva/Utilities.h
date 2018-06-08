/**
 * @file Utilities.h
 * @brief utilities header
 *
 * This file contains the buffer and mode functions
 *
 * @author Benjamen Lim
 * @author Huan Weng
 * @author Blake Strebel
 *
 */

#ifndef UTILITIES_H_
#define UTILITIES_H_

#define DECIMATION 10    // Decimate the data if necessary
#define REFERENCE_DATA 100    // Reference data for trajectory tracking
#define BUFLEN 100 // Actual data; sent to client using circular buffer

/**
 * Data structure containing the modes of the system
 */
typedef enum {
	IDLE, /**< Sets motor effort to 0 */
	PWM, /**< PWM directly set by user */
    HOLD, /**< Sets motor effort to holding position */
    TRACK, /**< Sets motor effort to tracking mode */
	ITRACK,
	ITEST,
	ISENSE,
	ICALIB,
    READ1,
    READ2,
    READb,
    PID1,
    PID2,
    PIDb
	} mode;    // define data structure containing modes


/**
 * Data structure containing the control data
 */
typedef struct {                          
    int refPos[REFERENCE_DATA]; /**< The reference position of the motor */
    int actPos[BUFLEN];  /**< The actual position of the motor */
    int u[BUFLEN];  /**< The control effort of the motor */
} control_data_t;

int boundInt(int a, int n);
int maxInt(int a, int b);

/**
 * @brief Return the Mode of the robot
 * 
 * 
 *
 * @param mode operating mode of the robot
 * @return Void
 */
mode getMODE();                      // Return the current operating mode

/**
 * @brief Sets the Mode of the robot
 * 
 * The avaliable modes are IDLE, HOLD, TRACK
 *
 * @param mode operating mode of the robot
 * @return Void
 */
void setMODE(mode newMODE);          // Set operating mode


/**
 * @brief Receive number N of samples to save into data buffer
 * 
 * 
 *
 * @param Void
 * @return Void
 */
void setN(int timestep);

/**
 * @brief Returns the number of samples
 * 
 * 
 *
 * @param Void
 * @return int the number of samples
 */                                  
int getN(void);                                             


/**
 * @brief Writes the reference position
 * 
 * 
 *
 * @param int position 
 * @param int index
 * @param int motor
 * @return Void
 */
void write_refPos(int position, int index, int motor);      

/**
 * @brief Gets the reference position
 * 
 * 
 *
 * @param int index
 * @param int motor
 * @return int returns the reference position
 */
int get_refPos(int index, int motor);                       






#endif /* UTILITIES_H_ */
