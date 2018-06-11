/**
 * @file Utilities.h
 * @brief utilities header
 *
 * This file contains the communication buffer and mode functions
 *
 * @author Benjamen Lim
 * @author Huan Weng
 * @author Blake Strebel
 *
 */

#ifndef UTILITIES_H_
#define UTILITIES_H_

#define DECIMATION 10    // Decimate the data if necessary
#define REFERENCE_DATA 10000    // Reference data for trajectory tracking
#define BUFLEN 10000 // Actual data; sent to client using circular buffer

/**
 * Data structure containing the modes of the system
 */
typedef enum {
	IDLE, /**< Sets motor effort to 0 */
	PWM, /**< PWM directly set by user */
    HOLD, /**< Sets motor effort to holding position */
    TRACK, /**< Sets motor effort to tracking mode */
	ITRACK, /**< Track reference current */
	ITEST, /**< Test current control */
	ISENSE, /**< Read current for N samples */
	ICALIB /**< Calibrate current sensing offsets */
	} mode;    // define data structure containing modes


/**
 * Data structure containing the control data
 */
typedef struct {                          
    int refPos[REFERENCE_DATA]; /**< The reference position of the motor */
    int actPos[BUFLEN];  /**< The actual position of the motor */
    int u[BUFLEN];  /**< The control effort of the motor */
} control_data_t;

/**
 * @brief Recieve number of values to store in position data arrays from client
 * 
 *
 * @param Void 
 * @return Void
 */
void setNclient(int n);

/**
 * @brief Bounds a given input to a range of values
 * 
 *
 * @param int a the input value
 * @param int n the range of the bounds
 * @return int the bounded output
 */  
int boundInt(int a, int n);

/**
 * @brief returns the large value of the two
 * 
 *
 * @param int a one input value
 * @param int b the other input value
 * @return int the larger value
 */  
int maxInt(int a, int b);

/**
 * @brief Return the Mode of the robot
 * 
 * 
 *
 * @param mode operating mode of the robot
 * @return Void
 */
mode getMODE();

/**
 * @brief Sets the Mode of the robot
 * 
 * The avaliable modes are IDLE, HOLD, TRACK, PWM, ISENSE, ITEST, ITRACK, ICALIB
 *
 * @param mode operating mode of the robot
 * @return Void
 */
void setMODE(mode newMODE);


/**
 * @brief Receive number N of samples to save into data buffer
 * 
 * 
 *
 * @param Void
 * @return Void
 */
void setN(void);  

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

/**
 * @brief Returns true if the buffer is empty
 * 
 * READ == WRITE
 *
 * @param Void 
 * @return int true if buffer is empty
 */
int buffer_empty(void);             

/**
 * @brief Returns true if the buffer is full
 * 
 * (WRITE + 1) % BUFLEN == READ
 *
 * @param Void 
 * @return int true if buffer is full
 */
int buffer_full(void);              

/**
 * @brief Reads position from the current buffer location
 * 
 * This function assumes that buffer is not empty
 *
 * @param int motor the motor that is read from
 * @return int position of the current buffer location
 */
int buffer_read_position(int motor);

/**
 * @brief Reads current value from current buffer location
 * 
 * This function assumes that buffer is not empty
 *
 * @param int motor motor that is read from
 * @return int current value in buffer
 */
int buffer_read_u(int motor);

/**
 * @brief Increments the buffer read index
 * 
 *
 * @param Void 
 * @return Void
 */
void buffer_read_increment(void);   

/**
 * @brief Write data to buffer
 * 
 *
 * @param int M1_actPos motor 1 position
 * @param int M2_actPos motor 2 position
 * @param int M1_u motor 1 effort
 * @param int M1_u motor 2 effort
 * @return Void
 */
void buffer_write(int M1_actPos, int M2_actPos, int M1_u, int M2_u);

/**
 * @brief Send data to client when it becomes available
 * 
 *
 * @param Void 
 * @return Void
 */
void send_data(void);


#endif /* UTILITIES_H_ */
