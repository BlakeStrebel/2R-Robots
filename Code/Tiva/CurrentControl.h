#ifndef CURRENT_CONTROL_H_
#define CURRENT_CONTROL_H_

/**
 * @brief The interrupt handler for the current control loop
 *
 *
 * @param Void
 * @return Void
 *
 */
extern void CurrentControlIntHandler(void);

/**
 * @brief Simple PI controller that uses motorxControlPWM to control the motor output
 *
 *
 * @param int motor
 * @param int reference value
 * @param int actual value
 * @return Void
 *
 */
void PI_controller(int motor, int reference, int actual);

/**
 * @brief Sets the reference current for a given motor
 *
 *
 * @param int motor
 * @param int u the reference value
 * @return Void
 *
 */
void setCurrent(int motor, int u);

/**
 * @brief Print the current in mA to UART
 *
 *
 * @param Void
 * @return Void
 *
 */
void get_mA(void);
/**
 * @brief Print the current counts to UART
 *
 *
 * @param Void
 * @return Void
 *
 */
void get_counts(void);


/**
 * @brief Print the current gains to UART
 *
 *
 * @param Void
 * @return Void
 *
 */
void get_current_gains(void);

/**
 * @brief Sets the current gains by reading the UART buffer
 *
 *
 * @param Void
 * @return Void
 *
 */
void set_current_gains(void);

int getCurrent(int motor);
int getPWM(int motor);
/**
 * @brief Sets the current gains by reading the UART buffer
 *
 *
 * @param Void
 * @return Void
 *
 */
void reset_current_error(void);

/**
 * @brief Reads the counts of the current to UART from an array
 *
 *
 * @param Void
 * @return Void
 *
 */
void counts_read(void);

/**
 * @brief Triggers the ADC reading and saves it to the TEMP array
 *
 * On the PCB you have to specify either 1 or 2 to read the phase. For the breadboard prototype the number does not matter
 *
 * @param int the muxed pins, specify 1 for phase A and 2 for phase B
 * @return Void
 *
 */
void AD0_read(int mux);


/**
 * @brief Init ADC.
 *
 * Initialises ADC with Sequence 0, capturing 8 samples and a FIFO depth of 8 32-bit words, with last 12 bits containing the conversion result
 *
 * @param Void
 * @return Void
 *
 */
extern void currentControlInit(void);

/**
 * @brief Sets the ADC MUX to read from either the current sense resistor 1, 2 or 3 from motor 1 or 2
 *
 *
 * @param Void
 * @return Void
 *
 */
extern void setADCMux(int motor,int number);



#endif /* CURRENT_CONTROL_H_ */
