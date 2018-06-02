#ifndef CURRENT_CONTROL_H_
#define CURRENT_CONTROL_H_


extern void CurrentControlIntHandler(void);

void PI_controller(int motor, int reference, int actual);
void setCurrent(int motor, int u);

void get_mA(void);
void get_counts(void);

void get_current_gains(void);
void set_current_gains(void);
int getCounts(int motor, int phase);
void reset_current_error(void);

void counts_read(void);


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



/**
 * @brief Reads ADC values for each current sensor, using a complementary filter for A = 0.6
 *
 *
 *
 * @param Void
 * @return Void
 *
 */
extern void adcCurrentRead(void);

/**
 * @brief Reads ADC values and stores them into an array for current and tempertature
 *
 *
 *
 * @param Void
 * @return Void
 *
 */
extern void adcRead(void);
// Wrapper functions for reading current and temperature

/**
 * @brief Reads current from an array
 *
 *
 *
 * @param Void
 * @return uint32_t current
 *
 */
extern uint32_t currentRead1(void);

/**
 * @brief Reads current from an array
 *
 *
 *
 * @param Void
 * @return uint32_t current
 *
 */
extern uint32_t currentRead2(void);

/**
 * @brief Reads temperature from an array
 *
 *
 *
 * @param Void
 * @return uint32_t temperature
 *
 */
extern int32_t  tempRead1(void);

/**
 * @brief Reads temperature from an array
 *
 *
 *
 * @param Void
 * @return uint32_t temperature
 *
 */
extern int32_t  tempRead2(void);




#endif /* CURRENT_CONTROL_H_ */
