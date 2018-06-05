/**
 * @file Adc.h
 * @brief header for ADC
 *
 * This file contains ADC functions for 1x PWM mode. It also contains utility functions for 3x PWM mode.
 * 
 * Because the ADC is so closely tied to the motor for the 3x PWM Mode, the ADC has already been 
 * initialised in `motorInit()` and configured. The only function that is currently being used is `setADCMux`.
 * The other functions are maintained in this file in case the user wishes to move back to 1x PWM Mode.
 *
 * @author Benjamen Lim
 * @author Huan Weng
 * @author Blake Strebel
 *
 */

#ifndef ADC_H_
#define ADC_H_

#include "includes.h"

/*
 * ADC functions
 */
extern uint32_t adcArray[8]; // This variable is defined globally

/**
 * @brief Init ADC.
 *
 * Initialises ADC with Sequence 0, capturing 8 samples and a FIFO depth of 8 32-bit words, with last 12 bits containing the conversion result
 *
 * @param Void
 * @return Void
 *
 */
extern void adcInit(void);

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
 * @brief Filters ADC value
 *
 *
 * @param Void
 * @return Void
 *
 */
extern void filterValues(void);

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


/**
 * @brief Reads current from an array
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



#endif /* ADC_H_ */
