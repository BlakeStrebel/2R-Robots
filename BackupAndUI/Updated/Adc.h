/*
 * adc.h
 *
 *  Created on: May 31, 2018
 *      Author: Ben
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
