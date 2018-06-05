/**
 * @file CurrentControl.h
 * @brief The header for the current control file
 *
 * @author Benjamen Lim
 * @author Huan Weng
 * @author Blake Strebel
 *
 */

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
 * @brief Init Current control
 *
 * Initialises ADC with Sequence 0, capturing 8 samples and a FIFO depth of 8 32-bit words, with last 12 bits containing the conversion result
 *
 * @param Void
 * @return Void
 *
 */
extern void currentControlInit(void);







#endif /* CURRENT_CONTROL_H_ */
