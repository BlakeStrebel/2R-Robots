#ifndef CONTROL_H_
#define CONTROL_H_

// TODO in setup set these to zero
typedef struct {                          // Define data structure containing control data
    int Enew;
    int Eold;
    int Eint;
    int Edot;
    int desired;
    int actual;
    float u;
} control_error;


extern void timerIntInit(void);

extern void Timer1IntHandler(void);


//void positioncontrol_setup(void);                   // Setup position control module
void set_position_gains(void);                      // Set position gains
void get_position_gains(void);                      // Get position gains

int get_desired_angle(int motor);
void set_desired_angle(int angle, int motor);

void reset_pos(void);                               // Reset desired position to origin (0 um)
void reset_controller_error(void);                  // Reset the error on both controllers to be zero
void load_position_trajectory(int motor);                // Load desired position trajectory from client


int get_motor_pwm(int motor);
void PID_Controller(int reference, int actual, int motor);


#endif /* CONTROL_H_ */
