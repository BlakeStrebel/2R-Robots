/**
 * @file main.c
 * @brief Main file for 2 arm test
 *
 * This file is the main file for the R2R Project
 *
 * @author Benjamen Lim
 * @author Huan Weng
 *
 */



#include "r2r.h"




int main()
{

    // Init code
    r2rDefaultInit();


    while(1){
        sensorUpdate();
        UARTprintf("Encoder val is: %d\n",readMotor2RawRelative());
        motor1PWM(60);
        motor2PWM(40);


    }






    return(0);
}

