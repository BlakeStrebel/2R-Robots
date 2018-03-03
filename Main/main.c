#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/hw_mpu6050.h"
#include "sensorlib/mpu6050.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/pwm.h"
#include "r2r.h"



//
// A boolean that is set when a MPU6050 command has completed.
//
volatile bool g_bMPU6050Done;

//
// I2C master instance
//
tI2CMInstance g_sI2CMSimpleInst;

// The function that is provided by this example as a callback when MPU6050
// transactions have completed.
//
void MPU6050Callback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    //
    // See if an error occurred.
    //
    if (ui8Status != I2CM_STATUS_SUCCESS)
    {
        //
        // An error occurred, so handle it here if required.
        //
    }
    //
    // Indicate that the MPU6050 transaction has completed.
    //
    g_bMPU6050Done = true;
}

//
// The interrupt handler for the I2C module.
//
void I2CMSimpleIntHandler(void)
{
    //
    // Call the I2C master driver interrupt handler.
    //
    I2CMIntHandler(&g_sI2CMSimpleInst);
}

// Sensor test code
// Reads 2 encoders
// This is a SSI sensor
void EncodersExampleTest (void){
    ;
}

// Sensor test code
// This is an I2C sensor for temperature
void MCP9600ExampleTest(void){
    // Adapted from https://ncd.io/k-type-thermocouple-mcp9600-with-arduino/
    //
    // Initialize the MCP9600. This code assumes that the I2C master instance
    // has already been initialized.
    //
    //MCP9600Init(&g_sI2CMSimpleInst,0x64);

    ;

}

// Sensor test code
// This is an analog sensor for current
void ACS825ExampleTest(void){
    ;
}




// Sensor test code
// This is an I2C sensor for acceleration
void MPU6050Example(void)
{
    float fAccel[3], fGyro[3];
    tMPU6050 sMPU6050;
    //
    // Initialize the MPU6050. This code assumes that the I2C master instance
    // has already been initialized.
    //
    g_bMPU6050Done = false;
    MPU6050Init(&sMPU6050, &g_sI2CMSimpleInst, 0x68, MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5,0);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,100);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7,0);
    }
    //
    // Configure the MPU6050 for +/- 4 g accelerometer range.
    //
    g_bMPU6050Done = false;
    MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_ACCEL_CONFIG,
        ~MPU6050_ACCEL_CONFIG_AFS_SEL_M,
        MPU6050_ACCEL_CONFIG_AFS_SEL_4G, MPU6050Callback, &sMPU6050);
    while (!g_bMPU6050Done)
    {
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5,160);
    }
    //
    // Loop forever reading data from the MPU6050. Typically, this process
    // would be done in the background, but for the purposes of this example,
    // it is shown in an infinite loop.
    //
    while (1)
    {
        //
        // Request another reading from the MPU6050.
        //
        g_bMPU6050Done = false;
        MPU6050DataRead(&sMPU6050, MPU6050Callback, &sMPU6050);
        while (!g_bMPU6050Done)
        {
        }
        //
        // Get the new accelerometer and gyroscope readings.
        //
        MPU6050DataAccelGetFloat(&sMPU6050, &fAccel[0], &fAccel[1],
            &fAccel[2]);
        MPU6050DataGyroGetFloat(&sMPU6050, &fGyro[0], &fGyro[1], &fGyro[2]);
        //
        // Do something with the new accelerometer and gyroscope readings.
        //
    }
}

int main()
{
    //uint32_t myarray[4];
    int run_program =1 ; // flag to check if code should be running
    int kp = 0;
    int kd = 0;
    int ki = 0;
    // Init code
    sysInit();
    uartInit();
    adcInit();
    //spiInit();
    motorInit();
    //i2cInit(g_sI2CMSimpleInst);

    UARTprintf("Simple Motor test with PID control\r\n");

    while(run_program == 1){
        sensorUpdate();
        PIDUpdate();
        currentRead1();
        currentRead2();
        tempRead1();
        tempRead2();

        safetyCheck();

    }
    //shutdownAll();



    return(0);
}
