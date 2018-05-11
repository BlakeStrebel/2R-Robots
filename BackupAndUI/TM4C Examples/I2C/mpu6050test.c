#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/hw_mpu6050.h"
#include "sensorlib/mpu6050.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"

tI2CMInstance g_sI2CMSimpleInst;

void InitI2C0(void)
{
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    // Configure the pin muxing for I2C0 functions on port B2 and B3.
        GPIOPinConfigure(GPIO_PB2_I2C0SCL);
        GPIOPinConfigure(GPIO_PB3_I2C0SDA);

        // Select the I2C function for these pins.
        GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
        GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

        // Enable and initialize the I2C0 master module.  Use the system clock for
        // the I2C0 module.     // I2C data transfer rate set to 400kbps.
        I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true);

        //clear I2C FIFOs
        HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;        // Initialize the I2C master driver.
        I2CMInit(&g_sI2CMSimpleInst, I2C0_BASE, INT_I2C0, 0xff, 0xff, 120000000);
    }

    //
    // A boolean that is set when a MPU6050 command has completed.
    //

    volatile bool g_bMPU6050Done;

    //
    // The function that is provided by this example as a callback when MPU6050
    // transactions have completed.
    //


    void MPU6050Callback(void *pvCallbackData, uint_fast8_t ui8Status)
        {
    //
    // See if an error occurred.
    //
            if(ui8Status != I2CM_STATUS_SUCCESS)
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
    // The MPU6050 example.
    //

    void MPU6050Example(void)
        {
            float fAccel[3], fGyro[3];
            tI2CMInstance sI2CInst;
            tMPU6050 sMPU6050;
    //
    // Initialize the MPU6050. This code assumes that the I2C master instance
    // has already been initialized.
    //
            g_bMPU6050Done = false;
            MPU6050Init(&sMPU6050, &sI2CInst, 0x68, MPU6050Callback, 0);
            while(!g_bMPU6050Done)
                {
                }
    //
    // Configure the MPU6050 for +/- 4 g accelerometer range.
    //
            g_bMPU6050Done = false;
            MPU6050ReadModifyWrite(&sMPU6050, MPU6050_O_ACCEL_CONFIG,
                                    ~MPU6050_ACCEL_CONFIG_AFS_SEL_M,
                                    MPU6050_ACCEL_CONFIG_AFS_SEL_4G, MPU6050Callback,0);
            while(!g_bMPU6050Done)
                {
                }
    //
    // Loop forever reading data from the MPU6050. Typically, this process
    // would be done in the background, but for the purposes of this example,
    // it is shown in an infinite loop.
    //
            while(1)
            {
    //
    // Request another reading from the MPU6050.
    //
                g_bMPU6050Done = false;
                MPU6050DataRead(&sMPU6050, MPU6050Callback, 0);
                while(!g_bMPU6050Done)
                {
                }
                //
                // Get the new accelerometer and gyroscope readings.
                //
                MPU6050DataAccelGetFloat(&sMPU6050, &fAccel[0], &fAccel[1], &fAccel[2]);
                MPU6050DataGyroGetFloat(&sMPU6050, &fGyro[0], &fGyro[1], &fGyro[2]);
                //
                // Do something with the new accelerometer and gyroscope readings. Blink LEDs?
                //
            }
        }

    int main()
    {
        InitI2C0();
        MPU6050Example();
        return(0);
    }
