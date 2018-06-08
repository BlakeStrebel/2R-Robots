#include "r2r.h"
#include "Motor.h"
#include "Utilities.h"

static volatile int motorHalls[3] = {0}; // Hall sensor data
static volatile int M_PWM[3] = {0}; // Motor PWM

void motorInit()
{
    // Initialize INHx pins as PWM outputs
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);

    GPIOPinConfigure(GPIO_PF0_M0PWM0);
    GPIOPinConfigure(GPIO_PF1_M0PWM1);
    GPIOPinConfigure(GPIO_PF2_M0PWM2);
    GPIOPinConfigure(GPIO_PG0_M0PWM4);
    GPIOPinConfigure(GPIO_PG1_M0PWM5);
    GPIOPinConfigure(GPIO_PK4_M0PWM6);

    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
    GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypePWM(GPIO_PORTK_BASE, GPIO_PIN_4);

    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN |
                                          PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN |
                                          PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN |
                                          PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN |
                                          PWM_GEN_MODE_NO_SYNC);

    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, PWMPERIOD);   // PWMPERIOD = (1 / f) * SysClk --> (1 / 30000Hz) * 120MHz = 4000 cycles
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, PWMPERIOD);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, PWMPERIOD);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, PWMPERIOD);

    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);

    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_2_BIT |
                   PWM_OUT_4_BIT | PWM_OUT_5_BIT | PWM_OUT_6_BIT, true);

    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, 0);

    // Initialize INLx pins as outputs
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);

    GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_0);
    GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_5);
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_6);
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_7);

    // Break motors on startup by default
    motorINLWrite(1, 0, 0, 0);
    motorINLWrite(2, 0, 0, 0);

    // Configure hall inputs to generate interrupts on state change
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    GPIOIntRegister(GPIO_PORTM_BASE, M1HIntHandler);
    GPIOIntRegister(GPIO_PORTN_BASE, M2HIntHandler);

    GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
    GPIOPinTypeGPIOInput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);

    GPIOIntTypeSet(GPIO_PORTM_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, GPIO_BOTH_EDGES);
    GPIOIntTypeSet(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, GPIO_BOTH_EDGES);

    IntPrioritySet(INT_GPIOM, 0x00); // Highest Priority
    IntPrioritySet(INT_GPION, 0x00);

    GPIOIntEnable(GPIO_PORTM_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
    GPIOIntEnable(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);

    // Initialize hall sensor values
    motorHalls[1] = GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
    motorHalls[2] = GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);

    // Initialize motor enable pins and enable motors
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_1 | GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1 | GPIO_PIN_3, GPIO_PIN_3 | GPIO_PIN_1); // Enable the motors
}

void M1HIntHandler(void)
{
    GPIOIntClear(GPIO_PORTM_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);               // Clear the interrupt flag

    motorHalls[1] = GPIOPinRead(GPIO_PORTM_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);    // Update Hall States

    motorControlPWM(1, M_PWM[1]);                       // Update PWM outputs
}

void M2HIntHandler(void)
{
    GPIOIntClear(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);               // Clear the interrupt flag

    motorHalls[2] = GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);    // Update Hall States

    motorControlPWM(2, M_PWM[2]);                       // Update PWM outputs
}

void motorControlPWM(int motor, int control)
{
    // Update global variable containing PWM value
    IntMasterDisable();
    M_PWM[motor] = control;
    IntMasterEnable();

    control = abs(control); // PWM must be positive

    // Use hall information and PWM direction to commutate motor
    if (M_PWM[motor] > 0) // CCW Torque
    {
        switch (motorHalls[motor])
        {
            case (GPIO_PIN_0 | GPIO_PIN_1):
                motorCommutate(motor, 3, control);    // Commutation state 3
                break;
            case GPIO_PIN_0:
                motorCommutate(motor, 4, control);    // Commutation state 4
                break;
            case (GPIO_PIN_0 | GPIO_PIN_2):
                motorCommutate(motor, 5, control);    // Commutation state 5
                break;
            case GPIO_PIN_2:
                motorCommutate(motor, 0, control);    // Commutation state 0
                break;
            case (GPIO_PIN_1 | GPIO_PIN_2):
                motorCommutate(motor, 1, control);    // Commutation state 1
                break;
            case GPIO_PIN_1:
                motorCommutate(motor, 2, control);    // Commutation state 2
                break;
        }
    }
    else if (M_PWM[motor] < 0) // CW Torque
    {
        switch (motorHalls[motor])
        {
            case (GPIO_PIN_0 | GPIO_PIN_1):
                motorCommutate(motor, 0, control);    // Commutation state 0
                break;
            case GPIO_PIN_0:
                motorCommutate(motor, 1, control);    // Commutation state 1
                break;
            case (GPIO_PIN_0 | GPIO_PIN_2):
                motorCommutate(motor, 2, control);    // Commutation state 2
                break;
            case GPIO_PIN_2:
                motorCommutate(motor, 3, control);    // Commutation state 3
                break;
            case (GPIO_PIN_1 | GPIO_PIN_2):
                motorCommutate(motor, 4, control);    // Commutation state 4
                break;
            case GPIO_PIN_1:
                motorCommutate(motor, 5, control);    // Commutation state 5
                break;
        }
    }
    else
    {
        motorPWMSet(motor, 0, 0, 0); // Apply zero current
    }
}

void motorCommutate(int motor, int state, int control)
{
    // Apply commutation state to the motor driver
    switch (state)
    {
        case 0: // B-->C
            motorINLWrite(motor, 0, 1, 1);
            motorPWMSet(motor, 0, control, 0);
            break;
        case 1: // A-->C
            motorINLWrite(motor, 1, 0, 1);
            motorPWMSet(motor, control, 0, 0);
            break;
        case 2: // A-->B
            motorINLWrite(motor, 1, 1, 0);
            motorPWMSet(motor, control, 0, 0);
            break;
        case 3: // C-->B
            motorINLWrite(motor, 0, 1, 1);
            motorPWMSet(motor, 0, 0, control);
            break;
        case 4: // C-->A
            motorINLWrite(motor, 1, 0, 1);
            motorPWMSet(motor, 0, 0, control);
            break;
        case 5: // B-->A
            motorINLWrite(motor, 1, 1, 0);
            motorPWMSet(motor, 0, control, 0);
            break;
    }
}

void motorPWMSet(int motor, int pwm1, int pwm2, int pwm3)
{
    if (motor == 1)
    {
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, pwm1);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, pwm2);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, pwm3);
    }
    else
    {
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, pwm1);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, pwm2);
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, pwm3);
    }
}


int getmotorPWM(int motor)
{
   return M_PWM[motor];
}

int getmotorHALLS(int motor)
{
    return motorHalls[motor];
}

/*
 * This function initializes the SPI on SSI0, using PA2 (CLK), PA3(SS), PA4(RX), PA5(TX)
 */
void MotorSPIInit(void)
{
    // The SSI2 peripheral must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2); // SSI2

    // Enable GPIO for SPI2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);

    // Configure Motor Pins
    GPIOPinConfigure(GPIO_PD3_SSI2CLK); //CLK
    GPIOPinConfigure(GPIO_PD1_SSI2XDAT0); // MOSI
    GPIOPinConfigure(GPIO_PD0_SSI2XDAT1); // MISO

    // Configure the GPIO settings for the SSI pins.
    GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 |  GPIO_PIN_3); // SCK/MOSI/MISO
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_2 | GPIO_PIN_3); // CS L2 for 1, L3 for 2

    // Configure and enable the SSI port for SPI master mode.
    //TODO: Test if the bit rate can be the highest 5M.
    SSIConfigSetExpClk(SSI2_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_1,
                       SSI_MODE_MASTER, 1000000, 16); // 16 bits for motor, use 1Mbps mode, use the system clock

    // Configure CS
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_2, GPIO_PIN_2); //Set CS to HIGH
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, GPIO_PIN_3); //Set CS to HIGH

    // Enable the SSI2 module.
    SSIEnable(SSI2_BASE);
}

/*
 * This function sends SPI data over to the motor driver to setup the driver for 1x PWM mode
 * Comes after:
 * Motor power is supplied
 * motorInit()
 * spiInit()
 *
 * Note that the driver will NOT be set if motor power is not supplied because the driver will not be able to read the registers over SPI, even though it responds
 */
void motorDriverInit(void)
{
    uint32_t pui32DataTx[4], pui32DataRx[4], i = 0;

    pui32DataTx[0] = 0b1001000000000000; // read register 3
    pui32DataTx[1] = 0b0001000000100000; // set register 3, bit 6 and 5 to 10, option 3, 3x PWM mode
    pui32DataTx[2]=  0b1001000000000000; // read register 3
    pui32DataTx[3]=  0b1001000000000000; // read register 3


    /***  Motor 1 ***/
    // First do a read to clear buffers, then write to set driver to 3x PWM mode, then read again to verify
    for (i = 0; i < 4; i++)
    {
        GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_3,0x00);          // Pull CS pin low
        SysCtlDelay(50);                                        // delay before sending data
        SSIDataPut(SSI2_BASE, pui32DataTx[i]);                  // Send data
        while(SSIBusy(SSI2_BASE)){}                             // wait until data sent
        SysCtlDelay(50);                                        // delay after sending data
        GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_3,GPIO_PIN_3);    // Bring the CS pin high
        SSIDataGet(SSI2_BASE, &pui32DataRx[i]);                 // Get the data
        SysCtlDelay(1000);  // Delay between writes
    }

    /***  Motor 2 ***/
    // First do a read to clear buffers, then write to set driver to 3x PWM mode
    for (i = 0; i < 4; i++)
    {
        GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_2,0x00);          // Pull CS pin low
        SysCtlDelay(50);                                        // delay before sending data
        SSIDataPut(SSI2_BASE, pui32DataTx[i]);                  // Send data
        while(SSIBusy(SSI2_BASE)){}                             // wait until data sent
        SysCtlDelay(50);                                        // delay after sending data
        GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_2,GPIO_PIN_2);    // Bring the CS pin high
        SSIDataGet(SSI2_BASE, &pui32DataRx[i]);                 // Get the data
        SysCtlDelay(1000);  // Delay between writes
    }
}

void motorINLWrite(int motor, int a, int b, int c)
{
    if (motor == 1)
    {
        if (a)
            GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_0, GPIO_PIN_0);
        else
            GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_0, 0);
        if (b)
            GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_1, GPIO_PIN_1);
        else
            GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_1, 0);
        if (c)
            GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_2, GPIO_PIN_2);
        else
            GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_2, 0);
    }
    else
    {
        if (a)
            GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_5, GPIO_PIN_5);
        else
            GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_5, 0);
        if (b)
            GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_6, GPIO_PIN_6);
        else
            GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_6, 0);
        if (c)
            GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_7, GPIO_PIN_7);
        else
            GPIOPinWrite(GPIO_PORTM_BASE, GPIO_PIN_7, 0);
    }
}
