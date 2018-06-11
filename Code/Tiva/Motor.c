/**
 * @file Motor.c
 * @brief The motor source code
 *
 * This file contains the motor functions
 *
 * @author Benjamen Lim
 * @author Huan Weng
 * @author Blake Strebel
 *
 */


#include "r2r.h"
#include "Motor.h"
#include "Utilities.h"

static volatile int32_t M1H_HALLS = 0, M2H_HALLS = 0; // Hall sensor data
static volatile int M1_PWM = 0, M2_PWM = 0; // Motor PWM

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
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, PWMPERIOD);   // PWMPERIOD = (1 / f) * SysClk --> (1 / 30000Hz) * 120MHz = 4000 cycles
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, PWMPERIOD);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, PWMPERIOD);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, PWMPERIOD);
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT | PWM_OUT_2_BIT | PWM_OUT_4_BIT | PWM_OUT_5_BIT | PWM_OUT_6_BIT, true);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, 0);

    // Initialize INLx pins as outputs
    SysCtlPeripheralEnable(M1_INL_PERIPH_A);
    SysCtlPeripheralEnable(M1_INL_PERIPH_B);
    SysCtlPeripheralEnable(M1_INL_PERIPH_C);
    SysCtlPeripheralEnable(M2_INL_PERIPH_A);
    SysCtlPeripheralEnable(M2_INL_PERIPH_B);
    SysCtlPeripheralEnable(M2_INL_PERIPH_C);
    GPIOPinTypeGPIOOutput(M1_INL_PORT_A, M1_INL_PIN_A);
    GPIOPinTypeGPIOOutput(M1_INL_PORT_B, M1_INL_PIN_B);
    GPIOPinTypeGPIOOutput(M1_INL_PORT_C, M1_INL_PIN_C);
    GPIOPinTypeGPIOOutput(M2_INL_PORT_A, M2_INL_PIN_A);
    GPIOPinTypeGPIOOutput(M2_INL_PORT_B, M2_INL_PIN_B);
    GPIOPinTypeGPIOOutput(M2_INL_PORT_C, M2_INL_PIN_C);
    M1_INL_WRITE(0, 0, 0);  // Break motors on startup by default
    M2_INL_WRITE(0, 0, 0);

    // Configure hall inputs to generate interrupts on state change
    SysCtlPeripheralEnable(M1H_PERIPH);
    SysCtlPeripheralEnable(M2H_PERIPH);
    GPIOIntRegister(M1H_PORT, M1HIntHandler);
    GPIOIntRegister(M2H_PORT, M2HIntHandler);
    GPIOPinTypeGPIOInput(M1H_PORT, M1H_PINS);
    GPIOPinTypeGPIOInput(M2H_PORT, M2H_PINS);
    GPIOIntTypeSet(M1H_PORT, M1H_PINS, GPIO_BOTH_EDGES);
    GPIOIntTypeSet(M2H_PORT, M2H_PINS, GPIO_BOTH_EDGES);
    IntPrioritySet(M1H_PORT_INT, 0x00); // Highest Priority
    IntPrioritySet(M2H_PORT_INT, 0x00);
    GPIOIntEnable(M1H_PORT, M1H_PINS);
    GPIOIntEnable(M2H_PORT, M2H_PINS);

    // Initialize hall sensor values
    M1H_HALLS = GPIOPinRead(M1H_PORT, M1H_PINS);
    M2H_HALLS = GPIOPinRead(M2H_PORT, M2H_PINS);

    // Initialize motor enable pins and enable motors
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_1 | GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1 |  GPIO_PIN_3 , GPIO_PIN_3 | GPIO_PIN_1); // Enable the motors
}

void M1HIntHandler(void)
{
    GPIOIntClear(M1H_PORT, M1H_PINS);               // Clear the interrupt flag

    M1H_HALLS = GPIOPinRead(M1H_PORT, M1H_PINS);    // Update Hall States

    motor1ControlPWM(M1_PWM);                       // Update PWM outputs
}

void M2HIntHandler(void)
{
    GPIOIntClear(M2H_PORT, M2H_PINS);               // Clear the interrupt flag

    M2H_HALLS = GPIOPinRead(M2H_PORT, M2H_PINS);    // Update Hall States

    motor2ControlPWM(M2_PWM);                       // Update PWM outputs
}

void motor1ControlPWM(int control)
{
    // Update global variable containing PWM value
    IntMasterDisable();
    M1_PWM = control;
    IntMasterEnable();

    control = abs(control); // PWM must be positive

    // Use hall information and PWM direction to commutate motor
    if (M1_PWM > 0) // CCW Torque
    {
        switch (M1H_HALLS)
        {
            case M1_HALLSTATE_0:
                motor1Commutate(3, control);    // Commutation state 3
                break;
            case M1_HALLSTATE_1:
                motor1Commutate(4, control);    // Commutation state 4
                break;
            case M1_HALLSTATE_2:
                motor1Commutate(5, control);    // Commutation state 5
                break;
            case M1_HALLSTATE_3:
                motor1Commutate(0, control);    // Commutation state 0
                break;
            case M1_HALLSTATE_4:
                motor1Commutate(1, control);    // Commutation state 1
                break;
            case M1_HALLSTATE_5:
                motor1Commutate(2, control);    // Commutation state 2
                break;
        }
    }
    else if (M1_PWM < 0) // CW Torque
    {
        switch (M1H_HALLS)
        {
            case M1_HALLSTATE_0:
                motor1Commutate(0, control);    // Commutation state 0
                break;
            case M1_HALLSTATE_1:
                motor1Commutate(1, control);    // Commutation state 1
                break;
            case M1_HALLSTATE_2:
                motor1Commutate(2, control);    // Commutation state 2
                break;
            case M1_HALLSTATE_3:
                motor1Commutate(3, control);    // Commutation state 3
                break;
            case M1_HALLSTATE_4:
                motor1Commutate(4, control);    // Commutation state 4
                break;
            case M1_HALLSTATE_5:
                motor1Commutate(5, control);    // Commutation state 5
                break;
        }
    }
    else
    {
        motor1PWM(0, 0, 0); // Apply zero current
    }
}

void motor2ControlPWM(int control)
{
    // Update global variable containing PWM value
    IntMasterDisable();
    M2_PWM = control;
    IntMasterEnable();

    control = abs(control); // PWM must be positive

    // Use hall information and PWM direction to commutate motor
    if (M2_PWM > 0) // CCW Torque
    {
        switch (M2H_HALLS)
        {
            case M2_HALLSTATE_0:
                motor2Commutate(3, control);    // Commutation state 3
                break;
            case M2_HALLSTATE_1:
                motor2Commutate(4, control);    // Commutation state 4
                break;
            case M2_HALLSTATE_2:
                motor2Commutate(5, control);    // Commutation state 5
                break;
            case M2_HALLSTATE_3:
                motor2Commutate(0, control);    // Commutation state 0
                break;
            case M2_HALLSTATE_4:
                motor2Commutate(1, control);    // Commutation state 1
                break;
            case M2_HALLSTATE_5:
                motor2Commutate(2, control);    // Commutation state 2
                break;
        }
    }
    else if (M2_PWM < 0) // CW Torque
    {
        switch (M2H_HALLS)
        {
            case M2_HALLSTATE_0:
                motor2Commutate(0, control);    // Commutation state 0
                break;
            case M2_HALLSTATE_1:
                motor2Commutate(1, control);    // Commutation state 1
                break;
            case M2_HALLSTATE_2:
                motor2Commutate(2, control);    // Commutation state 2
                break;
            case M2_HALLSTATE_3:
                motor2Commutate(3, control);    // Commutation state 3
                break;
            case M2_HALLSTATE_4:
                motor2Commutate(4, control);    // Commutation state 4
                break;
            case M2_HALLSTATE_5:
                motor2Commutate(5, control);    // Commutation state 5
                break;
        }
    }
    else
    {
        motor2PWM(0, 0, 0); // Apply zero current
    }
}


void motor1Commutate(int State, int control)
{
    // Apply commutation state to the motor driver
    switch (State)
    {
        case 0: // B-->C
            M1_INL_WRITE(0, 1, 1);
            motor1PWM(0, control, 0);
            break;
        case 1: // A-->C
            M1_INL_WRITE(1, 0, 1);
            motor1PWM(control, 0, 0);
            break;
        case 2: // A-->B
            M1_INL_WRITE(1, 1, 0);
            motor1PWM(control, 0, 0);
            break;
        case 3: // C-->B
            M1_INL_WRITE(0, 1, 1);
            motor1PWM(0, 0, control);
            break;
        case 4: // C-->A
            M1_INL_WRITE(1, 0, 1);
            motor1PWM(0, 0, control);
            break;
        case 5: // B-->A
            M1_INL_WRITE(1, 1, 0);
            motor1PWM(0, control, 0);
            break;
    }
}

void motor2Commutate(int State, int control)
{
    // Apply commutation state to the motor driver
    switch (State)
    {
        case 0: // B-->C
            M2_INL_WRITE(0, 1, 1);
            motor2PWM(0, control, 0);
            break;
        case 1: // A-->C
            M2_INL_WRITE(1, 0, 1);
            motor2PWM(control, 0, 0);
            break;
        case 2: // A-->B
            M2_INL_WRITE(1, 1, 0);
            motor2PWM(control, 0, 0);
            break;
        case 3: // C-->B
            M2_INL_WRITE(0, 1, 1);
            motor2PWM(0, 0, control);
            break;
        case 4: // C-->A
            M2_INL_WRITE(1, 0, 1);
            motor2PWM(0, 0, control);
            break;
        case 5: // B-->A
            M2_INL_WRITE(1, 1, 0);
            motor2PWM(0, control, 0);
            break;
    }
}

void motor1PWM(int pwm1, int pwm2, int pwm3)
{
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, pwm1);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, pwm2);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, pwm3);
}

void motor2PWM(int pwm1, int pwm2, int pwm3)
{
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, pwm1);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, pwm2);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, pwm3);
}

int getmotor1PWM(void)
{
   return M1_PWM;
}

int getmotor2PWM(void)
{
    return M2_PWM;
}

int32_t getmotor1HALLS(void)
{
    return M1H_HALLS;
}

int32_t getmotor2HALLS(void)
{
    return M2H_HALLS;
}

/*
 * This function initializes the SPI on SSI0, using PA2 (CLK), PA3(SS), PA4(RX), PA5(TX)
 */
void MotorSPIinit(void){
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
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE,GPIO_PIN_2 | GPIO_PIN_3); // CS L2 for 1, L3 for 2

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
void motorDriverInit(void){
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

void M1_INL_WRITE(int a, int b, int c)
{
    if (a)
    {
        GPIOPinWrite(M1_INL_PORT_A, M1_INL_PIN_A, M1_INL_PIN_A);
    }
    else
    {
        GPIOPinWrite(M1_INL_PORT_A, M1_INL_PIN_A, 0);
    }

    if (b)
    {
        GPIOPinWrite(M1_INL_PORT_B, M1_INL_PIN_B, M1_INL_PIN_B);
    }
    else
    {
        GPIOPinWrite(M1_INL_PORT_B, M1_INL_PIN_B, 0);
    }

    if (c)
    {
        GPIOPinWrite(M1_INL_PORT_C, M1_INL_PIN_C, M1_INL_PIN_C);
    }
    else
    {
        GPIOPinWrite(M1_INL_PORT_C, M1_INL_PIN_C, 0);
    }
}

void M2_INL_WRITE(int a, int b, int c)
{
    if (a)
    {
        GPIOPinWrite(M2_INL_PORT_A, M2_INL_PIN_A, M2_INL_PIN_A);
    }
    else
    {
        GPIOPinWrite(M2_INL_PORT_A, M2_INL_PIN_A, 0);
    }

    if (b)
    {
        GPIOPinWrite(M2_INL_PORT_B, M2_INL_PIN_B, M2_INL_PIN_B);
    }
    else
    {
        GPIOPinWrite(M2_INL_PORT_B, M2_INL_PIN_B, 0);
    }

    if (c)
    {
        GPIOPinWrite(M2_INL_PORT_C, M2_INL_PIN_C, M2_INL_PIN_C);
    }
    else
    {
        GPIOPinWrite(M2_INL_PORT_C, M2_INL_PIN_C, 0);
    }
}
