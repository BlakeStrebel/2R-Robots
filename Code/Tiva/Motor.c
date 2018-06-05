#include "System.h"
#include "Motor.h"
#include "Utilities.h"


#define NUM_SSI_DATA            8
// Data from motor driver 2
uint32_t pui32DataTx[NUM_SSI_DATA] = {0x00};
uint32_t pui32DataRx[NUM_SSI_DATA] = {0x00};
uint32_t ui32Index;

#define PWMPERIOD 4000

int error_state;

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
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, PWMPERIOD); // N = (1 / f) * SysClk --> (1 / 30000Hz) * 120MHz = 4000 cycles
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, PWMPERIOD); // N = (1 / f) * SysClk --> (1 / 30000Hz) * 120MHz = 4000 cycles
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, PWMPERIOD); // N = (1 / f) * SysClk --> (1 / 30000Hz) * 120MHz = 4000 cycles
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, PWMPERIOD); // N = (1 / f) * SysClk --> (1 / 30000Hz) * 120MHz = 4000 cycles
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
    GPIOIntClear(M1H_PORT, M1H_PINS);

    // Update Hall States
    M1H_HALLS = GPIOPinRead(M1H_PORT, M1H_PINS);

    // Update PWM outputs
    motor1ControlPWM(M1_PWM);
}

void M2HIntHandler(void)
{
    GPIOIntClear(M2H_PORT, M2H_PINS);

    // Update Hall States
    M2H_HALLS = GPIOPinRead(M2H_PORT, M2H_PINS);

    // Update PWM outputs
    motor2ControlPWM(M2_PWM);
}

void motor1ControlPWM(int control)
{
    //static char buffer[10];
    //static int i = 0;
    //i++;

    IntMasterDisable();
    M1_PWM = control; // Update global variable containing PWM value
    IntMasterEnable();



    control = abs(control); // PWM must be positive

    if (M1_PWM > 0) // CCW
    {
        switch (M1H_HALLS)
        {
            case M1_HALLSTATE_0:
            {
                //sprintf(buffer, "%d %d %d\r\n", 0, M1_PWM, i);

                M1_INL_WRITE(0, 1, 1);
                motor1PWM(0, control, 0);
                break;
            }
            case M1_HALLSTATE_1:
            {
                //sprintf(buffer, "%d %d %d\r\n", 1, M1_PWM, i);

                M1_INL_WRITE(1, 0, 1);
                motor1PWM(control, 0, 0);
                break;
            }
            case M1_HALLSTATE_2:
            {
                //sprintf(buffer, "%d %d %d\r\n", 2, M1_PWM, i);

                M1_INL_WRITE(1, 1, 0);
                motor1PWM(control, 0, 0);
                break;
            }
            case M1_HALLSTATE_3:
            {
                //sprintf(buffer, "%d %d %d\r\n", 3, M1_PWM, i);

                M1_INL_WRITE(0, 1, 1);
                motor1PWM(0, 0, control);
                break;
            }
            case M1_HALLSTATE_4:
            {
                //sprintf(buffer, "%d %d %d\r\n", 4, M1_PWM, i);

                M1_INL_WRITE(1, 0, 1);
                motor1PWM(0, 0, control);
                break;
            }
            case M1_HALLSTATE_5:
            {
                //sprintf(buffer, "%d %d %d\r\n", 5, M1_PWM, i);

                M1_INL_WRITE(1, 1, 0);
                motor1PWM(0, control, 0);
                break;
            }
            default :
            {
                //sprintf(buffer, "Check Hall Wiring\r\n");
                // You should not end up here, check hall wiring
            }
        }
    }
    else if (M1_PWM < 0) // CW
    {
        switch (M1H_HALLS)
        {
            case M1_HALLSTATE_3:
            {
                //sprintf(buffer, "%d %d %d\r\n", 3, M1_PWM, i);

                M1_INL_WRITE(0, 1, 1);
                motor1PWM(0, control, 0);
                break;
            }
            case M1_HALLSTATE_4:
            {
                //sprintf(buffer, "%d %d %d\r\n", 4, M1_PWM, i);

                M1_INL_WRITE(1, 0, 1);
                motor1PWM(control, 0, 0);
                break;
            }
            case M1_HALLSTATE_5:
            {
                //sprintf(buffer, "%d %d %d\r\n", 5, M1_PWM, i);

                M1_INL_WRITE(1, 1, 0);
                motor1PWM(control, 0, 0);
                break;
            }
            case M1_HALLSTATE_0:
            {
                //sprintf(buffer, "%d %d %d\r\n", 0, M1_PWM, i);

                M1_INL_WRITE(0, 1, 1);
                motor1PWM(0, 0, control);
                break;
            }
            case M1_HALLSTATE_1:
            {
                //sprintf(buffer, "%d %d %d\r\n", 1, M1_PWM, i);

                M1_INL_WRITE(1, 0, 1);
                motor1PWM(0, 0, control);
                break;
            }
            case M1_HALLSTATE_2:
            {
                //sprintf(buffer, "%d %d %d\r\n", 2, M1_PWM, i);

                M1_INL_WRITE(1, 1, 0);
                motor1PWM(0, control, 0);
                break;
            }
            default :
            {
                // You should not end up here, check hall wiring
            }
        }
    }
    else
    {
        //sprintf(buffer, "zero\r\n");

        motor1PWM(0, 0, 0); // Apply zero current, Alternatively, break using M1_INL_BREAK
    }

    //UART0write(buffer);
}

void motor2ControlPWM(int control)
{
    //static char buffer[10];
    //static int i = 0;
    //i++;

    IntMasterDisable();
    M2_PWM = control; // Update global variable containing PWM value
    IntMasterEnable();

    control = abs(control); // PWM must be positive

    if (M2_PWM > 0) // CCW
    {
        switch (M2H_HALLS)
        {
            case M2_HALLSTATE_0:
            {
                //sprintf(buffer, "%d %d %d\r\n", 0, M2_PWM, i);

                M2_INL_WRITE(0, 1, 1);
                motor2PWM(0, control, 0);
                break;
            }
            case M2_HALLSTATE_1:
            {
                //sprintf(buffer, "%d %d %d\r\n", 1, M2_PWM, i);

                M2_INL_WRITE(1, 0, 1);
                motor2PWM(control, 0, 0);
                break;
            }
            case M2_HALLSTATE_2:
            {
                //sprintf(buffer, "%d %d %d\r\n", 2, M2_PWM, i);

                M2_INL_WRITE(1, 1, 0);
                motor2PWM(control, 0, 0);
                break;
            }
            case M2_HALLSTATE_3:
            {
                //sprintf(buffer, "%d %d %d\r\n", 3, M2_PWM, i);

                M2_INL_WRITE(0, 1, 1);
                motor2PWM(0, 0, control);
                break;
            }
            case M2_HALLSTATE_4:
            {
                //sprintf(buffer, "%d %d %d\r\n", 4, M2_PWM, i);

                M2_INL_WRITE(1, 0, 1);
                motor2PWM(0, 0, control);
                break;
            }
            case M2_HALLSTATE_5:
            {
                //sprintf(buffer, "%d %d %d\r\n", 5, M2_PWM, i);

                M2_INL_WRITE(1, 1, 0);
                motor2PWM(0, control, 0);
                break;
            }
            default :
            {
                // You should not end up here, check hall wiring
            }
        }
    }
    else if (M2_PWM < 0) // CW
    {
        switch (M2H_HALLS)
        {
            case M2_HALLSTATE_3:
            {
                //sprintf(buffer, "%d %d %d\r\n", 3, M2_PWM, i);

                M2_INL_WRITE(0, 1, 1);
                motor2PWM(0, control, 0);
                break;
            }
            case M2_HALLSTATE_4:
            {
                //sprintf(buffer, "%d %d %d\r\n", 4, M2_PWM, i);

                M2_INL_WRITE(1, 0, 1);
                motor2PWM(control, 0, 0);
                break;
            }
            case M2_HALLSTATE_5:
            {
                //sprintf(buffer, "%d %d %d\r\n", 5, M2_PWM, i);

                M2_INL_WRITE(1, 1, 0);
                motor2PWM(control, 0, 0);
                break;
            }
            case M2_HALLSTATE_0:
            {
                //sprintf(buffer, "%d %d %d\r\n", 0, M2_PWM, i);

                M2_INL_WRITE(0, 1, 1);
                motor2PWM(0, 0, control);
                break;
            }
            case M2_HALLSTATE_1:
            {
                //sprintf(buffer, "%d %d %d\r\n", 1, M2_PWM, i);

                M2_INL_WRITE(1, 0, 1);
                motor2PWM(0, 0, control);
                break;
            }
            case M2_HALLSTATE_2:
            {
                //sprintf(buffer, "%d %d %d\r\n", 2, M2_PWM, i);

                M2_INL_WRITE(1, 1, 0);
                motor2PWM(0, control, 0);
                break;
            }
            default :
            {
                // You should not end up here, check hall wiring
            }
        }
    }
    else
    {
        //sprintf(buffer, "zero\r\n");


        motor2PWM(0, 0, 0); // Apply zero current, Alternatively, break using M2_INL_BREAK
    }

    //UART0write(buffer);
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
 * This function sets up the pwm on pins PF1,PF2,PF3 which are avaliable on the TM4C123 Launchpad as RGB outputs.
 * TODO: Change PWM output pins
 */
void pwmInit(void)
{
    // Set the PWM clock to the system clock.
    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    // The PWM peripheral must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    // Enable GPIO for PWM.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);

    // Configure the GPIO pin muxing to select PWM functions for these pins.
    GPIOPinConfigure(GPIO_PF0_M0PWM0);
    GPIOPinConfigure(GPIO_PG0_M0PWM4);

    // Configure the GPIO pad for PWM function on pins PF0 and PG0.
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0);
    GPIOPinTypePWM(GPIO_PORTG_BASE, GPIO_PIN_0);

    // Configure the PWM0 to count up/down without synchronization.
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_UP_DOWN |
                    PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN |
                    PWM_GEN_MODE_NO_SYNC);

    //
    // Set the PWM period to 12500Hz.  To calculate the appropriate parameter
    // use the following equation: N = (1 / f) * SysClk.  Where N is the
    // function parameter, f is the desired frequency, and SysClk is the
    // system clock frequency.
    // In this case you get: (1 / 12500Hz) * 120MHz = 9600 cycles.  Note that
    // the maximum period you can set is 2^16 - 1.
    // TODO: Here is a tradeoff. Ideally should be larger than 1KHz.
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 9600);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 9600);

    // Set PWM0 PD0 to a duty cycle of 0.
    //TODO: Test if this can be 0.
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 0);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 0);

    // Enable the PWM0 Bit 0 and Bit 4 output signals.
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_4_BIT, true);

    // Enables the counter for a PWM generator block.
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
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
    pui32DataTx[0] = 0b0001000000100000; // set register 3, bit 6 and 5 to 10, option 3, 1x PWM mode

    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_2,0x00); // Pull CS pin low
    SysCtlDelay(1000);
    SSIDataPut(SSI2_BASE, pui32DataTx[0]); // Send data
    SysCtlDelay(1000); // wait (at least 50ns)
    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_2,GPIO_PIN_2); // Being the CS pin high
    SSIDataGet(SSI2_BASE, &pui32DataRx[0]); // Get the data
    SysCtlDelay(1000);


    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_3,0x00); // Pull CS pin low
    SysCtlDelay(1000);  //50?
    SSIDataPut(SSI2_BASE, pui32DataTx[0]); // Send data
    while(SSIBusy(SSI2_BASE)){}
    SysCtlDelay(1000); // wait (at least 50ns)  50?
    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_3,GPIO_PIN_3); // Being the CS pin high
    SSIDataGet(SSI2_BASE, &pui32DataRx[0]); // Get the data
    SysCtlDelay(1000);
}

/*
 * Wrapper function that turns on the brakes for the motor and sets pwm to 0
 */
void motorBrake(int motor_number)
{
    motorControlPWM(motor_number, 0);
    if (motor_number == 1)
    {
        GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_0,GPIO_PIN_0); // forward direction
        GPIOPinWrite(GPIO_PORTP_BASE,GPIO_PIN_4,0); // Set brake pin to low, brake!
    }
    else
    {
        GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_2,GPIO_PIN_2); // forward direction
        GPIOPinWrite(GPIO_PORTP_BASE,GPIO_PIN_5,0); // Set brake pin to low, brake!
    }

}

/*
 * Wrapper function that takes in a control signal for the 1xPWM mode
 * if control is zero = float!
 *      // Setting up motor driver pins,
       // Motor directions PK0:1, PK2:2
       // Motor enable pins, PK1: 1, PK3: 2
       // Motor brake pins, PP4: 1, PP5: 2
 */
void motorControlPWM(int motor_number, int control)
{
    if (motor_number == 1)
    {
        GPIOPinWrite(GPIO_PORTP_BASE,GPIO_PIN_4,GPIO_PIN_4); // no braking
        if (control >= 0)
        {
            // Positive direction
            GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_0,GPIO_PIN_0); // Set to HIGH  - forward
            motorPWM(1, control); // set the pwm
        }
        else
        {
            // Negative direction
            GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_0,0); // set to LOW - reverse
            motorPWM(1, -1*control); // set the pwm, but since control is negative, flip the sign
        }
    }
    else
    {
        GPIOPinWrite(GPIO_PORTP_BASE,GPIO_PIN_5,GPIO_PIN_5); // Set to HIGH - no braking
        if (control >= 0)
        {
            // Positive direction
            GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_2,GPIO_PIN_2); // Set to HIGH  - forward
            motorPWM(2, control); // set the pwm
        }
        else
        {
            // Negative direction
            GPIOPinWrite(GPIO_PORTK_BASE,GPIO_PIN_2, 0); // set to LOW - reverse
            motorPWM(2, -1*control); // set the pwm, but since control is negative, flip the sign
        }
    }
}

/*
 * Wrapper function to write a PWM value to the motor
 *
 * Comes after:
 * motorInit()
 * spiInit()
 * motorDriverInit()
 */

void motorPWM(int motor_number, int pwmValue)
{
    // assuming PWM has been initialized.
    if (motor_number == 1)
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,pwmValue); // set pwm
    else
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4,pwmValue); // set pwm
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
