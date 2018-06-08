#include "r2r.h"
#include "Encoder.h"

#define PI          3.1415926

// Data from encoders
#define NUM_ENCODER_DATA           5
uint32_t encoderDataTx[NUM_ENCODER_DATA] = {0};
uint32_t encoderDataRx[NUM_ENCODER_DATA];
uint32_t encoderMsgIndex;

static volatile encoder_states encoder[3];

/*
 * This function initializes the SPI on SSI3, using PA2 (CLK), PA3(SS), PA4(RX), PA5(TX)
 */
void encoderSPIInit(void)
{
    // Enable SSI3 and SSI1 peripherals for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3); // SSI3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1); // SSI1

    // Enable GPIO for SSI
    // Encoder 1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    // Encoder2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // Configure SSI pins
    // Encoder 1
    GPIOPinConfigure(GPIO_PQ0_SSI3CLK);
    GPIOPinConfigure(GPIO_PQ2_SSI3XDAT0);
    GPIOPinConfigure(GPIO_PQ3_SSI3XDAT1);

    // Encoder 2
    GPIOPinConfigure(GPIO_PB5_SSI1CLK);
    GPIOPinConfigure(GPIO_PE4_SSI1XDAT0);
    GPIOPinConfigure(GPIO_PE5_SSI1XDAT1);

    // Configure the GPIO settings for the SSI pins.
    // encoder 1
    GPIOPinTypeSSI(GPIO_PORTQ_BASE, GPIO_PIN_0 | GPIO_PIN_2 | GPIO_PIN_3); // SCK/MOSI/MISO
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_4); //CS

    // encoder 2
    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_5); // SCK
    GPIOPinTypeSSI(GPIO_PORTE_BASE, GPIO_PIN_5 | GPIO_PIN_4); // MOSI/MISO
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_1); //CS

    // Configure and enable the SSI port for SPI master mode.
    SSIConfigSetExpClk(SSI3_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_1,
                       SSI_MODE_MASTER, 1500000, 8); // 8 bits for encoder, note that we can't go above 16 bits using SPI Freescale mode
    SSIConfigSetExpClk(SSI1_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_1,
                       SSI_MODE_MASTER, 1500000, 8); // 8 bits for encoder, note that we can't go above 16 bits using SPI Freescale mode

    // Configure the CS
    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_4,GPIO_PIN_4); //Set CS to HIGH
    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_1,GPIO_PIN_1); // Set CS to HIGH

    // Enable the SSI3 and SSI1 modules
    SSIEnable(SSI3_BASE);
    SSIEnable(SSI1_BASE);

    // Set the initial encoder values
    encoderRead(1);
    encoderRead(2);
    setMotorZero(1);
    setMotorZero(2);
    encoder[1].previous_count = readMotorRaw(1);
    encoder[2].previous_count = readMotorRaw(2);
}


/**
 * This function reads both encoder counts and stores them in an array
 *
 * Comes after:
 * spiInit();
 *
 * Writes to:
 * encoderVal array
 *
 * TODO: check the delay
 */
void encoderRead(int motor)
{
    encoderDataTx[0] = 0x73; // "s" return readhead temperature
    encoder[motor].previous_count = encoder[motor].position_count; // save the last angle for comparision the next time.
    if (motor == 1)
    {
        while(SSIDataGetNonBlocking(SSI3_BASE, &encoderDataRx[0])){} // clear out the SSI buffer

        GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_4,0x00); // SSI enable pin pull low to start data transfer

        SysCtlDelay(10); // need to wait before we start sending data (spec sheet > 10ns)

        for(encoderMsgIndex = 0; encoderMsgIndex < NUM_ENCODER_DATA; encoderMsgIndex++) // for the total length of the data
            SSIDataPut(SSI3_BASE, encoderDataTx[encoderMsgIndex]); // put data into the buffer

        while(SSIBusy(SSI3_BASE)) {} // don't do anything while SSI is writing to the buffer

        GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_4,GPIO_PIN_4); // SSI complete, pull enable line high

        SysCtlDelay(10);

        for(encoderMsgIndex = 0; encoderMsgIndex < NUM_ENCODER_DATA; encoderMsgIndex++)
            SSIDataGet(SSI3_BASE, &encoderDataRx[encoderMsgIndex]); // get the data that was shifted in
    }
    else
    {
        // Read the other encoder
        while(SSIDataGetNonBlocking(SSI1_BASE, &encoderDataRx[0])) {}  // clear out the SSI buffer

        GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_1,0x00); // SSI enable pin pull low to start data transfer

        SysCtlDelay(10); // need to wait before we start sending data (spec sheet > 10ns)

        for(encoderMsgIndex = 0; encoderMsgIndex < NUM_ENCODER_DATA; encoderMsgIndex++)// for the total length of the data
            SSIDataPut(SSI1_BASE, encoderDataTx[encoderMsgIndex]); // put data into the buffer

        while(SSIBusy(SSI1_BASE)){} // don't do anything while SSI is writing to the buffer

        GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_1,GPIO_PIN_1);// SSI complete, pull enable line high

        SysCtlDelay(10);

        for(encoderMsgIndex = 0; encoderMsgIndex < NUM_ENCODER_DATA; encoderMsgIndex++)
            SSIDataGet(SSI1_BASE, &encoderDataRx[encoderMsgIndex]);// get the data that was shifted in
    }
    // The first element bit shift 6 to the left combining with the next element bit shifted right by 2 to get the 14 bits encoder data.
    encoder[motor].position_count = (encoderDataRx[0] << 6) | (encoderDataRx[1] >> 2);

    // reading speed in rev/s * 10
    // bit shift 8 to the left and OR it with the next value in array
    encoder[motor].velocity_count = (encoderDataRx[2] << 8) | encoderDataRx[3];

    // Calculate the relative counts.
    int angle_gap = encoder[motor].position_count - encoder[motor].previous_count; // difference in angles

    if (angle_gap > 8000) // we crossed over a singularity, if greater than means that we went from 1 to 360, backwards, so take away counts
        encoder[motor].continuous_count -= 16384; // 14 bits = 16384 counts
    else if (angle_gap < -8000)// we crossed over a singularity, if less than means that we went from 360 to 1, forwards, so add counts
        encoder[motor].continuous_count += 16384; // 14 bits = 16384 counts
    encoder[motor].continuous_count += angle_gap; // add or - 360 from our relative angle - the zeroing, return the angle.
}

/*
 * Wrapper function to read the encoder's raw data (int), angle data (float) or optionally angle in radians (float)
 *
 * Comes after:
 * sensorUpdate()
 */

void setMotorZero(int motor)
{
    encoder[motor].continuous_count = 0; // zero the modifier
    encoder[motor].zero_count = encoder[motor].position_count; // read the absolute value and set it as zeroing modifier.
}

/*
 * This function converts the absolute encoder into a relative encoder
 *
 * Comes after:
 * sensorUpdate();
 *
 * Uses globals:
 * modifier
 * last_motor_angle[2]
 *
 * TODO: CHANGE RANDOM 4000 value
 */

int readMotorRawRelative(int motor)
{
    return encoder[motor].continuous_count;
}

float readMotorRadRelative(int motor)
{
    return encoder[motor].continuous_count * 2 * PI / 16384; // convert to radians
}

int readMotorRaw(int motor)
{
    int raw = encoder[motor].position_count - encoder[motor].zero_count;
    if (raw >= 0)
        return raw;
    else
        return 16383 + raw;
}

float readMotorRad(int motor)
{
    return readMotorRaw(motor) * 2 * PI / 16384;
}

int readMotorCounts(int motor)
{
    return encoder[motor].position_count;
}

