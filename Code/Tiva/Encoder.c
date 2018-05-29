#include "System.h"
#include "Encoder.h"


// Data from encoder 2
#define NUM_SSI_DATA2           5
uint32_t pui32DataTx2[NUM_SSI_DATA2] = {0};
uint32_t pui32DataRx2[NUM_SSI_DATA2];
uint32_t ui32Index2;

static volatile encoder_states encoder[3];

/*
 * This function initilizes the SPI on SSI0, using PA2 (CLK), PA3(SS), PA4(RX), PA5(TX)
 */
void encoderSPIInit(void)
{
    // Enable SSI0 and SSI1 peripherals for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0); // SSI0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1); // SSI1

    // Enable GPIO for SSI
    // Encoder 1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    // Encoder2
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    // Configure SSI pins
    // Encoder 1
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA4_SSI0XDAT0);
    GPIOPinConfigure(GPIO_PA5_SSI0XDAT1);
    // Encoder 2
    GPIOPinConfigure(GPIO_PB5_SSI1CLK);
    GPIOPinConfigure(GPIO_PE4_SSI1XDAT0);
    GPIOPinConfigure(GPIO_PE5_SSI1XDAT1);

    // Configure the GPIO settings for the SSI pins.
    // encoder 1
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_5); // SCK/MOSI/MISO
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_0); //CS
    // encoder 2
    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_5); // SCK
    GPIOPinTypeSSI(GPIO_PORTE_BASE, GPIO_PIN_5 | GPIO_PIN_4); // MOSI/MISO
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_1); //CS

    // Configure and enable the SSI port for SPI master mode.
    // TODO: Ideally the max bit rate is 2M, but there will be some error reading the value.
    SSIConfigSetExpClk(SSI0_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_1,
                                    SSI_MODE_MASTER, 1500000, 8); // 8 bits for encoder, note that we can't go above 16 bits using SPI Freescale mode
    SSIConfigSetExpClk(SSI1_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_1,
                            SSI_MODE_MASTER, 1500000, 8); // 8 bits for encoder, note that we can't go above 16 bits using SPI Freescale mode

    // Configure the CS
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_0, GPIO_PIN_0); // Set CS to HIGH        //Is it necessary for L0?
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_1, GPIO_PIN_1); //Set CS to HIGH

    // Enable the SSI0 and SSI1 modules
    SSIEnable(SSI0_BASE);
    SSIEnable(SSI1_BASE);

    // Set the initial encoder values
    encoderRead(1);
    encoderRead(2);
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
void encoderRead(int motor_number)
{
    pui32DataTx2[0] = 0x73; // "s" return readhead temperature

    if (motor_number == 1)
    {
        while(SSIDataGetNonBlocking(SSI0_BASE, &pui32DataRx2[0])){} // clear out the SSI buffer

        GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_0,0x00); // SSI enable pin pull low to start data transfer

        SysCtlDelay(10); // need to wait before we start sending data (spec sheet > 10ns)

        for(ui32Index2 = 0; ui32Index2 < NUM_SSI_DATA2; ui32Index2++) // for the total length of the data
            SSIDataPut(SSI0_BASE, pui32DataTx2[ui32Index2]); // put data into the buffer

        while(SSIBusy(SSI0_BASE)) {} // don't do anything while SSI is writing to the buffer

        GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_0,GPIO_PIN_0); // SSI complete, pull enable line high

        SysCtlDelay(10);

        for(ui32Index2 = 0; ui32Index2 < NUM_SSI_DATA2; ui32Index2++)
            SSIDataGet(SSI0_BASE, &pui32DataRx2[ui32Index2]); // get the data that was shifted in
    }
    else
    {
        // Read the other encoder
        while(SSIDataGetNonBlocking(SSI1_BASE, &pui32DataRx2[0])) {}  // clear out the SSI buffer

        GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_1,0x00); // SSI enable pin pull low to start data transfer

        SysCtlDelay(10); // need to wait before we start sending data (spec sheet > 10ns)

        for(ui32Index2 = 0; ui32Index2 < NUM_SSI_DATA2; ui32Index2++)// for the total length of the data
            SSIDataPut(SSI1_BASE, pui32DataTx2[ui32Index2]); // put data into the buffer

        while(SSIBusy(SSI1_BASE)){} // don't do anything while SSI is writing to the buffer

        GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_1,GPIO_PIN_1);// SSI complete, pull enable line high

        SysCtlDelay(10);

        for(ui32Index2 = 0; ui32Index2 < NUM_SSI_DATA2; ui32Index2++)
            SSIDataGet(SSI1_BASE, &pui32DataRx2[ui32Index2]);// get the data that was shifted in
    }
    // TODO: check that the data RX2 is overwritten
    // The first element bit shift 6 to the left combining with the next element bit shifted right by 2 to get the 14 bits encoder data.
    encoder[motor_number].position_count = (pui32DataRx2[0] << 6) | (pui32DataRx2[1] >> 2);

    // reading speed in rev/s * 10
    // bit shift 8 to the left and OR it with the next value in array
    encoder[motor_number].velocity_count = (pui32DataRx2[2] << 8) | pui32DataRx2[3];
}

void zeroMotorRawRelative(int motor_number)
{
    encoder[motor_number].round = 0; // zero the modifier
    encoder[motor_number].zero_count = readMotorRaw(motor_number); // read the absolute value and set it as zeroing modifier.
}

/*
 * This function converts the absolute encoder into a relative encoder
 *
 * Comes after:
 * sensorUpdate();
 *
 * Uses globals:
 * modifier
 * last_motor_angle
 *
 * TODO: CHANGE RANDOM 4000 value
 */
int readMotorRawRelative(int motor_number)
{
    int angle_gap = readMotorRaw(motor_number) - encoder[motor_number].previous_count; // difference in angles
    if (angle_gap>8000){ // we crossed over a singularity, if greater than means that we went from 1 to 360, backwards, so take away counts
        encoder[motor_number].round--; // 14 bits = 16383 counts
    }
    else if (angle_gap<-8000){// we crossed over a singularity, if less than means that we went from 360 to 1, forwards, so add counts
        encoder[motor_number].round++;// 14 bits = 16383 counts
    }
    int relative_angle = readMotorRaw(motor_number)+encoder[motor_number].round*16383-encoder[motor_number].zero_count; // add or - 360 from our relative angle - the zeroing, return the angle.
    encoder[motor_number].previous_count = readMotorRaw(motor_number); // save the last angle for comparision the next time
    return relative_angle;
}

float readMotorSpeed(int motor_number)
{
    return (float)((int16_t)encoder[motor_number].velocity_count)/10.0; // we saved the speed in an array but it is 10x the actual speed so/10
}

float readMotorAngleRelative(int motor_number)
{
    return ((float)readMotorRawRelative(motor_number)/16383.0)*360.0; // convert to degrees
}

float readMotorRadRelative(int motor_number)
{
    return readMotorRawRelative(motor_number)/16383*2*3.14; // convert to radians
}

int readMotorRaw(int motor_number)
{
    return encoder[motor_number].position_count; // read the raw encoder values
}

float readMotorAngle(int motor_number)
{
    return ((float)readMotorRaw(motor_number)/16383.0)*360.0;
}

// TODO: include M_PI here
float readMotorRad(int motor_number)
{
    return readMotorRaw(motor_number)/16383*2*3.14;
}

void zeroEncoderCount(int motor_number)
{

}

