/**
 * @file Encoder.c
 * @brief Encoder source code
 *
 * This file contains the encoder functions
 *
 * @author Benjamen Lim
 * @author Huan Weng
 * @author Blake Strebel
 *
 */



#include "r2r.h"
#include "Encoder.h"


// Data from encoder 2
#define NUM_SSI_DATA2           5
uint32_t pui32DataTx2[NUM_SSI_DATA2];
uint32_t pui32DataRx2[NUM_SSI_DATA2];
uint32_t ui32Index2;

// Array to store encoder data
uint32_t encoderVal[4];

// Global angle
int modifier1=0;
int last_motor_1_angle = 0;
int modifier2=0;
int last_motor_2_angle = 0;
int zeroing1 = 0;
int zeroing2 = 0;

/*
 * This function initializes the SPI on SSI3, using PA2 (CLK), PA3(SS), PA4(RX), PA5(TX)
 */
void encoderSPIInit(void){
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
                                    SSI_MODE_MASTER, 1000000, 8); // 8 bits for encoder, note that we can't go above 16 bits using SPI Freescale mode
    SSIConfigSetExpClk(SSI1_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_1,
                            SSI_MODE_MASTER, 1000000, 8); // 8 bits for encoder, note that we can't go above 16 bits using SPI Freescale mode

    // Configure the CS
    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_4,GPIO_PIN_4); //Set CS to HIGH
    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_1,GPIO_PIN_1); // Set CS to HIGH        //Is it necessary for L0?

    // Enable the SSI3 and SSI1 modules
    SSIEnable(SSI3_BASE);
    SSIEnable(SSI1_BASE);

    // Set the initial encoder values
    encoderRead();
    last_motor_1_angle = readMotor1Raw();
    last_motor_2_angle = readMotor2Raw();
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
void encoderRead(void){
    pui32DataTx2[0] = 0x73; // "t" return readhead temperature
    pui32DataTx2[1] = 0x00; // Sends empty data so that the encoder will complete sending data. (40 bits in total)
    pui32DataTx2[2] = 0x00;
    pui32DataTx2[3] = 0x00;
    pui32DataTx2[4] = 0x00;
    while(SSIDataGetNonBlocking(SSI3_BASE, &pui32DataRx2[0])){
    }

    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_4,0x00);
    SysCtlDelay(10);

    for(ui32Index2 = 0; ui32Index2 < NUM_SSI_DATA2; ui32Index2++)
    {
       SSIDataPut(SSI3_BASE, pui32DataTx2[ui32Index2]);
    }
    while(SSIBusy(SSI3_BASE))
    {
    }

    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_4,GPIO_PIN_4);
    SysCtlDelay(10);

    for(ui32Index2 = 0; ui32Index2 < NUM_SSI_DATA2; ui32Index2++)
    {
      pui32DataRx2[ui32Index2] &= 0x00FF;
      SSIDataGet(SSI3_BASE, &pui32DataRx2[ui32Index2]);
    }

    // The angle is the first 14 bits of the response.
    // TODO: check that the data RX2 is overwritten
    int num = pui32DataRx2[0]<<6;
    num = num | (pui32DataRx2[1]>>2);
    encoderVal[0] = num;
    // reading speed in rev/s * 10
    num = pui32DataRx2[2]<<8;
    num = num | pui32DataRx2[3];
    encoderVal[2] = num;

    // Read the other encoder
    while(SSIDataGetNonBlocking(SSI1_BASE, &pui32DataRx2[0]))
    {
    }
    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_1,0x00);
    SysCtlDelay(10);

    for(ui32Index2 = 0; ui32Index2 < NUM_SSI_DATA2; ui32Index2++)
    {
       SSIDataPut(SSI1_BASE, pui32DataTx2[ui32Index2]);
    }
    while(SSIBusy(SSI1_BASE)){
    }

    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_1,GPIO_PIN_1);
    SysCtlDelay(10);

    for(ui32Index2 = 0; ui32Index2 < NUM_SSI_DATA2; ui32Index2++)
    {
      pui32DataRx2[ui32Index2] &= 0x00FF;
      SSIDataGet(SSI1_BASE, &pui32DataRx2[ui32Index2]);
    }
    // The angle is the first 14 bits of the response.
    num = pui32DataRx2[0]<<6;
    num = num | (pui32DataRx2[1]>>2);
    encoderVal[1] = num;

    num = pui32DataRx2[2]<<8;
    num = num | pui32DataRx2[3];
    encoderVal[3] = num;
}

void zeroMotor1RawRelative(void){
    modifier1 = 0; // zero the modifier
    zeroing1 = 0; // zero the previous zeroing, now it is just RAW angles, now relative should be synced up with absolute
    //last_motor_1_angle = 0;
    zeroing1 = readMotor1Raw(); // read the absolute value and set it as zeroing modifier.
}
void zeroMotor2RawRelative(void){
    modifier2 = 0; // zero the modifier
    zeroing2 = 0; // zero the previous zeroing, now it is just RAW angles, now relative should be synced up with absolute
    //last_motor_2_angle = 0;
    zeroing2 = readMotor2Raw();  // read the absolute value and set it as zeroing modifier.
}

void resetMotor1RawRelative(void){
    modifier1 = 0; // zero the modifier
    zeroing1 = 0; // zero the previous zeroing, now it is just RAW angles, now relative should be synced up with absolute
}
void resetMotor2RawRelative(void){
    modifier2 = 0; // zero the modifier
    zeroing2 = 0; // zero the previous zeroing, now it is just RAW angles, now relative should be synced up with absolute
}


/*
 * This function converts the absolute encoder into a relative encoder
 *
 * Comes after:
 * sensorUpdate();
 *
 * Uses globals:
 * modifier
 * last_motor_2_angle
 *
 * TODO: CHANGE RANDOM 4000 value
 */

int readMotor1RawRelative(void){
    int angle_gap = readMotor1Raw() - last_motor_1_angle; // difference in angles
    if (angle_gap>8000){ // we crossed over a singularity
        modifier1 = modifier1 - 16383;
    }
    else if (angle_gap<-8000){
        modifier1 = modifier1 + 16383;
    }
    int relative_angle = readMotor1Raw()+modifier1-zeroing1; // add or - 360 from our relative angle - the zeroing, return the angle.
    last_motor_1_angle = readMotor1Raw(); // save the last angle for comparision the next time
    return relative_angle;
}

int readMotor2RawRelative(void){
    int angle_gap = readMotor2Raw() - last_motor_2_angle; // difference in angles
    if (angle_gap>8000){ // we crossed over a singularity
        modifier2 = modifier2 - 16383;
    }
    else if (angle_gap<-8000){ // crossed over in the opposite direction
        modifier2 = modifier2 + 16383;
    }
    int relative_angle = readMotor2Raw()+modifier2-zeroing2;//-readMotor1RawRelative(); // add or - 360 from our relative angle - the zeroing, return the angle.
    last_motor_2_angle = readMotor2Raw(); // save the last angle for comparision the next time
    return relative_angle;
}

float readMotor2Speed(void){
    return (float)((int16_t)encoderVal[3])/10.0;
}

float readMotor1Speed(void){
    return (float)((int16_t)encoderVal[2])/10.0;
}



float readMotor1AngleRelative(void){
    return ((float)readMotor1RawRelative()/16383.0)*360.0;
}
float readMotor2AngleRelative(void){
    return ((float)readMotor2RawRelative()/16383.0)*360.0;
}

float readMotor1RadRelative(void){
    return readMotor1RawRelative()/16383*2*3.14;
}
float readMotor2RadRelative(void){
    return readMotor2RawRelative()/16383*2*3.14;
}

int readMotor2Raw(void){
    return encoderVal[1];
}
int readMotor1Raw(void){
    return encoderVal[0];
}

float readMotor1Angle(void){
    return ((float)readMotor1Raw()/16383.0)*360.0;
}
float readMotor2Angle(void){
    return ((float)readMotor2Raw()/16383.0)*360.0;
}
// TODO: include M_PI here
float readMotor1Rad(void){
    return readMotor1Raw()/16383*2*3.14;
}
float readMotor2Rad(void){
    return readMotor2Raw()/16383*2*3.14;
}

