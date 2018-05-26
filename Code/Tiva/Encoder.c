#include "System.h"
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
    pui32DataTx2[0] = 0x73; // "s" return readhead temperature
    pui32DataTx2[1] = 0x00; // Sends empty data so that the encoder will complete sending data. (40 bits in total)
    pui32DataTx2[2] = 0x00;
    pui32DataTx2[3] = 0x00;
    pui32DataTx2[4] = 0x00;
    while(SSIDataGetNonBlocking(SSI0_BASE, &pui32DataRx2[0])){ // clear out the SSI buffer
    }

    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_0,0x00); // SSI enable pin pull low to start data transfer
    SysCtlDelay(10); // need to wait before we start sending data (spec sheet > 10ns)

    for(ui32Index2 = 0; ui32Index2 < NUM_SSI_DATA2; ui32Index2++) // for the total length of the data
    {
       SSIDataPut(SSI0_BASE, pui32DataTx2[ui32Index2]); // put data into the buffer
    }
    while(SSIBusy(SSI0_BASE)) // don't do anything while SSI is writing to the buffer
    {
    }

    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_0,GPIO_PIN_0); // SSI complete, pull enable line high
    SysCtlDelay(10);

    for(ui32Index2 = 0; ui32Index2 < NUM_SSI_DATA2; ui32Index2++)
    {
      pui32DataRx2[ui32Index2] &= 0x00FF; // ensure that the data is 8 bit
      SSIDataGet(SSI0_BASE, &pui32DataRx2[ui32Index2]); // get the data that was shifted in
    }

    // The angle is the first 14 bits of the response.
    // TODO: check that the data RX2 is overwritten
    int num = pui32DataRx2[0]<<6; // 1 1 1 1 1 1 1 1 0 0 0 0 0 0, bit shift 6 to the left
    num = num | (pui32DataRx2[1]>>2); // 1 1 1 1 1 1 1 1 0 0 0 0 0 0 | 0 0 1 1 1 1 1 1, or it with next value in array bit shifted right by 2 to get the 14 bit encoder data
    encoderVal[0] = num;
    // reading speed in rev/s * 10
    num = pui32DataRx2[2]<<8; // bit shift 8 to the left
    num = num | pui32DataRx2[3]; // or it with the next value in array
    encoderVal[2] = num;

    // Read the other encoder
    while(SSIDataGetNonBlocking(SSI1_BASE, &pui32DataRx2[0])) // clear out the SSI buffer
    {
    }
    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_1,0x00); // SSI enable pin pull low to start data transfer
    SysCtlDelay(10); // need to wait before we start sending data (spec sheet > 10ns)

    for(ui32Index2 = 0; ui32Index2 < NUM_SSI_DATA2; ui32Index2++)// for the total length of the data
    {
       SSIDataPut(SSI1_BASE, pui32DataTx2[ui32Index2]); // put data into the buffer
    }
    while(SSIBusy(SSI1_BASE)){ // don't do anything while SSI is writing to the buffer
    }

    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_1,GPIO_PIN_1);// SSI complete, pull enable line high
    SysCtlDelay(10);

    for(ui32Index2 = 0; ui32Index2 < NUM_SSI_DATA2; ui32Index2++)
    {
      pui32DataRx2[ui32Index2] &= 0x00FF;// ensure that the data is 8 bit
      SSIDataGet(SSI1_BASE, &pui32DataRx2[ui32Index2]);// get the data that was shifted in
    }
    // The angle is the first 14 bits of the response.
    num = pui32DataRx2[0]<<6;// 1 1 1 1 1 1 1 1 0 0 0 0 0 0, bit shift 6 to the left
    num = num | (pui32DataRx2[1]>>2); // 1 1 1 1 1 1 1 1 0 0 0 0 0 0 | 0 0 1 1 1 1 1 1, or it with next value in array bit shifted right by 2 to get the 14 bit encoder data
    encoderVal[1] = num;
    // reading speed in rev/s * 10
    num = pui32DataRx2[2]<<8;  // bit shift 8 to the left
    num = num | pui32DataRx2[3];  // or it with the next value in array
    encoderVal[3] = num;
}


void zeroMotor1RawRelative(void){
    modifier1 = 0; // zero the modifier
    zeroing1 = 0; // zero the previous zeroing, now it is just RAW angles, now relative should be synced up with absolute
    zeroing1 = readMotor1Raw(); // read the absolute value and set it as zeroing modifier.
}

void zeroMotor2RawRelative(void){
    modifier2 = 0; // zero the modifier
    zeroing2 = 0; // zero the previous zeroing, now it is just RAW angles, now relative should be synced up with absolute
    zeroing2 = readMotor2Raw();  // read the absolute value and set it as zeroing modifier.
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
    if (angle_gap>8000){ // we crossed over a singularity, if greater than means that we went from 1 to 360, backwards, so take away counts
        modifier1 = modifier1 - 16383; // 14 bits = 16383 counts
    }
    else if (angle_gap<-8000){// we crossed over a singularity, if less than means that we went from 360 to 1, forwards, so add counts
        modifier1 = modifier1 + 16383;// 14 bits = 16383 counts
    }
    int relative_angle = readMotor1Raw()+modifier1-zeroing1; // add or - 360 from our relative angle - the zeroing, return the angle.
    last_motor_1_angle = readMotor1Raw(); // save the last angle for comparision the next time
    return relative_angle;
}

int readMotor2RawRelative(void){
    int angle_gap = readMotor2Raw() - last_motor_2_angle; // difference in angles
    if (angle_gap>8000){ // we crossed over a singularity, if greater than means that we went from 1 to 360, backwards, so take away counts
        modifier2 = modifier2 - 16383; // 14 bits = 16383 counts
    }
    else if (angle_gap<-8000){ // we crossed over a singularity, if less than means that we went from 360 to 1, forwards, so add counts
        modifier2 = modifier2 + 16383; // 14 bits = 16383 counts
    }
    int relative_angle = readMotor2Raw()+modifier2-zeroing2-readMotor1RawRelative(); // add or - 360 from our relative angle - the zeroing, return the angle.
    last_motor_2_angle = readMotor2Raw(); // save the last angle for comparision the next time
    return relative_angle;
}

float readMotor2Speed(void){
    return (float)((int16_t)encoderVal[3])/10.0; //we saved the speed in an array but it is 10x the actual speed, so/10
}

float readMotor1Speed(void){
    return (float)((int16_t)encoderVal[2])/10.0; // we saved the speed in an array but it is 10x the actual speed so/10
}



float readMotor1AngleRelative(void){
    return ((float)readMotor1RawRelative()/16383.0)*360.0; // convert to degrees
}
float readMotor2AngleRelative(void){
    return ((float)readMotor2RawRelative()/16383.0)*360.0; // convert to degrees
}

float readMotor1RadRelative(void){
    return readMotor1RawRelative()/16383*2*3.14; // convert to radians
}
float readMotor2RadRelative(void){
    return readMotor2RawRelative()/16383*2*3.14; // convert counts to radians
}

int readMotor2Raw(void){
    return encoderVal[1]; // read the raw encoder values
}
int readMotor1Raw(void){
    return encoderVal[0]; //read the raw encoder values
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

