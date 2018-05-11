#include "System.h"
#include "Encoder.h"


// Data from encoder 2
#define NUM_SSI_DATA2           5
uint32_t pui32DataTx2[NUM_SSI_DATA2];
uint32_t pui32DataRx2[NUM_SSI_DATA2];
uint32_t ui32Index2;

// Array to store encoder data
uint32_t encoderVal[2];

// Global angle
int modifier1=0;
int last_motor_1_angle = 0;
int modifier2=0;
int last_motor_2_angle = 0;

/*
 * This function initilizes the SPI on SSI0, using PA2 (CLK), PA3(SS), PA4(RX), PA5(TX)
 */
void enoderSPIinit(void){
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
    SSIConfigSetExpClk(SSI0_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_1,
                                    SSI_MODE_MASTER, 1000000, 8); // 8 bits for encoder, note that we can't go above 16 bits using SPI Freescale mode
    SSIConfigSetExpClk(SSI1_BASE, ui32SysClock, SSI_FRF_MOTO_MODE_1,
                            SSI_MODE_MASTER, 1000000, 8); // 8 bits for encoder, note that we can't go above 16 bits using SPI Freescale mode

    // Configure the CS
    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_1,GPIO_PIN_1); // Set CS to HIGH        //Is it necessary for L0?
    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_2,GPIO_PIN_2); //Set CS to HIGH

    // Enable the SSI0 and SSI1 modules
    SSIEnable(SSI0_BASE);
    SSIEnable(SSI1_BASE);

    // Set the initial encoder values
    encoderRead();
    last_motor_1_angle = readMotor1Raw();
    last_motor_2_angle = readMotor2Raw();
}


/*
 * This function reads both encoder counts and stores them in an array
 /*
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
    pui32DataTx2[0] = 0x74; // "t" return readhead temperature
    pui32DataTx2[1] = 0x00; // Sends empty data so that the encoder will complete sending data. (40 bits in total)
    pui32DataTx2[2] = 0x00;
    pui32DataTx2[3] = 0x00;
    pui32DataTx2[4] = 0x00;
    while(SSIDataGetNonBlocking(SSI0_BASE, &pui32DataRx2[0])){
    }

    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_0,0x00);
    SysCtlDelay(10);

    for(ui32Index2 = 0; ui32Index2 < NUM_SSI_DATA2; ui32Index2++)
    {
       SSIDataPut(SSI0_BASE, pui32DataTx2[ui32Index2]);
    }
    while(SSIBusy(SSI0_BASE))
    {
    }

    GPIOPinWrite(GPIO_PORTL_BASE,GPIO_PIN_0,GPIO_PIN_0);
    SysCtlDelay(10);

    for(ui32Index2 = 0; ui32Index2 < NUM_SSI_DATA2; ui32Index2++)
    {
      pui32DataRx2[ui32Index2] &= 0x00FF;
      SSIDataGet(SSI0_BASE, &pui32DataRx2[ui32Index2]);
    }

    // The angle is the first 14 bits of the response.
    // TODO: check that the data RX2 is overwritten
    int num = pui32DataRx2[0]<<6;
    num = num | (pui32DataRx2[1]>>2);
    encoderVal[0] = num;

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
}


/*
 * Wrapper function to read the encoder's raw data (int), angle data (float) or optionally angle in radians (float)
 *
 * Comes after:
 * sensorUpdate()
 */

int angleFix(int curr_angle){
    int output_angle = 0;
    if ((curr_angle<90) || (curr_angle>270)){
        output_angle = curr_angle - 180;

    }
    else {
        output_angle = curr_angle;
    }
    return output_angle;

}


<<<<<<< HEAD
=======
void zeroMotor1RawRelative(void){
    modifier1 = 0; // zero the modifier
    zeroing1 = 0; // zero the previous zeroing, now it is just RAW angles, now relative should be synced up with absolute
    last_motor_1_angle = 0;
    zeroing1 = readMotor1RawRelative(); // read the absolute value and set it as zeroing modifier.
}
void zeroMotor2RawRelative(void){
    modifier2 = 0; // zero the modifier
    zeroing2 = 0; // zero the previous zeroing, now it is just RAW angles, now relative should be synced up with absolute
    last_motor_2_angle = 0;
    zeroing2 = readMotor2RawRelative();  // read the absolute value and set it as zeroing modifier.
}


>>>>>>> parent of 7bcd163... Fixed adc code, fixed zeroing code
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
    int angle_gap = readMotor1Raw() - last_motor_1_angle;
    if (angle_gap>8000){
        modifier1 = modifier1 - 16383;
    }
    else if (angle_gap<-8000){
        modifier1 = modifier1 + 16383;
    }
    int relative_angle = readMotor1Raw()+modifier1;
    last_motor_1_angle = readMotor1Raw();
    return relative_angle;
}

int readMotor2RawRelative(void){

    int angle_gap = readMotor2Raw() - last_motor_2_angle; // difference in angles
    if (angle_gap>4000){ // we crossed over a singularity
        modifier2 = modifier2 - 16383;
    }
    else if (angle_gap<-4000){ // crossed over in the opposite direction
        modifier2 = modifier2 + 16383;
    }
    int relative_angle = readMotor2Raw()+modifier2-zeroing2; // add or - 360 from our relative angle - the zeroing, return the angle.
    last_motor_2_angle = readMotor2Raw(); // save the last angle for comparision the next time

    return relative_angle;
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

