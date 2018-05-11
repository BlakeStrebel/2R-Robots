#include "r2r.h"


// Variables
uint32_t ledFlag = 1;
// Variables
float eint = 0;
float eprev = 0;
// PID control loop
float kp = 0.01;
float kd = 0.1;
float ki = 0.1;
float offset_angle;
char trajr[40000];
float *getfloat;
// Interrupts
// Update the STARTUP_CCS.C file when adding new interrupts!
/*
void
Timer0IntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Toggle the flag for the first timer.
    //
    //ledFlag ^= 1;
    if (ledFlag==1){
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0,0);
        ledFlag=0;
    }
    else {
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0,  GPIO_PIN_0);
        ledFlag=1;
    }


    //
    // Use the flags to Toggle the LED for this timer
    //


}
*/

int i;
float data[10000];

void
Timer1IntHandler(void)                          //This is the time interrupt
{

    //
    // Clear the timer interrupt.
    //
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    //Test
    //
    // Toggle the flag for the second timer.
    ledFlag ^= 1;
    //

    //
    // Use the flags to Toggle the LED for this timer
    //
    //GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_0, ledFlag);

    //ROM_UARTCharPutNonBlocking(UART0_BASE,
     //                          'c');


    //
    // Update the interrupt status.
    //
    IntMasterDisable();                                     //Should be at the top of this function?

    sensorUpdate(); // updates encoders



    float desired_angle = -60;
    float current_angle;
    current_angle = readMotor2AngleRelative() - offset_angle;

    //float error = desired_angle - current_angle;
    float error = (*getfloat++) /3.1415926 * 180 - current_angle;

    //UARTprintf("Desired_angle: %d \t Current Angle: %d\n",desired_angle,(int)current_angle);

    eint = eint+error;
    // Integration clamp
    if(eint>300){
        eint=300;
    }
    else if(eint<-300){
        eint=-300;
    }
    float control = kp*error+kd*eprev+ki*eint;
    // Control signal clamp
    if (control>1800){
        control = 1800;
    }
    else if (control<-1800){
        control = -1800;
    }
    //UARTprintf("Motor 2: %d \t Motor 2 angle: %d \t Motor 2 control: %d\n",readMotor2Raw(),(int)readMotor2Angle(),(int)control);
    motor1ControlPWM((int)control);
    motor2ControlPWM((int)control);
    eprev=error;

    data[i++] = current_angle;
    GPIOPinWrite(GPIO_PORTQ_BASE, GPIO_PIN_1, 0);
    if (i < 10000)

        IntMasterEnable();

}


int main()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
    GPIOPinTypeGPIOOutput(GPIO_PORTQ_BASE,GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTQ_BASE,GPIO_PIN_1,GPIO_PIN_1);


    char traj[2000];
    char recev[12];
    uint8_t *array;

    uint8_t x;


    // Init code
    r2rDefaultInit();

    // Interrupts

    // Move the motor to show that it is working
    int numcount = 0;
    while(numcount<1000000){
        motor2ControlPWM(1500);
        motor1PWM(1500);
        //motor1PWM(0);
        numcount++;
    }
    motor2PWM(1); // stop the motor.
    motor1PWM(1); // stop the motor.



   for (i = 0; i < 2000; i++){
       recev[i] = 0;
   }

    // Infinite loop

   i = 0;
    while(1){
        while(UARTCharsAvail(UART0_BASE)){
            for (i = 0; i < 4 * 3; i++) {
                recev[i] = UARTCharGet(UART0_BASE);
            }
            getfloat = &recev;
            kp = *getfloat++;
            ki = *getfloat++;
            kd = *getfloat;
            //GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_0, 1);
            UARTCharPut(UART0_BASE,'p');
            while(!UARTCharsAvail(UART0_BASE)){}

            for (i = 0; i < 4 * 10000; i++) {
                trajr[i] = UARTCharGet(UART0_BASE);
            }
            getfloat = &trajr;

            UARTCharPut(UART0_BASE,'p');
            while(!UARTCharsAvail(UART0_BASE)){}


            //GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_0, 1);
            if (UARTCharGet(UART0_BASE) == 's'){
               offset_angle = readMotor2AngleRelative();
               i = 0;
               IntMasterEnable();

            }

        }
        if (i == 10000){

            motor2PWM(1);
            motor1PWM(1);
            GPIOPinWrite(GPIO_PORTQ_BASE, GPIO_PIN_1, 1);
            array = &data;
            for (i = 0; i < 10000 * 4; i++){
                UARTCharPut(UART0_BASE, *array++);
            }
        }






    }


    return(0);
}

