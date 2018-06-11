/**
 * @file Utilities.c
 * @brief The source code for utilities
 *
 * This file contains the buffer and mode functions
 *
 * @author Benjamen Lim
 * @author Huan Weng
 * @author Blake Strebel
 *
 */

#include "r2r.h"
#include "Utilities.h"

static volatile mode MODE;              // Operating mode

static volatile control_data_t M1;    // Struct containing data arrays
static volatile control_data_t M2;

static volatile int N;                  // Number of samples to store
static volatile unsigned int READ = 0, WRITE = 0; // circular buffer indexes

void setMODE(mode newMODE) {  // Set mode
    MODE = newMODE;     // Update global MODE
}

mode getMODE() {  // Return mode
    return MODE;
}

void setN(void)          // Receive number of values to store in position data arrays from client
{
    char buffer[10];            // Buffer holding number of samples
    UART0read(buffer,10);  // Read number of samples from client
    sscanf(buffer,"%d",&N);     // Update global N
}

void setNclient(int n)          // Receive number of values to store in position data arrays from client
{
    N = n;
}

int getN(void){
    return N;                   // Return number of samples to be stored
}

void write_refPos(int position, int index, int motor)    // Write reference position to data array
{
    if (motor == 1)
    {
        M1.refPos[index] = position;
    }
    else if (motor == 2)
    {
        M2.refPos[index] = position;
    }
}

int get_refPos(int index, int motor)                   // Return reference position to given index
{
    int pos = 0;
    if (motor == 1)
    {
        pos = M1.refPos[index];
    }
    else if (motor == 2)
    {
        pos = M2.refPos[index];
    }
    return pos;
}

int buffer_empty() {    // return true if the buffer is empty (read = write)
  return READ == WRITE;
}

int buffer_full() {     // return true if the buffer is full.
  return (WRITE + 1) % BUFLEN == READ;
}

int buffer_read_position(int motor) {    // reads position from current buffer location; assumes buffer not empty
    int pos = 0;
    if (motor == 1)
    {
        pos = M1.actPos[READ];
    }
    else if(motor == 2)
    {
        pos = M2.actPos[READ];
    }
    return pos;
}

int buffer_read_u(int motor) {   // reads current from current buffer location; assumes buffer not empty
    int u = 0;
    if (motor == 1)
    {
        u = M1.u[READ];
    }
    else if (motor == 2)
    {
        u = M2.u[READ];
    }

    return u;
}

void buffer_read_increment() {  // increment the buffer read location
    ++READ; // increment buffer read index
    if (READ >= BUFLEN) {   // wraparound read location if necessary
        READ = 0;
    }
}

void buffer_write(int M1_actPos, int M2_actPos, int M1_u, int M2_u) {   // write data to buffer
  if(!buffer_full()) {        // if the buffer is full the data is lost
    M1.actPos[WRITE] = M1_actPos;  // write motor position to buffer
    M2.actPos[WRITE] = M2_actPos;
    M1.u[WRITE] = M1_u;    // write motor effort to buffer
    M2.u[WRITE] = M2_u;
    ++WRITE;                  // increment the write index and wrap around if necessary
    if(WRITE >= BUFLEN) {
      WRITE = 0;
    }
  }
}

void send_data(void)
{
    int sent = 0;
    char msg[50];
    sprintf(msg, "%d\r\n",getN()/DECIMATION);   // tell the client how many samples to expect
    UART0write(msg);

    for(sent = 0; sent < (N/DECIMATION); ++sent) { // send the samples to the client
        while(buffer_empty()) { ; }                                             //wait for data to be in the queue
        sprintf(msg,"%d %d %d %d\r\n",buffer_read_position(1), buffer_read_position(2), buffer_read_u(1), buffer_read_u(2));  // read from buffer
        UART0write(msg);                                                   // send data over uart
        buffer_read_increment();                                                // increment buffer read index
  }
}

int boundInt(int a, int n)
{
     if (a < -n)
     {
         a = -n;
     }
     else if (a > n)
     {
         a = n;
     }

     return a;
}

int maxInt(int a, int b)
{
    int max;
    if (a > b)
    {
        max = a;
    }
    else
    {
        max = b;
    }
    return max;
}
