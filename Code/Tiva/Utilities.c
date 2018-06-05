#include "System.h"
#include "Utilities.h"

static volatile mode MODE;              // Operating mode

static volatile motor_control_data M[3];    // Struct containing data arrays

static volatile int N;                  // Number of samples to store
static volatile unsigned int READ = 0, WRITE = 0; // circular buffer indexes

void setMODE(mode newMODE) {  // Set mode
    MODE = newMODE;     // Update global MODE
}

mode getMODE() {  // Return mode
    return MODE;
}

void setN(int timestep)          // Recieve number of values to store in position data arrays from client
{
    N = timestep;
}

int getN(void){
    return N;                   // Return number of samples to be stored
}

void write_refPos(int position, int index, int motor)    // Write reference position to data array
{
    M[motor].refPos[index] = position;
}

int get_refPos(int index, int motor)                   // Return reference position to given index
{
    return M[motor].refPos[index];
}

int buffer_empty() {    // return true if the buffer is empty (read = write)
  return READ == WRITE;
}

int buffer_full() {     // return true if the buffer is full.
  return (WRITE + 1) % BUFLEN == READ;
}

int buffer_read_position(int motor) // reads position from current buffer location; assumes buffer not empty
{
    return M[motor].actPos[READ];
}

float buffer_read_u(int motor)    // reads current from current buffer location; assumes buffer not empty
{
    return M[motor].u[READ];
}

void buffer_read_increment() {  // increment the buffer read location
    ++READ; // increment buffer read index
    if (READ >= BUFLEN) {   // wraparound read location if necessary
        READ = 0;
    }
}

void buffer_write(int M1_actPos, int M2_actPos, float M1_u, float M2_u) {   // write data to buffer
  if(!buffer_full()) {        // if the buffer is full the data is lost
    M[1].actPos[WRITE] = M1_actPos;  // write motor position to buffer
    M[2].actPos[WRITE] = M2_actPos;
    M[1].u[WRITE] = M1_u;    // write motor effort to buffer
    M[2].u[WRITE] = M2_u;
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
        sprintf(msg,"%d %d %f %f\r\n",buffer_read_position(1), buffer_read_position(2), buffer_read_u(1), buffer_read_u(2));  // read from buffer
        UART0write(msg);                                                   // send data over uart
        buffer_read_increment();                                                // increment buffer read index
  }
}

