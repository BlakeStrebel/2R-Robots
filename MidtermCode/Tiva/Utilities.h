#ifndef UTILITIES_H_
#define UTILITIES_H_

#define DECIMATION 10    // Decimate the data if necessary
#define REFERENCE_DATA 10000    // Reference data for trajectory tracking
#define BUFLEN 1024 // Actual data; sent to client using circular buffer

typedef enum {IDLE, HOLD, TRACK, WATCH} mode;    // define data structure containing modes

typedef struct {                          // Define data structure containing control data
    int refPos[REFERENCE_DATA];
    int actPos[BUFLEN];
    float u[BUFLEN];
} control_data_t;

// MODE
mode getMODE();                      // Return the current operating mode
void setMODE(mode newMODE);          // Set operating mode

// NUMBER OF SAMPLES
void setN(void);                                     // Recieve number N of samples to save into data buffer
int getN(void);                                             // Returns number N of samples

// REFERENCE DATA
void write_refPos(int position, int index, int motor);      // Write reference position
int get_refPos(int index, int motor);                       // Get reference position

// ACTUAL DATA
int buffer_empty();             // return true if the buffer is empty (read = write)
int buffer_full();              // return true if the buffer is full.
int buffer_read_position();     // reads position from current buffer location; assumes buffer not empty
float buffer_read_u();    // reads current from current buffer location; assumes buffer not empty
void buffer_read_increment();   // increments buffer read index
void buffer_write(int M1_actPos, int M2_actPos, float M1_u, float M2_u);  // write data to buffer
void send_data(void);           // send data to client as it becomes available


#endif /* UTILITIES_H_ */
