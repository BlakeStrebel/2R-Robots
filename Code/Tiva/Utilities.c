#include "r2r.h"
#include "Utilities.h"

static volatile mode MODE;              // Operating mode

static volatile control_data_t M1;    // Struct containing data arrays
static volatile control_data_t M2;

static volatile int N = 0;                  // Number of samples to store
static volatile unsigned int READ = 0, WRITE = 0; // circular buffer indexes

void setMODE(mode newMODE)   // Set mode
{
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


int boundInt(int a, int n)
{
     if (a < -n)
         a = -n;
     else if (a > n)
         a = n;
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
