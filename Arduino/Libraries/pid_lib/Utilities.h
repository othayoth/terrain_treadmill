#ifndef UTILITIES_H
#define UTILITIES_H

// Data structure containing possible modes
typedef enum {IDLE, POSITION_HOLD,VELOCITY_HOLD} mode;

// MODE
mode getMODE();
void setMODE(mode newMODE);

#endif
