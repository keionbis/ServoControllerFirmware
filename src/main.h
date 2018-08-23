#ifndef MAIN_H
#define MAIN_H
#include "mbed.h"


//PWM Pulse width and period
#define PWMPulseWidth_us 0.1;
#define PWmPriod_us 2;

//Motor PWM pins
#define In1Pin PA_15
#define In2Pin PB_1
#define D1Pin PB_3
#define D2Pin PA_12
//General SPI pins
#define MOSIPin PA_7
#define MISOPin PA_6
#define SCKPin PA_5

#define CommsCSPin PA_4 //Chip select line for Communications

#define EnoderCSPin PB_7 //Chip select line for Encoder

#define FBPin PB_0 //current FB pin from motor ontroller

#define signaling PA_0 //Led Signaling pin




#endif
