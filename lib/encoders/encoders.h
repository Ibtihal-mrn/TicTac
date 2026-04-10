#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>
#include "../../src/config.h"

// ---------- ENCODEURS ----------
// #define ENC_L_A 2   // in config.h
// #define ENC_L_B 5
// #define ENC_R_A 3
// #define ENC_R_B 9

extern volatile long ticksL;
extern volatile long ticksR;

extern long prevL;
extern long prevR;

void encoders_init();
void encoders_reset();

void encoders_read(long *leftTicks, long *rightTicks);
void encoders_computeDelta(long leftTicks, long rightTicks, long *leftDelta, long *rightdelta);

// ISR doivent être visibles pour attachInterrupt
void ISR_left(void);
void ISR_right(void);

void printEncodersVal();  // Debug

// ------ geometry --------
extern float wheelDiameterMm;
extern float trackWidthMm;
extern long ticksPerRevolution;

float mm_per_tick();
long ticks_for_distance_mm(float distanceMm);
long ticks_for_rotation_deg(float angleDeg);




#endif
