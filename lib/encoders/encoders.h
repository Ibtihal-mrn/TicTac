#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>

// ---------- ENCODEURS ----------
#define ENC_L_A 2   // interruption
#define ENC_L_B 5
#define ENC_R_A 3  // interruption
#define ENC_R_B 9

extern volatile long ticksL;
extern volatile long ticksR;

extern long prevL;
extern long prevR;

void encoders_init(void);
void encoders_read(long *left, long *right);
void encoders_computeDelta(long left, long right, long *dL, long *dR);

// ISR doivent être visibles pour attachInterrupt
void ISR_left(void);
void ISR_right(void);

#endif
