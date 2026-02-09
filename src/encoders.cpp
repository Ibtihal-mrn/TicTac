#include "encoders.h"

volatile long ticksL = 0;
volatile long ticksR = 0;

long prevL = 0;
long prevR = 0;

// ---------- INTERRUPTIONS ----------
void ISR_left(void) {
  //if (digitalRead(ENC_L_A))
  ticksL++;
  // else
  //   ticksL--;
}

void ISR_right(void) {
  // if (digitalRead(ENC_R_A))
  ticksR++;
  // else
  //   ticksR--;
}

void encoders_init(void) {
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), ISR_left, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), ISR_right, RISING);
}

void encoders_read(long *left, long *right) {
  noInterrupts();
  *left = ticksL;
  *right = ticksR;
  interrupts();
}

void encoders_computeDelta(long left, long right, long *dL, long *dR) {
  *dL = left - prevL;
  *dR = right - prevR;

  prevL = left;
  prevR = right;
}
