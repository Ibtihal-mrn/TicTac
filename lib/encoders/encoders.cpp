#include "encoders.h"

volatile long ticksL = 0;
volatile long ticksR = 0;

long prevL = 0;
long prevR = 0;

// ---------- INTERRUPTIONS ----------
void ISR_left(void) {
  bool A = digitalRead(ENC_L_A);
  bool B = digitalRead(ENC_L_B);
  if (A == B) ticksL++;
  else ticksL--;
}

void ISR_right(void) {
  bool A = digitalRead(ENC_R_A);
  bool B = digitalRead(ENC_R_B);
  if (A == B) ticksR--;
  else ticksR++;
}

void encoders_init(void) {
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), ISR_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), ISR_right, CHANGE);
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


// ------ Debug ------
void printEncodersVal() {
  static unsigned long millis_print = 0;
  if(millis() - millis_print >= 2000) { 
    long left, right;
    encoders_read(&left, &right);
    Serial.print("Encoders: L=");
    Serial.print(left);
    Serial.print(" R=");
    Serial.println(right);

    millis_print = millis(); 
  }
}

