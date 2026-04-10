#include "encoders.h"
#include "BLEBridge.h"
#include "Debug.h"
#include "config.h"

volatile long ticksL = 0;
volatile long ticksR = 0;

long prevL = 0;
long prevR = 0;


float wheelDiameterMm = WHEELDIAMM;
float trackWidthMm = TRACKWIDTHMM;
long ticksPerRevolution = TICKSPERREV;

static constexpr int LEFT_ENCODER_DIRECTION = +1;
static constexpr int RIGHT_ENCODER_DIRECTION = +1;

// ---------- INTERRUPTIONS ----------
void IRAM_ATTR ISR_left(void) {
  bool A = digitalRead(ENC_L_A);
  bool B = digitalRead(ENC_L_B);
  if (A == B) ticksL++;
  else ticksL--;
}

void IRAM_ATTR ISR_right(void) {
  bool A = digitalRead(ENC_R_A);
  bool B = digitalRead(ENC_R_B);
  if (A == B) ticksR--;
  else ticksR++;
}

void encoders_reset() {
    noInterrupts();
    ticksL = 0;
    ticksR = 0;
    interrupts();

    prevL = 0;
    prevR = 0;
}


void encoders_init(void) {
  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_L_A), ISR_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), ISR_right, CHANGE);

  encoders_reset();
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
  static unsigned long lastPrintMs = 0;
  if(millis() - lastPrintMs >= 1000) { 
    long left, right;
    encoders_read(&left, &right);
    bleSerial.print("Encoders: L=");
    bleSerial.print(left);
    bleSerial.print(" R=");
    bleSerial.println(right);

    lastPrintMs = millis(); 
  }
}


// ------- Helpers --------
float mm_per_tick() {
    return (PI * wheelDiameterMm) / (float)ticksPerRevolution;
}

long ticks_for_distance_mm(float distanceMm) {
    return lround(distanceMm / mm_per_tick());
}

long ticks_for_rotation_deg(float angleDeg) {
    float arcMm = PI * trackWidthMm * (fabs(angleDeg) / 360.0f);
    return ticks_for_distance_mm(arcMm);
}














