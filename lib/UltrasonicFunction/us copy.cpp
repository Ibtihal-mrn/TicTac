#include "us.h"
#include <Ultrasonic.h>
#include "../../src/config.h"
#include "../utils/Debug.h"

// ...existing code...

Ultrasonic us1(US_TRIG_PIN,  US_ECHO_PIN,  US_TIMEOUT);
Ultrasonic us2(US2_TRIG_PIN, US2_ECHO_PIN, US_TIMEOUT);
Ultrasonic us3(US3_TRIG_PIN, US3_ECHO_PIN, US_TIMEOUT);

int obstcle_threshold_cm = US_OBSTACLE_THRESHOLD_CM;

static int16_t dUS1 = -1, dUS2 = -1, dUS3 = -1;
static uint8_t nextSensor = 0;
static unsigned long lastReadMs = 0;
static const uint16_t US_INTER_SENSOR_MS = 30; // anti-crosstalk sans delay bloquant

static int16_t readOne(Ultrasonic& us) {
  int d = (int)us.read(CM);
  if (d <= 0 || d > 400) return -1;
  return (int16_t)d;
}

static int16_t minValid(int16_t a, int16_t b) {
  if (a < 0) return b;
  if (b < 0) return a;
  return (a < b) ? a : b;
}

static int16_t minValid3(int16_t a, int16_t b, int16_t c) {
  return minValid(minValid(a, b), c);
}

void ultrasonic_update() {
  const unsigned long now = millis();
  if (now - lastReadMs < US_INTER_SENSOR_MS) return;

  switch (nextSensor) {
    case 0: dUS1 = readOne(us1); break;
    case 1: dUS2 = readOne(us2); break;
    case 2: dUS3 = readOne(us3); break;
  }

  nextSensor = (nextSensor + 1) % 3;
  lastReadMs = now;
}

bool ultrasonic_isObstacle() {
  ultrasonic_update(); // maintient les mesures fraîches
  int16_t dmin = minValid3(dUS1, dUS2, dUS3);
  return (dmin > 0 && dmin <= obstcle_threshold_cm);
}


int16_t ultrasonic_read() {
  ultrasonic_update();
  return minValid3(dUS1, dUS2, dUS3);
}

void printUltrasonicVal() {
  static unsigned long lastPrint = 0;
  ultrasonic_update();

  if (millis() - lastPrint < 500) return;
  lastPrint = millis();

  int16_t dmin = minValid3(dUS1, dUS2, dUS3);
  // Serial.print("Safety triggered\n");
  Serial.print("US1: "); Serial.print(dUS1);
  Serial.print(" | US2: "); Serial.print(dUS2);
  Serial.print(" | US3: "); Serial.print(dUS3);
  Serial.print(" | MIN: "); Serial.println(dmin);
}