#include "safety.h"
#include "EmergencyButton.h"
#include "ultrasonic.h"

static int g_obstacle_cm;
static uint16_t g_sonar_period;

static bool g_triggered = false;
static int g_lastDist = -1;
static unsigned long g_lastSonarMs = 0;

void safety_init(int obstacle_cm, uint16_t sonar_period_ms) {
  g_obstacle_cm = obstacle_cm;
  g_sonar_period = sonar_period_ms;
  g_triggered = false;

  emergencyButton_init();
}

void safety_update() {
  // bouton (rapide)
  if (emergencyButton_isPressed()) {
    g_triggered = true;
  }

  // ultrason (lent → throttling)
  unsigned long now = millis();
  if (now - g_lastSonarMs >= g_sonar_period) {
    g_lastSonarMs = now;

    int d = ultrasonic_readDistance();
    g_lastDist = d;

    if (d > 0 && d <= g_obstacle_cm) {
      g_triggered = true;
    }
  }
}

bool safety_isTriggered() {
  return g_triggered;
}

void safety_clearIfSafe() {
  if (!emergencyButton_isPressed() &&
      (g_lastDist <= 0 || g_lastDist > g_obstacle_cm)) {
    g_triggered = false;
  }
}
