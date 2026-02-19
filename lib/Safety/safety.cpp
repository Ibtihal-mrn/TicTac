#include "safety.h"

#include "../../src/config.h"
#include "ultrasonic.h"
#include <Ultrasonic.h>
#include "EmergencyButton.h"



static unsigned long us_millis = 0;
static int us_read_delay_ms = 500; // délai entre lectures du sonar (throttling)



// bool safety_update() {
//   // 1. bouton (rapide)
//   if (emergencyButton_isPressed()) return true;

//   // 2. ultrason (lent → throttling)
//   if (millis() - us_millis >= us_read_delay_ms && ultrasonic_isObstacle()) {  // print distance si DBG_SENSORS dans setup()
//     us_millis = millis(); 
//     return true;
//   }
//   return false;
// }

bool safety_update() {
    if (emergencyButton_isPressed()) return true;
    
    static unsigned long lastUS = 0;
    if (millis() - lastUS >= 100) {  // ← 10Hz
        lastUS = millis();
        return ultrasonic_isObstacle();
    }
    return false;
}




// ----------------------------------


// static int g_obstacle_cm;
// static uint16_t g_sonar_period;

// static bool g_triggered = false;
// static int g_lastDist = -1;
// static unsigned long g_lastSonarMs = 0;

// void safety_init(int obstacle_cm, uint16_t sonar_period_ms) {
//   g_obstacle_cm = obstacle_cm;
//   g_sonar_period = sonar_period_ms;
//   g_triggered = false;

//   emergencyButton_init();
// }

// void safety_update() {
//   // bouton (rapide)
//   if (emergencyButton_isPressed()) {
//     g_triggered = true;
//   }

//   // ultrason (lent → throttling)
//   unsigned long now = millis();
//   if (now - g_lastSonarMs >= g_sonar_period) {
//     g_lastSonarMs = now;

//     int d = ultrasonic_readDistance();
//     g_lastDist = d;

//     if (d > 0 && d <= g_obstacle_cm) {
//       g_triggered = true;
//     }
//   }
// }



// bool safety_isTriggered() { return g_triggered; }

// void safety_clearIfSafe() {
//   if (!emergencyButton_isPressed() &&
//       (g_lastDist <= 0 || g_lastDist > g_obstacle_cm)) {
//     g_triggered = false;
//   }
// }
