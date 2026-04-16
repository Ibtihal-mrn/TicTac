// us.cpp
#include "us.h"

// =========== STATE ============
Sensor sensors[MAX_SENSORS];
uint8_t sensorCount = 0;
uint8_t nextSensor = 0;
uint8_t activeZones = 0xFF;

static unsigned long lastRead = 0;


// ============= API =============
// Add sensor
void us_add(uint8_t zone, int trig, int echo) {
    if (sensorCount >= MAX_SENSORS) return;

    sensors[sensorCount].us = new Ultrasonic(trig, echo, US_TIMEOUT);
    sensors[sensorCount].zone = zone;
    sensors[sensorCount].enabled = true;
    sensors[sensorCount].distance = -1;

    sensorCount++;
}

// Enable/disable measurements in a zone
void us_setZones(uint8_t mask) {
    activeZones = mask;

    for (int i = 0; i < sensorCount; i++) {
        sensors[i].enabled = (mask & sensors[i].zone);
    }
}


// ============= GETTERS =========
uint8_t us_count() {
    // returns number of sensors added
    return sensorCount;
}
uint8_t us_getZone(uint8_t index) {
    // returns the zone of sensor i
    if (index >= sensorCount) return 0;
    return sensors[index].zone;
}
int16_t us_getDistance(uint8_t index) {
    // returns the distance (cm) of sensor i
    if (index >= sensorCount) return -1;
    return sensors[index].distance;
}
int16_t us_getDistanceForZone(uint8_t zone) {
    // returns minimum distance of all sensors in a specific zone
    int16_t minDist = 9999;
    for (int i = 0; i < sensorCount; i++) {
        if (sensors[i].zone & zone && sensors[i].distance > 0) {
            if (sensors[i].distance < minDist) minDist = sensors[i].distance;
        }
    }
    return (minDist == 9999) ? -1 : minDist;
}


// ===== UPDATE =====
bool updateSensorAndStop(int i) {
    unsigned long now = millis();
    Sensor &s = sensors[i];

    if (!s.enabled) return false;

    // 1. Read ultrasonic distance
    int d = s.us->read(CM);  // current reading
    if (d <= 0 || d > 400) d = -1;
    s.distance = d;

    // 2. Hysteresis + debounce
    const bool validObstacle = (d > 0 && d < US_OBSTACLE_THRESHOLD_CM);
    const bool validClear = (d > 0 && d > US_OBSTACLE_CLEAR_CM);
    const bool invalid = (d < 0);

    if (!s.obstacle) {
        if (validObstacle) {
            if (s.obstacleSince == 0) { s.obstacleSince = now; }
            if (now - s.obstacleSince >= STOP_HOLD_MS) {
                s.obstacle = true;
                s.obstacleSince = 0;
                s.clearSince = 0;
            }
        } else { s.obstacleSince = 0; }
    } else {
        if (validClear) {
            if (s.clearSince == 0) { s.clearSince = now; }
            if (now - s.clearSince >= STOP_HOLD_MS) {
                s.obstacle = false;
                s.clearSince = 0;
                s.obstacleSince = 0;
            }
        } else if (invalid) { s.clearSince = 0;
        } else { s.clearSince = 0;
        }
    }

    s.lastDistance = d;

    // digitalWrite(STOP_PIN, stop); // update STOP pin in loop()
    return s.obstacle; // for debug / packet
}








// ---end