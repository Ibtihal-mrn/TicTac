// us.h
#pragma once
#include <Arduino.h>
#include <Ultrasonic.h>
#include "../../src/config_coprocessor.h"

// Max number of sensors allowed
#define MAX_SENSORS 12


// ===== ZONES =====
enum Zone {
    ZONE_FRONT  = 1 << 0,
    ZONE_LEFT   = 1 << 1,
    ZONE_RIGHT  = 1 << 2,
    ZONE_BACK   = 1 << 3
};

// ===== SENSOR STRUCT =====
struct Sensor {
    Ultrasonic *us;
    uint8_t zone;
    bool enabled;
    bool obstacle=false;
    int16_t distance=-2;
    int16_t lastDistance = -2; //Hysteresis
    unsigned long lastChange=0;
};



static Sensor sensors[MAX_SENSORS];
static uint8_t sensorCount = 0;
static uint8_t nextSensor = 0;
static uint8_t activeZones = 0xFF;

static unsigned long lastRead = 0;
static const uint16_t US_DELAY = 30;  // delay between sensor read

// ===== ADD =====
inline void us_add(uint8_t zone, int trig, int echo) {
    if (sensorCount >= MAX_SENSORS) return;

    sensors[sensorCount].us = new Ultrasonic(trig, echo, US_TIMEOUT);
    sensors[sensorCount].zone = zone;
    sensors[sensorCount].enabled = true;
    sensors[sensorCount].distance = -1;

    sensorCount++;
}

// ===== ENABLE / DISABLE =====
inline void us_setZones(uint8_t mask) {
    activeZones = mask;

    for (int i = 0; i < sensorCount; i++) {
        sensors[i].enabled = (mask & sensors[i].zone);
    }
}



// ============= GETTERS =========
inline uint8_t us_count() {
    // returns number of sensors added
    return sensorCount;
}
inline uint8_t us_getZone(uint8_t index) {
    // returns the zone of sensor i
    if (index >= sensorCount) return 0;
    return sensors[index].zone;
}
inline int16_t us_getDistance(uint8_t index) {
    // returns the distance (cm) of sensor i
    if (index >= sensorCount) return -1;
    return sensors[index].distance;
}
inline int16_t us_getDistanceForZone(uint8_t zone) {
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
    if (!s.obstacle && d > 0 && d < US_OBSTACLE_THRESHOLD_CM) {
        if (now - s.lastChange > STOP_HOLD_MS) {
            s.obstacle = true;
            s.lastChange = now;
        }
    } else if (s.obstacle && (d < 0 || d > US_OBSTACLE_CLEAR_CM)) {
        if (now - s.lastChange > STOP_HOLD_MS) {
            s.obstacle = false;
            s.lastChange = now;
        }
    }

    s.lastDistance = d;

    // digitalWrite(STOP_PIN, stop); // update STOP pin in loop()
    return s.obstacle; // for debug / packet
}




// ---- end ----