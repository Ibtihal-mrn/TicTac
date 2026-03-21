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
    int16_t distance;
};



static Sensor sensors[MAX_SENSORS];
static uint8_t sensorCount = 0;

static uint8_t activeZones = 0xFF;
static uint8_t nextSensor = 0;
static unsigned long lastRead = 0;

static const uint16_t US_DELAY = 30;

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

// ===== UPDATE =====
inline uint8_t us_update() {
    unsigned long now = millis();
    if (now - lastRead < US_DELAY) return 0;

    if (sensorCount == 0) return 0;

    Sensor &s = sensors[nextSensor];

    uint8_t danger = 0;

    if (s.enabled) {
        int d = (int)s.us->read(CM);
        if (d <= 0 || d > 400) d = -1;

        s.distance = d;

        if (d > 0 && d < US_OBSTACLE_THRESHOLD_CM) {
            danger |= s.zone;
        }
    }

    nextSensor = (nextSensor + 1) % sensorCount;
    lastRead = now;

    return danger;
}



// ============= GETTERS =========
// returns number of sensors added
inline uint8_t us_count() {
    return sensorCount;
}

// returns the zone of sensor i
inline uint8_t us_getZone(uint8_t index) {
    if (index >= sensorCount) return 0;
    return sensors[index].zone;
}

// returns the distance (cm) of sensor i
inline int16_t us_getDistance(uint8_t index) {
    if (index >= sensorCount) return -1;
    return sensors[index].distance;
}

// returns minimum distance of all sensors in a specific zone
inline int16_t us_getDistanceForZone(uint8_t zone) {
    int16_t minDist = 9999;
    for (int i = 0; i < sensorCount; i++) {
        if (sensors[i].zone & zone && sensors[i].distance > 0) {
            if (sensors[i].distance < minDist) minDist = sensors[i].distance;
        }
    }
    return (minDist == 9999) ? -1 : minDist;
}





// ==== PRINTS =======
void debugSensors() {
    static unsigned long lastPrint = 0;

    if (millis() - lastPrint < 200) return; // 5 Hz
    lastPrint = millis();

    for (int i = 0; i < us_count(); i++) {
        Serial.print("S"); Serial.print(i);
        // Serial.print(" Z:"); Serial.print(us_getZone(i));
        Serial.print(" D:"); Serial.print(us_getDistance(i));
        Serial.print(" | ");
    }
    Serial.println("");
}





//