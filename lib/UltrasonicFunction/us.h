#pragma once
#include <Arduino.h>
#include <Ultrasonic.h>

// ===== ZONES =====
#define ZONE_FRONT  (1 << 0)
#define ZONE_LEFT   (1 << 1)
#define ZONE_RIGHT  (1 << 2)
#define ZONE_BACK   (1 << 3)

// ===== SENSOR STRUCT =====
struct Sensor {
    Ultrasonic us;
    uint8_t zone;
    bool enabled;
    int16_t distance;
};

#define MAX_SENSORS 12

static Sensor sensors[MAX_SENSORS];
static uint8_t sensorCount = 0;

static uint8_t activeZones = 0xFF;
static uint8_t nextSensor = 0;
static unsigned long lastRead = 0;

static const uint16_t US_DELAY = 30;
static const uint16_t THRESHOLD_CM = 15;

// ===== ADD =====
inline void us_add(uint8_t zone, int trig, int echo) {
    if (sensorCount >= MAX_SENSORS) return;

    sensors[sensorCount] = {
        Ultrasonic(trig, echo, 40000),
        zone,
        true,
        -1
    };

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
        int d = (int)s.us.read(CM);
        if (d <= 0 || d > 400) d = -1;

        s.distance = d;

        if (d > 0 && d < THRESHOLD_CM) {
            danger |= s.zone;
        }
    }

    nextSensor = (nextSensor + 1) % sensorCount;
    lastRead = now;

    return danger;
}