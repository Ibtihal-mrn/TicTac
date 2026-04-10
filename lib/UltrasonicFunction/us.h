// us.h
#pragma once
#include <Arduino.h>
#include <Ultrasonic.h>
#include "../../src/config_coprocessor.h"
#include "i2c_comm.h"  // Zone enum

// ====== CONFIG =======
#define MAX_SENSORS 10
#define US_DELAY = 30

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

// ======== EXTERN VARS ======
extern Sensor sensors[MAX_SENSORS];
extern uint8_t sensorCount;
extern uint8_t activeZones;


// ========= API =========
void us_add(uint8_t zone, int trig, int echo);
void us_setZones(uint8_t mask);

uint8_t us_count();
uint8_t us_getZone(uint8_t index);
int16_t us_getDistance(uint8_t index);
int16_t us_getDistanceForZone(uint8_t zone);

bool updateSensorAndStop(int i);



// ---- end ----