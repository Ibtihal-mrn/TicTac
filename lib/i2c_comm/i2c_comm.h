// i2c_comm.h
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "us.h"

// Note :
//      robot_master : SDA - 6 , SCL - 7 
//      sensor_hub   : SDA - 13, SCL - 14 


#define HUB_ADDR 0x08

#define CMD_GET_DATA                  0x01
#define CMD_SET_ZONES                  0x02
#define CMD_ENABLE_SENSOR             0x04
#define CMD_DISABLE_SENSOR            0x05
#define CMD_SET_OBSTACLE_THRESHOLD    0x06
#define CMD_SET_CLEAR_THRESHOLD       0x07
#define CMD_PING                      0x08
#define CMD_RESET                     0x09
#define CMD_GET_CONFIG                0x10

static volatile uint8_t pingResponse = 0XAA;



// ================== STRUCT ==================
struct CommandPacket {
    uint8_t cmd;
    uint8_t param1;
    uint8_t param2;
};
// to send sensor data
struct __attribute__((packed)) SensorPacket {
    int16_t front_mm=-2;
    int16_t left_mm=-2;
    int16_t right_mm=-2;
    int16_t back_mm=-2;
    int8_t danger_flags;
    int8_t status;
    int16_t checksum;
};

// ================== DATA ==================
static SensorPacket packet;
static volatile uint8_t lastCmd = 0;
static volatile uint8_t lastParam = 0;
static volatile bool newCommand = false;



// ===== RECEIVE COMMAND FROM MASTER =====
static void onReceive(int len) {
    if (len < 1) return;

    lastCmd = Wire.read();

    if (len >= 2) {
        lastParam = Wire.read();
    }

    newCommand = true;
}

// ===== SEND DATA TO MASTER =====
static void onRequest() {
    if (lastCmd == CMD_PING) {
        Wire.write(pingResponse);  // 1 byte response
    } else {
        SensorPacket copy = packet;
        Wire.write((uint8_t*)&copy, sizeof(copy));
    }
}




// ====== MASTER =========
uint8_t pingHub() {
    Wire.beginTransmission(HUB_ADDR);
    Wire.write(CMD_PING);
    Wire.endTransmission();

    Wire.requestFrom(HUB_ADDR, 1);

    if (Wire.available()) { return Wire.read(); }

    return 0xFF; // error
}

void enableSensor(uint8_t id) {
    Wire.beginTransmission(HUB_ADDR);
    Wire.write(CMD_ENABLE_SENSOR);
    Wire.write(id);
    Wire.endTransmission();
}

void disableSensor(uint8_t id) {
    Wire.beginTransmission(HUB_ADDR);
    Wire.write(CMD_DISABLE_SENSOR);
    Wire.write(id);
    Wire.endTransmission();
}

bool setZones(uint8_t mask) {
    Wire.beginTransmission(HUB_ADDR);
    Wire.write(CMD_SET_ZONES);
    Wire.write(mask);
    uint8_t err = Wire.endTransmission();

    if (err != 0) {
        Serial.print("I2C error (setThresholds): ");
        Serial.println(err);
        return false;
    }
    return true;
}




bool setThresholds(uint8_t obst_thr, uint8_t clear_thr) {
    Wire.beginTransmission(HUB_ADDR);
    Wire.write(CMD_SET_OBSTACLE_THRESHOLD);
    Wire.write(obst_thr);
    Wire.write(CMD_SET_CLEAR_THRESHOLD);
    Wire.write(clear_thr);
    uint8_t err = Wire.endTransmission();

    if (err != 0) {
        Serial.print("I2C error (setThresholds): ");
        Serial.println(err);
        return false;
    }
    return true;
}

SensorPacket getData() {
    SensorPacket p;
    Wire.beginTransmission(HUB_ADDR);
    Wire.write(CMD_GET_DATA);
    uint8_t err = Wire.endTransmission();

    if (err != 0) {
        Serial.print("I2C error (getData): ");
        Serial.println(err);
        return;
    }

    Wire.requestFrom(HUB_ADDR, sizeof(SensorPacket));
    Wire.readBytes((uint8_t*)&p, sizeof(SensorPacket));
    return p;
}

// =============== INTI US ================
void initUSConfig(){
    // Enable zones
    setZones(ZONE_FRONT);

    // enable sensors
    enableSensor(0);
    enableSensor(1);
    enableSensor(2);

    // change obstacle detection thresholds
    setThresholds(20, 22);
    
}


