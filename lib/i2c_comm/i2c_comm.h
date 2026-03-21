// i2c_comm.h
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "us.h"

// Note :
//      robot_master : SDA - 6 , SCL - 7 
//      sensor_hub   : SDA - 13, SCL - 14 



// ===== CONFIG =====
#define HUB_ADDR 0x08

#define CMD_GET_DATA                  0x01
#define CMD_SET_ZONES                 0x02
#define CMD_ENABLE_SENSOR             0x03
#define CMD_DISABLE_SENSOR            0x04
#define CMD_SET_OBSTACLE_THRESHOLD    0x05
#define CMD_SET_CLEAR_THRESHOLD       0x06
#define CMD_PING                      0x07
#define CMD_RESET                     0x08
#define CMD_GET_CONFIG                0x09

// ================== STRUCT ==================
struct __attribute__((packed)) SensorPacket {
    int16_t front_mm=-2;
    int16_t left_mm=-2;
    int16_t right_mm=-2;
    int16_t back_mm=-2;
    int8_t danger_flags;
    int8_t status;
    int16_t checksum; //should total 12bytes/packet..
};

// ====== shared variables ======
extern volatile bool newCommand;
extern volatile uint8_t lastCmd;
extern volatile uint8_t lastParam;
extern uint8_t activeZones;
extern SensorPacket packet;



// ===== INIT =====
void initUSConfig();

// ===== SLAVE HANDLERS =====
void onReceive(int len);
void onRequest();

// ===== MASTER API =====
uint8_t pingHub();
void enableSensor(uint8_t id);
void disableSensor(uint8_t id);
bool setZones(uint8_t mask);
bool setThresholds(uint8_t obst_thr, uint8_t clear_thr);
SensorPacket getData();

// ---end