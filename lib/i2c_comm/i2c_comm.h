// i2c_comm.h
#pragma once
#include <Arduino.h>
#include <Wire.h>



#define I2C_ADDR_SENSOR_HUB 0x10

#define CMD_GET_DATA      0x01
#define CMD_ENABLE_ZONE   0x02
#define CMD_DISABLE_ZONE  0x03


// ================== STRUCT ==================
struct CommandPacket {
    uint8_t cmd;
    uint8_t param1;
    uint8_t param2;
};
// to send sensor data
struct SensorPacket {
    uint16_t front_mm;
    uint16_t left_mm;
    uint16_t right_mm;
    uint16_t back_mm;
    uint8_t danger_flags;
    uint8_t status;
    uint16_t checksum;
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
    Wire.write((uint8_t*)&packet, sizeof(packet));
}










