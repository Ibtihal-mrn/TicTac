#pragma once
#include <Arduino.h>

// ================== PROTOCOL ==================

#define UART_BAUD 115200
#define PACKET_START 0xAA

#define CMD_GET_DATA      0x01
#define CMD_ENABLE_ZONE   0x02
#define CMD_DISABLE_ZONE  0x03

struct CommandPacket {
    uint8_t cmd;
    uint8_t param1;
    uint8_t param2;
};

// ================== DATA ==================
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

// ================== CHECKSUM ==================

inline uint16_t computeChecksum(uint8_t* data, size_t len) {
    uint16_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum ^= data[i];
    }
    return sum;
}


// ================== UART ==================
void uart_init(HardwareSerial& serial);

// SEND
void sendPacket(const SensorPacket& packet);

// RECEIVE
bool readPacket(SensorPacket& out);

