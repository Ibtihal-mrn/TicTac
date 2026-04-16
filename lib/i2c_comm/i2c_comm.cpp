#include "i2c_comm.h"

// ===== INTERNAL STATE (only here!) =====
volatile uint8_t pingResponse = 0xAA;

SensorPacket packet;
volatile uint8_t lastCmd = 0;
volatile uint8_t lastParam = 0;
volatile bool newCommand = false;


// ===== INIT =====
void initUSConfig(){
    setZones(ZONE_FRONT);

    enableSensor(0);
    enableSensor(1);
    enableSensor(2);

    setThresholds(20, 22);
}



// ===== SLAVE =====
void onReceive(int len) {
    // Note : SINGLE PARAM COMMANDS ONLY
    if (len < 1) return;

    lastCmd = Wire.read();

    if (len >= 2) {
        lastParam = Wire.read();
    }

    newCommand = true;
}

void onRequest() {
    if (lastCmd == CMD_PING) {
        Wire.write(pingResponse); // 1 byte response
    } else {
        SensorPacket copy = packet;
        Wire.write((uint8_t*)&copy, sizeof(copy));
    }
}




// ===== MASTER =====
uint8_t pingHub() {
    Wire.beginTransmission(HUB_ADDR);
    Wire.write(CMD_PING);
    Wire.endTransmission();

    Wire.requestFrom(HUB_ADDR, 1);

    if (Wire.available()) return Wire.read();
    return 0xFF;
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
        Serial.print("I2C error (setZones): ");
        Serial.println(err);
        return false;
    }
    return true;
}

bool setThresholds(uint8_t obst_thr, uint8_t clear_thr) {
    // Set OBSTACLE
    Wire.beginTransmission(HUB_ADDR);
    Wire.write(CMD_SET_OBSTACLE_THRESHOLD);
    Wire.write(obst_thr);
    uint8_t err = Wire.endTransmission();
    if (err != 0) {
        Serial.print("I2C error (setThresholds): ");
        Serial.println(err);
        return false;
    }

    // Set CLEAR
    Wire.beginTransmission(HUB_ADDR);
    Wire.write(CMD_SET_CLEAR_THRESHOLD);
    Wire.write(clear_thr);
    err = Wire.endTransmission();
    if (err != 0) {
        Serial.print("I2C error (setThresholds): ");
        Serial.println(err);
        return false;
    }

    // Return
    return true;
}

SensorPacket getData() {
    SensorPacket p;
    memset(&p, 0, sizeof(p));  // zero-init to avoid garbage if I2C fails

    Wire.beginTransmission(HUB_ADDR);
    Wire.write(CMD_GET_DATA);
    uint8_t err = Wire.endTransmission();

    if (err != 0) {
        Serial.print("I2C error (getData): ");
        Serial.println(err);
        return p;
    }

    int received = Wire.requestFrom(HUB_ADDR, sizeof(SensorPacket));

    if (received != sizeof(SensorPacket)) {
        Serial.println("Incomplete packet!");
        return p;
    }

    Wire.readBytes((uint8_t*)&p, sizeof(SensorPacket));
    return p;
}

// ---end