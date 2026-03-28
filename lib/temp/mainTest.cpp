#include <Arduino.h>
#include <Wire.h>
#include "utils.h"

#define SDA_PIN 5
#define SCL_PIN 6
#define SLAVE_ADDR 0x08

void setup() {
    Serial.begin(115200);

    Wire.begin(SDA_PIN, SCL_PIN, 100000);
    delay(200);

    i2c_scanner();

    Serial.println("Master ready");
}

void loop() {
    // Send data
    Wire.beginTransmission(SLAVE_ADDR);
    Wire.write("PING");
    uint8_t error = Wire.endTransmission();

    Serial.print("TX status: ");
    Serial.println(error);  // 0 = success

    delay(500);

    // Request response
    Wire.requestFrom(SLAVE_ADDR, 2);

    Serial.print("RX: ");
        while (Wire.available()) {
        Serial.print((char)Wire.read());
    }
    Serial.println();

    delay(1000);
}