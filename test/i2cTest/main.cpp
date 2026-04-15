#include <Arduino.h>
#include <Wire.h>

#include "imu.h"

#define SDA_PIN     5      // WhiteTrig  (4)
#define SCL_PIN     6     // Yellow


void i2c_scanner() {
  Serial.println("I2C Scanner");
  byte count = 0;
  for (byte i = 8; i < 120; i++) {
    Wire.beginTransmission(i);
    Serial.print(".");
    if (Wire.endTransmission() == 0) {
      // bleSerial.print("Found device at 0x");
      Serial.println(i, HEX);
      count++;
      delay(10);
    }
  }
  if(count == 0) Serial.println("No I2C devices found");
}




void setup() {
  Serial.begin(115200);

  Wire.begin(SDA_PIN, SCL_PIN, 50000);

  delay(2000);

  // i2c_scanner();

  // IMU
    if (!imu_init())
    {
        Serial.println("MPU6050 FAIL.");
    } else
    {
        delay(200);
        Serial.println("MPU6050 connected.");
        imu_calibrate(600, 2); // ~1.2s, robot immobile
        // Serial.println("IMU calibrated");
    }


    
  Serial.println("SETUP OK");
}

void loop() {
  Serial.println("LOOP");
  delay(1000);
}