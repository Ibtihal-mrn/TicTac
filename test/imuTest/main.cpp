#include <Arduino.h>
#include <Wire.h>
#include "imu.h"

#define SDA_PIN 5
#define SCL_PIN 6

static unsigned long lastUs = 0;
static float angleDeg = 0.0f;

void setup() {
    Serial.begin(115200);
    delay(1500);

    Wire.begin(SDA_PIN, SCL_PIN, 50000);
    delay(200);

    Serial.println();
    Serial.println("=== IMU sanity test ===");

    if (!imu_init()) {
        Serial.println("MPU6050 FAIL");
        while (true) {
            delay(1000);
        }
    }

    Serial.println("MPU6050 connected");
    Serial.println("Keep the robot still for calibration...");
    delay(1000);

    if (!imu_calibrate(600, 2)) {
        Serial.println("IMU calibration failed");
        while (true) {
            delay(1000);
        }
    }

    Serial.println("Calibration OK");
    Serial.println("Now rotate the robot by about 180 degrees and watch the logs.");
    Serial.println("Use CW then CCW to check the sign and integration.");
    lastUs = micros();
}

void loop() {
    unsigned long nowUs = micros();
    float dt = (nowUs - lastUs) / 1000000.0f;
    lastUs = nowUs;

    float gyroZ = imu_readGyroZ_dps();
    angleDeg += gyroZ * dt;

    Serial.print("[IMU TEST] gyroZ_dps=");
    Serial.print(gyroZ, 3);
    Serial.print(" angleDeg=");
    Serial.print(angleDeg, 2);
    Serial.print(" dt=");
    Serial.println(dt, 4);

    delay(50);
}