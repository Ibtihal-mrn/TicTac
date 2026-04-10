#include "imu.h"
#include <Wire.h>
#include "../../src/config.h"
<<<<<<< HEAD
#include "Debug.h"
=======
#include "../../src/globals.h"  // i2cMutex
>>>>>>> BLE

static const uint8_t MPU_ADDR = 0x68;

// Registres MPU6050
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_GYRO_CONFIG = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_CONFIG = 0x1A;
static const uint8_t REG_SMPLRT_DIV = 0x19;
static const uint8_t REG_GYRO_ZOUT_H = 0x47;

// Sensibilité gyro pour ±250°/s : 131 LSB/(deg/s)
static const float GYRO_SENS_250 = 131.0f;
static float gyroZ_bias_dps = 0.0f;

#ifndef IMU_GYRO_Z_SIGN
#define IMU_GYRO_Z_SIGN 1.0f
#endif



// ------- R/W ----------
static bool writeReg(uint8_t reg, uint8_t val)
{
  if (i2cMutex && xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) != pdTRUE) return false;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  bool ok = (Wire.endTransmission() == 0);
  if (i2cMutex) xSemaphoreGive(i2cMutex);
  return ok;
}

static bool readBytes(uint8_t reg, uint8_t *buf, uint8_t len)
{
  if (i2cMutex && xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(100)) != pdTRUE) return false;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    if (i2cMutex) xSemaphoreGive(i2cMutex);
    return false;
  }
  uint8_t n = Wire.requestFrom((uint16_t)MPU_ADDR, (uint8_t)len, true);
  if (n != len) {
    if (i2cMutex) xSemaphoreGive(i2cMutex);
    return false;
  }
  for (uint8_t i = 0; i < len; i++)
    buf[i] = Wire.read();
  if (i2cMutex) xSemaphoreGive(i2cMutex);
  return true;
}




// ---------- API --------
bool imu_init()
{
  // Réveil
  if (!writeReg(REG_PWR_MGMT_1, 0x00)) return false;
  delay(50);

  // Filtre passe-bas (DLPF). 3 ~ 44Hz (souvent bien pour robot)
  if (!writeReg(REG_CONFIG, 0x03)) return false;

  // Sample rate divider (optionnel). Avec DLPF actif, base 1kHz -> /9 => 100Hz
  if (!writeReg(REG_SMPLRT_DIV, 9)) return false;

  // Gyro ±250°/s
  if (!writeReg(REG_GYRO_CONFIG, 0x00)) return false;

  // Accel (pas indispensable ici). Laisse par défaut ±2g
  if (!writeReg(REG_ACCEL_CONFIG, 0x00)) return false;

  debugPrintf(DBG_IMU, "[IMU] init ok");

  return true;
}

bool imu_calibrate(uint16_t samples, uint16_t delay_ms)
{
  // Le robot doit être IMMOBILE pendant la calibration
  float sum = 0.0f;
  uint16_t ok = 0;

  for (uint16_t i = 0; i < samples; i++)
  {
    uint8_t b[2];
    if (readBytes(REG_GYRO_ZOUT_H, b, 2))
    {
      int16_t raw = (int16_t)((b[0] << 8) | b[1]);
      float dps = raw / GYRO_SENS_250;
      sum += dps;
      ok++;

      if (i < 5) {
        debugPrintf(DBG_IMU, "[IMU] cal raw=%d dps=%.3f", raw, dps);
      }
    }
    delay(delay_ms);
  }

  if (ok < samples / 2) {
    debugPrintf(DBG_IMU, "[IMU] calibration failed ok=%u/%u", ok, samples);
    return false;
  }

  gyroZ_bias_dps = sum / ok;
  debugPrintf(DBG_IMU, "[IMU] bias=%.4f dps", gyroZ_bias_dps);
  return true;
}

float imu_readGyroZ_dps()
{
  uint8_t b[2];
  if (!readBytes(REG_GYRO_ZOUT_H, b, 2)) {
    debugPrintf(DBG_IMU, "[IMU] read failed");
    return 0.0f;
  }

  int16_t raw = (int16_t)((b[0] << 8) | b[1]);
  float dps = raw / GYRO_SENS_250;
  float corrected = (dps - gyroZ_bias_dps) * IMU_GYRO_Z_SIGN;

  #if DBG_IMU
    static unsigned long lastImuPrintMs = 0;
    if (millis() - lastImuPrintMs >= 250) {
      Serial.print("[IMU] raw=");
      Serial.print(raw);
      Serial.print(" dps=");
      Serial.print(dps, 3);
      Serial.print(" bias=");
      Serial.print(gyroZ_bias_dps, 3);
      Serial.print(" out=");
      Serial.println(corrected, 3);
      lastImuPrintMs = millis();
    }
  #endif

  return corrected;
}


// ------ Debug Prints -------
void printIMUVal() {
    static unsigned long lastPrintMs = 0;
    if (millis() - lastPrintMs >= 1000) {
        float gyroZ = imu_readGyroZ_dps();
        Serial.print("IMU: gyroZ=");
        Serial.println(gyroZ, 4);
        lastPrintMs = millis();
    }
}

void printIMUAngleTest() {
    static unsigned long lastUs = micros();
    static float angleDeg = 0.0f;
    static unsigned long lastPrintMs = 0;

    unsigned long nowUs = micros();
    float dt = (nowUs - lastUs) / 1000000.0f;
    lastUs = nowUs;

    float gyroZ = imu_readGyroZ_dps();
    angleDeg += gyroZ * dt;

    if (millis() - lastPrintMs >= 100) {
        Serial.print("[IMU ANGLE] rate=");
        Serial.print(gyroZ, 3);
        Serial.print(" dps angle=");
        Serial.println(angleDeg, 2);
        lastPrintMs = millis();
    }
}


