#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// ═══════════════════════════════════════════════════════════════════════════
//  I2C Mutex — OBLIGATOIRE pour partager le bus I2C entre plusieurs tâches
// ═══════════════════════════════════════════════════════════════════════════
//  Le bus I2C (Wire) n'est PAS thread-safe. Si l'IMU (Core 1) et le
//  Sensor Hub essaient de lire/écrire en même temps → corruption.
//  Le mutex garantit qu'un seul périphérique accède au bus à la fois.
extern SemaphoreHandle_t i2cMutex;

extern volatile bool emergencyStopUS;
extern volatile bool bleStopRequested;