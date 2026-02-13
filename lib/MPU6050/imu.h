#pragma once
#include <Arduino.h>

bool imu_init();
bool imu_calibrate(uint16_t samples = 500, uint16_t delay_ms = 3);

// Renvoie la vitesse angulaire Z (yaw rate) en deg/s, bias déjà retiré
float imu_readGyroZ_dps();
