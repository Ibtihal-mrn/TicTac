#pragma once
#include <Arduino.h>

void safety_init(int obstacle_cm, uint16_t sonar_period_ms);
void safety_update();

bool safety_isTriggered();
void safety_clearIfSafe();
