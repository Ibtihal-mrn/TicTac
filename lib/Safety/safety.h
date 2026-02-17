#pragma once
#include <Arduino.h>



bool safety_update();


void safety_init(int obstacle_cm, uint16_t sonar_period_ms);


bool safety_isTriggered();
void safety_clearIfSafe();
