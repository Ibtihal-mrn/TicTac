#ifndef ULTRASONIC_FUNCTION_US_H
#define ULTRASONIC_FUNCTION_US_H

#include <Arduino.h>

void ultrasonic_update();      // nouveau: à appeler souvent (loop)

bool ultrasonic_isObstacle();
int16_t ultrasonic_read();     // min des 3 capteurs avant

void printUltrasonicVal();

#endif