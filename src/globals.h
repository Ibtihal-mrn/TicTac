#pragma once
#include <Arduino.h>


extern volatile bool emergencyStop = false;



extern Motors motors(ENA, IN1, IN2, ENB, IN3, IN4);