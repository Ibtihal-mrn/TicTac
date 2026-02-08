#include <Arduino.h>
#include "encoders.h"
#include "motors.h"
#include "control.h"
#include "robot.h"

void setup() {
  Serial.begin(9600);
  robot_init();
  
}

void loop() {

  robot_step();
  
}
