#include "robot.h"
#include "encoders.h"
#include "motors.h"
#include "control.h"

void robot_init() {
  motors_init();
  encoders_init();
}

void robot_step() {
  long left, right;
  encoders_read(&left, &right);

  long dL, dR;
  encoders_computeDelta(left, right, &dL, &dR);

  int speedL, speedR;
  control_computeSpeeds(dL, dR, speedL, speedR);

  motors_applySpeeds(speedL, speedR);
}

void robot_stop(){
    motors_stop();
}
