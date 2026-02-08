#include "robot.h"
#include "encoders.h"
#include "motors.h"
#include "control.h"
#include "kinematics.h"

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

void robot_rotate(float angle_deg, int speed){
    long targetTicks = ticks_for_rotation_deg(angle_deg);

    long startL, startR;
    encoders_read(&startL, &startR);

    // choix du sens
    if (angle_deg > 0) {
        motors_rotateRight(speed);   // droite = angle positif
    } else {
        motors_rotateLeft(speed);    // gauche = angle négatif
    }

    while (true) {
        long curL, curR;
        encoders_read(&curL, &curR);

        long dL = labs(curL - startL);
        long dR = labs(curR - startR);

        if ((dL + dR) / 2 >= labs(targetTicks)) {
        break;
        }
    }

  motors_stop();
}
