#include "robot.h"
#include "encoders.h"
#include "motors.h"
#include "control.h"
#include "kinematics.h"

void robot_init() {
  motors_init();
  encoders_init();
}

void robot_step() { // On va aussi plus l'utiliser normalement
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


void robot_move_distance(float dist_mm, int speed) {
  // on garde ton système : control_computeSpeeds utilise baseSpeed
  int oldBase = baseSpeed;
  baseSpeed = speed;

  long target = ticks_for_distance_mm(abs(dist_mm));

  long startL, startR;
  encoders_read(&startL, &startR);

  // IMPORTANT : repartir propre pour les deltas de vitesse
  prevL = startL;
  prevR = startR;

  while (true) {
    long curL, curR;
    encoders_read(&curL, &curR);

    // arrêt basé sur la distance (position totale)
    long distTicksL = labs(curL - startL);
    long distTicksR = labs(curR - startR);
    if ((distTicksL + distTicksR) / 2 >= target) break;

    // correction basée sur la vitesse instantanée (comme avant)
    long dL, dR;
    encoders_computeDelta(curL, curR, &dL, &dR);

    int speedL, speedR;
    control_computeSpeeds(dL, dR, speedL, speedR);

    motors_applySpeeds(speedL, speedR);

    delay(40);
  }

  motors_stop();
  baseSpeed = oldBase;
}