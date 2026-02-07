#include "control.h"
#include "motors.h"   // pour baseSpeed, Kp, trimL, trimR

void control_computeSpeeds(long dL, long dR, int &speedL, int &speedR) {
  // erreur basée sur la vitesse, PAS la position totale
  long error = dL - dR;

  int correction = error * Kp;

  // limite la correction pour éviter les crashs
  correction = constrain(correction, -60, 60);

  speedL = constrain(baseSpeed - correction + trimL, 0, 255);
  speedR = constrain(baseSpeed + correction + trimR, 0, 255);
}
