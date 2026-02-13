#pragma once
#include <Arduino.h>

// Calcule les vitesses moteurs à partir des deltas d'encodeurs
void control_computeSpeeds(long dL, long dR, int &speedL, int &speedR);

struct DrivePIState {
  float iL = 0;
  float iR = 0;
  int pwmBase = 0;
};

void control_reset(DrivePIState &st);

// Calcule les PWM à appliquer en marche avant pour aller droit
// - headingErrTicks : (posL - posR) en ticks (erreur de cap cumulée)
// - dL, dR : deltas ticks sur la période dt (mesure vitesse)
// - pwmTargetBase : consigne PWM "globale" (ex 140)
// - dt : période en secondes (ex 0.01)
// Sorties: pwmL/pwmR
void control_driveStraight_PI(
  DrivePIState &st,
  long headingErrTicks,
  long dL, long dR,
  int pwmTargetBase,
  float dt,
  int &pwmL, int &pwmR
);
