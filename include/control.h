#pragma once
#include <Arduino.h>

// Calcule les vitesses moteurs à partir des deltas d'encodeurs
void control_computeSpeeds(long dL, long dR, int &speedL, int &speedR);
