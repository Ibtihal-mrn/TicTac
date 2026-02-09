#pragma once
#include <Arduino.h>

extern float wheelDiameter_mm;   // diamètre roue
extern float trackWidth_mm;      // entraxe
extern long  ticksPerRev;        // ticks pour 1 tour de roue (ce que TON code compte)

float mm_per_tick();
long  ticks_for_distance_mm(float dist_mm);
long  ticks_for_rotation_deg(float angleDeg);