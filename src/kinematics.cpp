#include "kinematics.h"

// TES mesures
float wheelDiameter_mm = 60.0;   // rayon 3cm -> diamètre 6cm = 60mm
float trackWidth_mm    = 213.0;  // entraxe 21.3cm = 213mm
long  ticksPerRev      = 2292;   // 1 tour de roue = 2292 ticks (mesuré)

float mm_per_tick() {
  return (PI * wheelDiameter_mm) / (float)ticksPerRev;
}

long ticks_for_distance_mm(float dist_mm) {
  return lround(dist_mm / mm_per_tick());
}

// rotation sur place
long ticks_for_rotation_deg(float angleDeg) {
  float arc_mm = PI * trackWidth_mm * (abs(angleDeg) / 360.0);
  return ticks_for_distance_mm(arc_mm);
}
