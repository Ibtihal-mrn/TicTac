#pragma once
#include <Arduino.h>

void robot_init();
void robot_step();
void robot_move_forward(float dist_mm, int speed);
void robot_rotate(float angle_deg, int speed);
void robot_stop();
