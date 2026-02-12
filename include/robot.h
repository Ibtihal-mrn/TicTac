#pragma once
#include <Arduino.h>

void robot_init();
void robot_step();
void robot_move_distance(float dist_mm, int speed);
void robot_rotate(float angle_deg, int speed);
void robot_stop();
void robot_rotate_gyro(float target_deg, int pwmMax);
