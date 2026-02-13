#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>
#include <NewPing.h>

#define NB_CAPTEURS 3
#define MAX_DISTANCE 200
#define SEUIL_OBSTACLE_DEFAULT 40

// Pins 7,10,13 → safe (pas 13 si PWM conflict)
static NewPing sonars[NB_CAPTEURS] = {
    NewPing(7, 7, MAX_DISTANCE),   // Cap1
    NewPing(10, 10, MAX_DISTANCE), // Cap2
    NewPing(13, 13, MAX_DISTANCE)  // Cap3 (change 2 si accoups)
};

void ultrasonic_init();
int ultrasonic_readDistance(int sensorIndex);
int ultrasonic_readAverageDistance();
bool ultrasonic_isObstacle(int threshold);

#endif
