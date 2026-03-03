#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <Arduino.h>


bool ultrasonic_isObstacle();

int8_t ultrasonic_read();

// ------ Debug ------
void printUltrasonicVal();

// // Initialisation du capteur
// void ultrasonic_init(int trigPin, int echoPin);

// // Lecture de la distance en cm
// int ultrasonic_readDistance();

// // Test si obstacle sous un seuil
// bool ultrasonic_isObstacle(int distance, int threshold);

#endif
