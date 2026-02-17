#ifndef RELAIS_ROBOT_H
#define RELAIS_ROBOT_H

#include <Arduino.h>

// Initialisation du relais
void relaisRobot_init(int pin, bool activeLow = true);

// Allumer le relais
void relaisRobot_on();

// Éteindre le relais
void relaisRobot_off();

// Vérifier si le relais est activé
bool relaisRobot_isOn();

#endif
