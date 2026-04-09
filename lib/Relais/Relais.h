#ifndef RELAIS_H
#define RELAIS_H

#include <Arduino.h>

// Initialisation du relais
void relais_init(int pin, bool activeLow = true);

// Allumer le relais
void relais_on();

// Éteindre le relais
void relais_off();

// Vérifier si le relais est activé
bool relais_isOn();

#endif