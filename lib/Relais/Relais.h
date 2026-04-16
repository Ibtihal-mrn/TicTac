#pragma once
#include <Arduino.h>

// Initialisation du relais (broche, logique active bas)
void relais_init(int pin, bool activeLow = true);

// Allumer le relais
void relais_on();

// Eteindre le relais
void relais_off();

// Retourne true si le relais est actif
bool relais_isOn();