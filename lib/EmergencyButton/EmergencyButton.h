#ifndef EMERGENCY_BUTTON_H
#define EMERGENCY_BUTTON_H

#include <Arduino.h>

// Initialise le bouton d'urgence (configure la pin)
void emergencyButton_init();

// Retourne true si le bouton est pressé (avec debounce)
bool emergencyButton_isPressed();

#endif // EMERGENCY_BUTTON_H