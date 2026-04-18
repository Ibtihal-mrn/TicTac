#include "Relais.h"

// Variables internes au module
static int _relayPin;
static bool _activeLow;
static bool _relayState = false;

// Configuration de la broche et extinction au demarrage
void relais_init(int pin, bool activeLow) {
    _relayPin  = pin;
    _activeLow = activeLow;

    pinMode(_relayPin, OUTPUT);

    // Eteindre le relais au demarrage
    relais_off();

    Serial.print("Relais initialise sur la broche ");
    Serial.println(_relayPin);
}

// Active le relais selon la logique active bas ou active haut
void relais_on() {
    digitalWrite(_relayPin, _activeLow ? LOW : HIGH);
    _relayState = true;
    Serial.println("Electroaimant ON");
}

// Desactive le relais selon la logique active bas ou active haut
void relais_off() {
    digitalWrite(_relayPin, _activeLow ? HIGH : LOW);
    _relayState = false;
    Serial.println("Electroaimant OFF");
}

// Retourne l'etat actuel du relais
bool relais_isOn() {
    return _relayState;
}