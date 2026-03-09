#include "relaisRobot.h"

// Broche et type de relais
static int relayPin;
static bool relayActiveLow;
static bool relayState = false; // false = OFF, true = ON

void relaisRobot_init(int pin, bool activeLow) {
    relayPin = pin;
    relayActiveLow = activeLow;
    pinMode(relayPin, OUTPUT);

    // Éteindre le relais au démarrage
    if (relayActiveLow) digitalWrite(relayPin, HIGH);
    else digitalWrite(relayPin, LOW);

    relayState = false;
    Serial.print("Relais initialisé sur la broche ");
    Serial.println(relayPin);
}

void relaisRobot_on() {
    digitalWrite(relayPin, relayActiveLow ? LOW : HIGH);
    relayState = true;
    Serial.println("Relais ON");
}

void relaisRobot_off() {
    digitalWrite(relayPin, relayActiveLow ? HIGH : LOW);
    relayState = false;
    Serial.println("Relais OFF");
}

bool relaisRobot_isOn() {
    return relayState;
}
