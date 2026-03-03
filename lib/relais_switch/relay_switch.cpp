#include "relay_switch.h"

RelaySwitch::RelaySwitch(int relayPin, int switchPin, bool relayActiveLow)
    : _relayPin(relayPin),
      _switchPin(switchPin),
      _relayActiveLow(relayActiveLow),
      _startTime(0),
      _timerRunning(false),
      _lastSwitchState(false)
{
}

void RelaySwitch::begin() {
    Serial.begin(9600);

    pinMode(_relayPin, OUTPUT);
    pinMode(_switchPin, INPUT_PULLUP);

    relayOff();

    _lastSwitchState = digitalRead(_switchPin);

    Serial.println("Systeme pret - Electroaimant OFF");
}

void RelaySwitch::update() {

    bool currentSwitchState = digitalRead(_switchPin);

    // Détection changement d'état
    if (currentSwitchState != _lastSwitchState) {

        Serial.println(">>> CHANGEMENT SWITCH DETECTE <<<");

        _timerRunning = true;
        _startTime = millis();

        relayOn();
        Serial.println("Electroaimant ON - Timer 10 secondes");

        delay(50); // anti-rebond simple
    }

    _lastSwitchState = currentSwitchState;

    // Gestion du timer
    if (_timerRunning && millis() - _startTime >= 10000) {

        _timerRunning = false;

        relayOff();
        Serial.println("Electroaimant OFF - Timer termine");
    }
}

void RelaySwitch::relayOn() {
    digitalWrite(_relayPin, _relayActiveLow ? LOW : HIGH);
}

void RelaySwitch::relayOff() {
    digitalWrite(_relayPin, _relayActiveLow ? HIGH : LOW);
}
