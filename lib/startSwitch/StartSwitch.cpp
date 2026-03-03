#include "StartSwitch.h"

StartSwitch::StartSwitch(gpio_num_t pin)
: _pin(pin) {}

void StartSwitch::begin() {
    pinMode(_pin, INPUT_PULLUP);
}

bool StartSwitch::isInserted() const {
    // Tirette insérée = contact GND = LOW
    return digitalRead(_pin) == LOW;
}

void StartSwitch::waitForStart() {
    Serial.println("En attente de la tirette...");

    // attendre insertion
    while (!isInserted()) {
        delay(5);
    }

    delay(20); // anti-rebond

    // attendre retrait
    while (isInserted()) {
        delay(5);
    }

    delay(20); // anti-rebond

    Serial.println("Tirette retiree -> DEMARRAGE !");
}