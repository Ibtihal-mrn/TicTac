#include "TeamSwitch.h"
#include "../../src/config.h"
#include "Debug.h"

TeamSwitch::TeamSwitch(gpio_num_t pin)
    : _pin(pin) {}

void TeamSwitch::begin()
{
    pinMode(_pin, INPUT_PULLUP);
}

TeamSwitchTeam TeamSwitch::readTeam() const
{
    if (DEBUG_TEAM_SWITCH) {
        Serial.print("Team Switch state: ");
        Serial.println(digitalRead(_pin) == LOW ? "B" : "A");
    }
    return (digitalRead(_pin) == LOW) ? TeamSwitchTeam::B : TeamSwitchTeam::A;
}