#include "TeamSwitch.h"

TeamSwitch::TeamSwitch(gpio_num_t pin)
    : _pin(pin) {}

void TeamSwitch::begin()
{
    pinMode(_pin, INPUT_PULLUP);
}

TeamSwitchTeam TeamSwitch::readTeam() const
{
    return (digitalRead(_pin) == LOW) ? TeamSwitchTeam::B : TeamSwitchTeam::A;
}