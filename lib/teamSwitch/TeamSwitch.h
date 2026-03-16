#pragma once
#include <Arduino.h>

enum class TeamSwitchTeam
{
    A,
    B
};

class TeamSwitch
{
public:
    explicit TeamSwitch(gpio_num_t pin);

    void begin();
    TeamSwitchTeam readTeam() const;

private:
    gpio_num_t _pin;
};