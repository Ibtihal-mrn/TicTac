#pragma once
#include <Arduino.h>

class RelaySwitch {
public:
    RelaySwitch(int relayPin, int switchPin, bool relayActiveLow = true);

    void begin();
    void update();

private:
    int _relayPin;
    int _switchPin;
    bool _relayActiveLow;

    unsigned long _startTime;
    bool _timerRunning;
    bool _lastSwitchState;

    void relayOn();
    void relayOff();
};
