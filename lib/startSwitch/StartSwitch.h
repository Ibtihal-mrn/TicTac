#pragma once
#include <Arduino.h>

class StartSwitch {
public:
    explicit StartSwitch(gpio_num_t pin);   // type ESP32 natif

    void begin();
    void waitForStart();
    bool isInserted() const;

private:
    gpio_num_t _pin;
};