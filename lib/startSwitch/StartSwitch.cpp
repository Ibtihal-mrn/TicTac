#include "StartSwitch.h"

namespace
{
    bool waitForStableState(gpio_num_t pin, bool insertedState, unsigned long stableMs)
    {
        unsigned long stableSince = 0;

        while (true)
        {
            const bool inserted = digitalRead(pin) == LOW;

            if (inserted == insertedState)
            {
                if (stableSince == 0)
                {
                    stableSince = millis();
                }

                if (millis() - stableSince >= stableMs)
                {
                    return true;
                }
            }
            else
            {
                stableSince = 0;
            }

            delay(5);
        }
    }
}

StartSwitch::StartSwitch(gpio_num_t pin)
    : _pin(pin) {}

void StartSwitch::begin()
{
    pinMode(_pin, INPUT_PULLUP);
    delay(5);
}

bool StartSwitch::isInserted() const
{
    // Tirette insérée = contact GND = LOW
    return digitalRead(_pin) == LOW;
}

void StartSwitch::waitForStart()
{
    // attendre insertion
    while (!isInserted())
    {
        static unsigned long lastStatePrintMs = 0;
        if (millis() - lastStatePrintMs >= 2000) {
            Serial.print("Wait for start");
            lastStatePrintMs = millis();
        }
        delay(5);
    }

    waitForStableState(_pin, true, 30);

    // attendre retrait
    while (isInserted())
    {
        delay(5);
    }

    waitForStableState(_pin, false, 30);
}