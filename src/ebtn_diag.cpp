#include <Arduino.h>

#include "config.h"

namespace {
constexpr unsigned long PRINT_EVERY_MS = 250;
constexpr unsigned long DEBOUNCE_MS = 20;

bool lastRawState = HIGH;
bool debouncedState = HIGH;
unsigned long lastChangeMs = 0;

const char *levelName(int value)
{
  return value == LOW ? "LOW" : "HIGH";
}
}

void setup()
{
  Serial.begin(115200);
  delay(1200);

  pinMode(EBTN_PIN, EBTN_USE_INTERNAL_PULLUP ? INPUT_PULLUP : INPUT);
  lastRawState = digitalRead(EBTN_PIN);
  debouncedState = lastRawState;
  lastChangeMs = millis();

  Serial.println();
  Serial.println("=== Diagnostic bouton urgence ===");
  Serial.printf("GPIO configure: %d\n", EBTN_PIN);
  Serial.printf("Pull-up interne: %s\n", EBTN_USE_INTERNAL_PULLUP ? "activee" : "desactivee");
  Serial.printf("Etat appui attendu: %s\n", EBTN_PRESSED_STATE == LOW ? "LOW" : "HIGH");
}

void loop()
{
  const int rawState = digitalRead(EBTN_PIN);

  if (rawState != lastRawState)
  {
    lastRawState = rawState;
    lastChangeMs = millis();
    Serial.printf("CHANGEMENT BRUT -> %s (%d)\n", levelName(rawState), rawState);
  }

  if (millis() - lastChangeMs >= DEBOUNCE_MS && debouncedState != lastRawState)
  {
    debouncedState = lastRawState;
    Serial.printf("ETAT STABLE -> %s (%d) | pressed=%s\n",
                  levelName(debouncedState),
                  debouncedState,
                  debouncedState == EBTN_PRESSED_STATE ? "true" : "false");
  }

  static unsigned long lastPrintMs = 0;
  if (millis() - lastPrintMs >= PRINT_EVERY_MS)
  {
    lastPrintMs = millis();
    Serial.printf("RAW=%s (%d) | PRESSED=%s\n",
                  levelName(rawState),
                  rawState,
                  debouncedState == EBTN_PRESSED_STATE ? "true" : "false");
  }

  delay(5);
}