#include <Arduino.h>
#include "emergencyButton.h"

#define EBTN_PIN 2
#define STABLE_READS 3  // pour debounce simple si besoin

static bool lastState = false;   // état précédent lu
static int changeCount = 0;      // compte les changements d'état
static int stableCounter = 0;    // pour un debounce léger
static bool isStable = true;     // vrai quand on peut détecter un changement

void emergencyButton_init() {
    pinMode(EBTN_PIN, INPUT_PULLUP);
    lastState = digitalRead(EBTN_PIN); // on lit l'état physique au démarrage
    stableCounter = 0;
    changeCount = 0;
    isStable = true;
}

bool emergencyButton_isPressed() {
    bool current = digitalRead(EBTN_PIN);

    // Debounce simple : attendre STABLE_READS lectures consécutives
    if (current != lastState) {
        stableCounter++;
        if (stableCounter >= STABLE_READS) {
            // on considère que l'état a changé
            lastState = current;
            stableCounter = 0;
            changeCount++;

            if (changeCount % 2 == 1) {
                Serial.println("Button pressed");
                return true; // état logique = pressed
            } else {
                Serial.println("Button released");
                return false; // état logique = released
            }
        }
    } else {
        stableCounter = 0;
    }

    // Si pas de changement confirmé, on renvoie le dernier état logique
    return (changeCount % 2 == 1);
}
