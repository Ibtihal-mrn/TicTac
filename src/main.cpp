#include <Arduino.h>
#include "encoders.h"
#include "motors.h"
#include "control.h"
#include "emergencyButton.h"

// Dernières positions des encodeurs
static long last_left = 0;
static long last_right = 0;

// Latch d'arrêt d'urgence
static bool emergencyStopActive = false;

void setup() {
    Serial.begin(9600);
    motors_init();
    encoders_init();
    emergencyButton_init();

    // Initialiser les dernières positions
    encoders_read(&last_left, &last_right);
}

void loop() {
    long left, right;
    encoders_read(&left, &right);

    // --- Détection du bouton (une seule fois suffit) ---
    if (emergencyButton_isPressed()) {
        emergencyStopActive = true;
    }

    // --- Mode arrêt d'urgence verrouillé ---
    if (emergencyStopActive) {
        motors_applySpeeds(0, 0);

        // Quand le bouton est relâché, on sort proprement du mode arrêt
        if (!emergencyButton_isPressed()) {
            emergencyStopActive = false;

            // Resynchronisation pour éviter les pics
            encoders_read(&last_left, &last_right);
        }

        delay(40);
        return;
    }

    // --- Fonctionnement normal ---

    long dL = left - last_left;
    long dR = right - last_right;

    last_left = left;
    last_right = right;

    int speedL, speedR;
    control_computeSpeeds(dL, dR, speedL, speedR);

    motors_applySpeeds(speedL, speedR);

    long error = dL - dR;
    Serial.print("dL:");
    Serial.print(dL);
    Serial.print(" dR:");
    Serial.print(dR);
    Serial.print(" err:");
    Serial.println(error);

    delay(40);
}
