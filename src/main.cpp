#include <Arduino.h>
#include "encoders.h"
#include "motors.h"
#include "control.h"
#include "emergencyButton.h"

// Dernières positions des encodeurs
static long last_left = 0;
static long last_right = 0;

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

    long dL = 0, dR = 0;

    // Si le bouton est pressé → stop moteurs et ne pas calculer de delta
    if (emergencyButton_isPressed()) {
        motors_applySpeeds(0, 0); // stop immédiat

        // On met à jour les dernières positions pour éviter les pics à la reprise
        last_left = left;
        last_right = right;

        // Pas de print pendant la pause
        delay(40);
        return;
    }

    // Calculer les deltas nous-mêmes depuis la dernière position
    dL = left - last_left;
    dR = right - last_right;

    // Mettre à jour les dernières positions
    last_left = left;
    last_right = right;

    // Calculer les vitesses
    int speedL, speedR;
    control_computeSpeeds(dL, dR, speedL, speedR);

    // Appliquer les vitesses
    motors_applySpeeds(speedL, speedR);

    // Affichage
    long error = dL - dR;
    Serial.print("dL:");
    Serial.print(dL);
    Serial.print(" dR:");
    Serial.print(dR);
    Serial.print(" err:");
    Serial.println(error);

    delay(40);
}
