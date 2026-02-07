#include <Arduino.h>
#include "encoders.h"
#include "motors.h"
#include "control.h"
#include "emergencyButton.h"

void setup() {
  Serial.begin(9600);
  motors_init();
  encoders_init();
  emergencyButton_init();
}

void loop() {
    // Si le bouton est pressé → tout s’arrête
    if (emergencyButton_isPressed()) {
        motors_applySpeeds(0, 0);   // STOP immédiat
        delay(40);
        return; // on quitte le loop avant tout calcul ou print
    }

    // Sinon, on continue normalement
    long left, right;
    encoders_read(&left, &right);

    long dL, dR;
    encoders_computeDelta(left, right, &dL, &dR);

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
