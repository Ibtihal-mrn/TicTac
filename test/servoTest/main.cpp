#include <Arduino.h>
#include <ESP32Servo.h>

#include "bras.h"



void setup() {
  Serial.begin(115200);
  ESP32PWM::allocateTimer(1); // Alloue un timer pour chaque servos (ESP32 spécifique)
  bras_init();

  Serial.println("Setup done, servo attached and initialized.");
}

void loop() {
  // Deploy servo
  Serial.println("Deploy servo");
  bras_deployer();
  delay(2000);

  // Retract servo
  Serial.println("Retract servo");
  bras_retracter();
  delay(2000);
}

// =====================================

// // Servo object
// Servo brasGauche;

// // Servo pin
// const int PIN_GAUCHE = 4;  // PWM-capable GPIO on ESP32-S3

// // Servo min/max pulse width (in microseconds)
// const int MIN_US = 500;
// const int MAX_US = 2400;

// // Servo angles
// const int ANGLE_INIT = 0;
// const int ANGLE_FINAL = 90;

// void setup() {
//   Serial.begin(115200);

//   // Allocate timers (ESP32 needs this)
//   ESP32PWM::allocateTimer(1);

//   // Attach servo with min/max pulse width
//   brasGauche.attach(PIN_GAUCHE, MIN_US, MAX_US);

//   // Set initial position
//   brasGauche.write(ANGLE_INIT);

//   Serial.println("Setup done, servo attached and initialized.");
// }

// void loop() {
//   // Deploy servo
//   Serial.println("Deploy servo");
//   brasGauche.write(ANGLE_FINAL);
//   delay(2000);

//   // Retract servo
//   Serial.println("Retract servo");
//   brasGauche.write(ANGLE_INIT);
//   delay(2000);
// }




