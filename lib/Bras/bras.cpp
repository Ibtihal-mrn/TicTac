#include "bras.h"
#include "../../src/config.h"
#include <ESP32Servo.h> 

// Création des deux objets servo
Servo brasGauche;
Servo brasDroit;

static bool initialized = false;

// Config
const int PIN_DROIT  = SERVO_LEFT;   // voir config.h
const int PIN_GAUCHE = SERVO_RIGHT;
const int MIN_US = 500;  // Servo min/max pulse width (in microseconds)
const int MAX_US = 2400;
int angleInitialgauche = 0;
int angleFinalgauche   = 90;
int angleInitialdroit  = 100;
int angleFinaldroit    = 20;

// int angleInitialgauche = -300;   // Servo (0-180° max) A REVOIR
// int angleFinalgauche   = 90;
// int angleInitialdroit  = 100;
// int angleFinaldroit    = 20;

void bras_init() {
  if (initialized) return;

  // Attache les servos aux pins définis dans config.h
  brasGauche.attach(PIN_GAUCHE, MIN_US, MAX_US);
  brasDroit.attach(PIN_DROIT, MIN_US, MAX_US);

  // Position de départ
  brasGauche.write(angleInitialgauche);
  brasDroit.write(angleInitialdroit);

  initialized = true;
}

void bras_deployer() {
  brasGauche.write(angleFinalgauche);
  brasDroit.write(angleFinaldroit);
}

void bras_retracter() {
  brasGauche.write(angleInitialgauche);
  brasDroit.write(angleInitialdroit);
}