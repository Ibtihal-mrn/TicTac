#include "bras.h"

// Création des deux objets servo
// Servo brasGauche;
// Servo brasDroit;

// --- CONFIGURATION ---
const int PIN_DROIT  = 12;   // bras droit
const int PIN_GAUCHE = 8;  // bras gauche

int angleInitialgauche = -300;
int angleFinalgauche   = 90;
int angleInitialdroit  = 100;
int angleFinaldroit    = 20;

void bras_init() {
  // brasGauche.attach(PIN_GAUCHE);
  // brasDroit.attach(PIN_DROIT);

  // Position de départ
  // brasGauche.write(angleInitialgauche);
  // brasDroit.write(angleInitialdroit);
}

void bras_deployer() {
  // brasGauche.write(angleFinalgauche);
  // brasDroit.write(angleFinaldroit);
}

void bras_retracter() {
  // brasGauche.write(angleInitialgauche);
  // brasDroit.write(angleInitialdroit);
}