#include "motors.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorL = NULL;
Adafruit_DCMotor *motorR = NULL;

// inverser si nécessaire
bool invertLeft = false;
bool invertRight = false;

// ---------- PARAMETRES ----------
int baseSpeed = 140; // vitesse de base
float Kp = 0.5;      // gain correction trajectoire

int trimL = -2.5;
int trimR = 0;

void motors_init(void) {
  Wire.begin();
  AFMS.begin();

  motorL = AFMS.getMotor(1); // moteur gauche
  motorR = AFMS.getMotor(2); // moteur droit

  // moteurs initialisés à l'arrêt
  // motorL->setSpeed(0);
  // motorL->run(FORWARD);

  // motorR->setSpeed(0);
  // motorR->run(FORWARD);
  motors_stop();
}

void motors_applySpeeds(int speedL, int speedR) {
  motorL->run(FORWARD);
  motorR->run(FORWARD);

  motorL->setSpeed(speedL);
  motorR->setSpeed(speedR);
}

void motors_stop() {
  motorL->setSpeed(0);
  motorR->setSpeed(0);
  motorL->run(RELEASE); // Release permet de couper le pont H, arrêt propre
  motorR->run(RELEASE);
}

void motors_forward(int speedL, int speedR) {
  speedL = constrain(speedL, 0, 255);
  speedR = constrain(speedR, 0, 255);

  motorL->setSpeed(speedL);
  motorR->setSpeed(speedR);

  motorL->run(FORWARD);
  motorR->run(FORWARD);
}

void motors_rotateRight(int speed) {
  speed = constrain(speed, 0, 255);
  motorL->setSpeed(speed);
  motorR->setSpeed(speed);
  motorL->run(FORWARD);
  motorR->run(BACKWARD);
}

void motors_rotateLeft(int speed) {
  speed = constrain(speed, 0, 255);
  motorL->setSpeed(speed);
  motorR->setSpeed(speed);
  motorL->run(BACKWARD);
  motorR->run(FORWARD);
}