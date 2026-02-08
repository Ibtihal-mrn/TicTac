#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

// ---------- MOTEURS ADATRUFS ----------
extern Adafruit_MotorShield AFMS;
extern Adafruit_DCMotor *motorL;
extern Adafruit_DCMotor *motorR;

// inverser si nécessaire
extern bool invertLeft;
extern bool invertRight;

// ---------- PARAMETRES ----------
extern int baseSpeed;  // vitesse de base
extern float Kp;       // gain correction trajectoire

extern int trimL;
extern int trimR;

void motors_init(void);
void motors_applySpeeds(int speedL, int speedR);
void motors_stop();
void motors_rotate();

#endif
