#include <Arduino.h>

#include <L298NX2.h>

#define ENA 14
#define IN1 13
#define IN2 12

L298N motor(ENA, IN1, IN2);

void setup() {
  pinMode(ENA, OUTPUT);
  digitalWrite(ENA, HIGH);   // FORCE ENABLE

  motor.forward();
}

void loop() {}

