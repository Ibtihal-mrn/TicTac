#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("SETUP OK");
}

void loop() {
  Serial.println("LOOP");
  delay(1000);
}