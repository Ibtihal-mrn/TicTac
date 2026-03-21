#include <Arduino.h>
#include <Wire.h>

#define SDA_PIN 5
#define SCL_PIN 6
#define SLAVE_ADDR 0x08


void onReceive(int len) {
    Serial.print("Received: ");
    while (Wire.available()) {
        char c = Wire.read();
        Serial.print(c);
    }
    Serial.println();
}

void onRequest() {
  Wire.write("OK");  // response to master
}

void setup() {
    Serial.begin(115200);

    Wire.begin(SLAVE_ADDR, SDA_PIN, SCL_PIN, 100000);
    // Wire.begin(SLAVE_ADDR); 
    
    Wire.onReceive(onReceive);
    Wire.onRequest(onRequest);

    Serial.println("Slave ready");
}

void loop() {}