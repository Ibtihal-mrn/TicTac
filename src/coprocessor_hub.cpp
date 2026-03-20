#include <Arduino.h>
#include "uart.h"
#include "us.h"
#include "config_coprocessor.h"

#define US_OBSTACLE_THRESHOLD_CM 15

SensorPacket packet;
HardwareSerial& link = Serial1;  // Use Serial1 for communication


// ===== STATE =====
static uint8_t enabled_zones = 0xFF; // all enabled
uint8_t dangerFlags = 0;


// ====== Handle Commands =====
void handleCommand() {
    if (!link.available()) return;
    
    uint8_t cmd = link.read();

    switch (cmd) {
        case CMD_GET_DATA:
            sendPacket(packet);
            break;

        case CMD_ENABLE_ZONE:
            if (link.available()) {
                uint8_t mask = link.read();
                enabled_zones |= mask;          // ADD zones
                us_setZones(enabled_zones);
            }
            break;

        case CMD_DISABLE_ZONE:
            if (link.available()) {
                uint8_t mask = link.read();
                enabled_zones &= ~mask;         // REMOVE zones
                us_setZones(enabled_zones);
            }
            break;
    }
    
}



// ================== SETUP ==================
void setup() {
    pinMode(STOP_PIN, OUTPUT); // emergency stop pin (defined in config_coprocessor.h)

    Serial.begin(UART_BAUD); // debug
    uart_init(link);         // uart comms

    // Add here all the Ultrasonic sensors
    us_add(ZONE_FRONT, US_F1_TRIG, US_F1_ECHO);
    us_add(ZONE_FRONT, US_F2_TRIG, US_F2_ECHO);
    us_add(ZONE_FRONT, US_F3_TRIG, US_F2_ECHO);

    us_setZones(enabled_zones); // apply initial config

    Serial.print("Setup done.");
}

// ================== LOOP ==================
void loop() {

    // 1. ALWAYS update sensors (real-time)
    uint8_t d = us_update();

    // 2. accumulate danger
    dangerFlags |= d;

    // 3. STOP pin
    digitalWrite(STOP_PIN, dangerFlags != 0);

    // 4. UPDATE PACKET (important!)
    packet.danger_flags = dangerFlags;


    // 5. Handle UART commands (non-blocking)
    handleCommand();

    // 6. reset flags each cycle
    dangerFlags = 0;
}

