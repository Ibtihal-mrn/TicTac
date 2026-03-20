// coprocessor_hub.cpp
#include <Arduino.h>
#include "uart.h"
#include "us.h"
#include "config_coprocessor.h"


#define DEBUG 1

#if DEBUG
    #define DBG_PRINT(x)  Serial.print(x)
    #define DBG_PRINTLN(x) Serial.println(x)
#else
    #define DBG_PRINT(x)
    #define DBG_PRINTLN(x)
#endif


SensorPacket packet;
HardwareSerial& uart = Serial1;  // Use Serial1 for communication


// ===== STATE =====
static uint8_t enabled_zones = 0xFF; // all enabled
uint8_t dangerFlags = 0;


// ====== Handle Commands =====
void handleCommand() {
    if (!uart.available()) return;
    
    uint8_t cmd = uart.read();

    switch (cmd) {
        case CMD_GET_DATA:
            sendPacket(packet);
            DBG_PRINTLN("CMD_GET_DATA -> packet sent");
            break;

        case CMD_ENABLE_ZONE:
            if (uart.available()) {
                uint8_t mask = uart.read();
                enabled_zones |= mask;          // ADD zones
                us_setZones(enabled_zones);
                // DBG_PRINT("Enabled zones mask: "); DBG_PRINTLN(enabled_zones, BIN);
            }
            break;

        case CMD_DISABLE_ZONE:
            if (uart.available()) {
                uint8_t mask = uart.read();
                enabled_zones &= ~mask;         // REMOVE zones
                us_setZones(enabled_zones);
                // DBG_PRINT("Disabled zones mask: "); DBG_PRINTLN(enabled_zones, BIN);
            }
            break;
    }
    
}

void imAlive()
{
    static unsigned long millis_print = 0;
    if (millis() - millis_print >= 2000)
    {
        Serial.println("I'm alive");
        millis_print = millis();
    }
}


// ================== SETUP ==================
void setup() {
    Serial.begin(UART_BAUD); // debug
    while(!Serial) delay(10); // wait for USB enumeration
    Serial.println("Setup starting...");

    pinMode(STOP_PIN, OUTPUT); // emergency stop pin (defined in config_coprocessor.h)

    // Add here all the Ultrasonic sensors
    // us_add(ZONE_FRONT, US_F1_TRIG, US_F1_ECHO);
    // us_add(ZONE_FRONT, US_F2_TRIG, US_F2_ECHO);
    // us_add(ZONE_FRONT, US_F3_TRIG, US_F3_ECHO);

    // us_setZones(enabled_zones); // apply initial config

    uart_init(uart);         // uart comms

    Serial.print("Setup done.");
}

// ================== LOOP ==================
void loop() {

    imAlive();

    // if (true) return;
    // 1. ALWAYS update sensors (real-time)
    uint8_t d = us_update();

    #if DEBUG
        for (int i = 0; i < us_count(); i++) {
            DBG_PRINTLN(String("Sensor ") + i +
                        " zone " + us_getZone(i) +
                        " distance " + us_getDistance(i) + " cm");
        }
    #endif

    // 2. accumulate danger
    dangerFlags |= d;

    // 3. STOP pin
    digitalWrite(STOP_PIN, dangerFlags != 0);

    // 4. UPDATE PACKET
    packet.danger_flags = dangerFlags;
    packet.front_mm = us_getDistanceForZone(ZONE_FRONT);
    packet.left_mm  = us_getDistanceForZone(ZONE_LEFT);
    packet.right_mm = us_getDistanceForZone(ZONE_RIGHT);
    packet.back_mm  = us_getDistanceForZone(ZONE_BACK);


    // 5. Handle UART commands (non-blocking)
    handleCommand();

    // 6. reset flags each cycle
    dangerFlags = 0;
}

