// coprocessor_hub.cpp
#include <Arduino.h>

// Hardware
#include "us.h"
// #include "uart.h"   
#include "i2c_comm.h"


// Config & Debug prints
#include "utils.h"
#include "Debug.h"
// #include "config.h"
#include "config_coprocessor.h"



// Debug prints Toggle
#define DEBUG 1
#if DEBUG
    #define DBG_PRINT(x)  Serial.print(x)
    #define DBG_PRINTLN(x) Serial.println(x)
#else
    #define DBG_PRINT(x)
    #define DBG_PRINTLN(x)
#endif


// SensorPacket packet; // defined in i2c_comm.h


// ===== STATE =====
static uint8_t enabled_zones = 0xFF; // all enabled
uint8_t dangerFlags = 0;


// ====== Handle Commands =====
void handleCommand() {
    if (newCommand) {
        newCommand = false;

        switch (lastCmd) {

            case CMD_ENABLE_ZONE:
                enabled_zones |= lastParam;
                us_setZones(enabled_zones);
                break;

            case CMD_DISABLE_ZONE:
                enabled_zones &= ~lastParam;
                us_setZones(enabled_zones);
                break;

            case CMD_GET_DATA:
                // nothing to do, handled in onRequest()
                break;
        }
    }
    
}

// ================== SETUP ==================
void setup() {
    Serial.begin(115200); // debug
    while(!Serial) delay(10); // wait for USB enumeration
    Serial.println("Hub starting...");

    // I2C
    Wire.begin(I2C_ADDR_SENSOR_HUB, SDA_PIN_HUB, SDA_PIN_HUB);
    Wire.onReceive(onReceive);
    Wire.onRequest(onRequest);


    pinMode(STOP_PIN, OUTPUT); // emergency stop pin (defined in config_coprocessor.h)

    // Add here all the Ultrasonic sensors
    us_add(ZONE_FRONT, US_F1_TRIG, US_F1_ECHO);
    us_add(ZONE_FRONT, US_F2_TRIG, US_F2_ECHO);
    us_add(ZONE_FRONT, US_F3_TRIG, US_F3_ECHO);

    us_setZones(enabled_zones); // apply initial config

    // uart_init(uart);         // uart comms

    Serial.println("Setup done.");
}

// ================== LOOP ==================
void loop() {
    // imAlive();

    // if (true) return;

    // 1. ALWAYS update sensors (real-time)
    uint8_t d = us_update();

    #if DEBUG
        debugSensors(); // print sensor data (selected only)
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

