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


// debug prints for Hysterisis debounce resilience
static bool stopState = false;
static unsigned long lastStopChange = 0;
static bool lastStopState = false; // only for debug prints


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

void debugSensors(bool stopState) {
    static unsigned long lastPrint = 0;

    if (millis() - lastPrint < 200) return; // 5 Hz
    lastPrint = millis();

    for (int i = 0; i < us_count(); i++) {
        Serial.print("S"); Serial.print(i);
        // Serial.print(" Z:"); Serial.print(us_getZone(i));
        Serial.print(" D:"); Serial.print(us_getDistance(i));
        Serial.print(" | ");
    }
    Serial.print("STOP:");
    Serial.print(stopState ? "1" : "0");
    Serial.println("");
}

void debugStopPinState(bool currentState){
    // ONLY toggle the lastStopState to print when stopPin is on or off
    if (currentState != lastStopState) {
        lastStopState = currentState;

        Serial.print("STOP → ");
        Serial.println(currentState ? "TRIGGERED" : "CLEARED");
    }
}


// ================== SETUP ==================
void setup() {
    Serial.begin(115200); // debug
    while(!Serial) delay(10); // wait for USB enumeration
    Serial.println("Hub starting...");

    // I2C
    Wire.begin(I2C_ADDR_SENSOR_HUB, SDA_PIN_HUB, SCL_PIN_HUB);
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
    // imAlive(); // 
    
    // Update each sensor
    for (int i = 0; i < us_count(); i++) {
        bool stopState = updateSensorAndStop(i);    // Update sensor with hysteresis
        digitalWrite(STOP_PIN, stopState);          // update STOP pin accordingly
        if (DEBUG) debugStopPinState(stopState);               // print if stop pin has been toggled
    }
    if (DEBUG) debugSensors(stopState); // optionally print all sensor distances



    
    // #if DEBUG
    //     debugStopPinState(stopState); 
    //     debugSensors(stopState); // print sensor data (selected only)
    // #endif
    
    if (true) return;





    // // 1. ALWAYS update sensors (real-time)
    // uint8_t d = us_update();

    // // 2. compute stop state
    // bool stopState = hysterisisStopState();

    // // 3. Apply STOP
    // digitalWrite(STOP_PIN, stopState);

    // // 4. Debug prints
    // #if DEBUG
    //     debugStopPinState(stopState); 
    //     debugSensors(stopState); // print sensor data (selected only)
    // #endif

    // // 5. UPDATE PACKET
    // packet.danger_flags = stopState ? ZONE_FRONT : 0;
    // packet.front_mm = us_getDistanceForZone(ZONE_FRONT);
    // packet.left_mm  = us_getDistanceForZone(ZONE_LEFT);
    // packet.right_mm = us_getDistanceForZone(ZONE_RIGHT);
    // packet.back_mm  = us_getDistanceForZone(ZONE_BACK);


    // // 5. Handle I2C commands (non-blocking)
    // handleCommand();
}

