// coprocessor_hub.cpp
#include <Arduino.h>

// Hardware
#include "../lib/UltrasonicFunction/us.h" 
#include "i2c_comm.h"


// Config & Debug prints
#include "utils.h"
#include "Debug.h"
// #include "config.h"  // do not add
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


// ===== STATE =====
static uint8_t enabled_zones = 0xFF; // all enabled

uint8_t US_OBSTACLE_THRESHOLD_CM = 17;
uint8_t US_OBSTACLE_CLEAR_CM     = 20;  // initial values

// debug prints for Hysterisis debounce resilience
static bool stopState = false;
static unsigned long lastStopChange = 0;
static bool lastStopState = false; // only for debug prints




// ====== Handle Commands =====
void handleCommand() {
    if (!newCommand) return;
    newCommand = false;

    Serial.print("Received command 0x");
    DBG_PRINT(lastCmd);
    DBG_PRINT(" with param 0x");
    DBG_PRINTLN(lastParam);

    switch(lastCmd) {
        case CMD_SET_ZONES:
            activeZones = lastParam;
            us_setZones(activeZones);
            Serial.print("Enabled zones: 0x"); DBG_PRINTLN(activeZones);
            break;

        case CMD_SET_OBSTACLE_THRESHOLD:
            US_OBSTACLE_THRESHOLD_CM = lastParam;
            Serial.print("Obstacle threshold set to: "); DBG_PRINTLN(US_OBSTACLE_THRESHOLD_CM);
            break;

        case CMD_SET_CLEAR_THRESHOLD:
            US_OBSTACLE_CLEAR_CM = lastParam;
            Serial.print("Clear threshold set to: "); DBG_PRINTLN(US_OBSTACLE_CLEAR_CM);
            break;

        case CMD_PING:
            DBG_PRINTLN("PING received.");
            break;

        case CMD_RESET:
            for (int i = 0; i < us_count(); i++) sensors[i].obstacle = false;
            Serial.println("All sensors reset.");
            break;

        default:
            Serial.println("Unknown command!");
            break;
    }
}


// ====== DEBUG Prints =====
const char* zoneName(uint8_t zone) {
    switch (zone) {
        case ZONE_FRONT: return "FRONT";
        case ZONE_LEFT:  return "LEFT";
        case ZONE_RIGHT: return "RIGHT";
        case ZONE_BACK:  return "BACK";
        default:         return "UNKNOWN";
    }
}

void debugSensors(bool stopState) {
    static unsigned long lastPrint = 0;
    const unsigned long PRINT_MS = 250;

    if (millis() - lastPrint < PRINT_MS) return;
    lastPrint = millis();

    Serial.println();
    Serial.println(F("========== US HUB =========="));
    Serial.print(F("STOP: "));
    Serial.println(stopState ? F("TRIGGERED") : F("CLEAR"));

    const uint8_t zones[] = {ZONE_FRONT, ZONE_LEFT, ZONE_RIGHT, ZONE_BACK};

    for (uint8_t z = 0; z < 4; z++) {
        uint8_t zone = zones[z];
        Serial.print(F("["));
        Serial.print(zoneName(zone));
        Serial.println(F("]"));

        bool found = false;

        for (int i = 0; i < us_count(); i++) {
            if (us_getZone(i) != zone) continue;

            found = true;
            Serial.print(F("  S"));
            Serial.print(i);
            Serial.print(F("  dist="));
            Serial.print(us_getDistance(i));
            Serial.print(F(" cm  obstacle="));
            Serial.print(sensors[i].obstacle ? F("1") : F("0"));
            Serial.print(F("  enabled="));
            Serial.println(sensors[i].enabled ? F("1") : F("0"));
        }

        if (!found) {
            Serial.println(F("  <no sensor>"));
        }
    }

    Serial.println(F("============================"));
}


//TODO: remove this duplicate fn
void debugStopPinState(bool currentState){
    // ONLY toggle the lastStopState to print when stopPin is on or off
    if (currentState != lastStopState) {
        lastStopState = currentState;

        Serial.print("STOP → ");
        Serial.println(currentState ? "TRIGGERED" : "CLEARED");
    }
}


void updatePacket(bool stopState){
    packet.front_mm = us_getDistanceForZone(ZONE_FRONT);
    packet.left_mm  = us_getDistanceForZone(ZONE_LEFT);
    packet.right_mm = us_getDistanceForZone(ZONE_RIGHT);
    packet.back_mm  = us_getDistanceForZone(ZONE_BACK);

    packet.danger_flags = stopState ? 1 : 0;

    if (true)return;

    #if DEBUG
        static unsigned long Lpt = 0;
        if (millis() - Lpt >= 1000)
        {
            Serial.print("PACKET → F:");
            Serial.print(packet.front_mm);
            Serial.print(" L:");
            Serial.print(packet.left_mm);
            Serial.print(" R:");
            Serial.print(packet.right_mm);
            Serial.print(" B:");
            Serial.print(packet.back_mm);
            Serial.print(" D:");
            Serial.print(packet.danger_flags);
            Serial.print("Packet size: ");
            Serial.println(sizeof(SensorPacket));
            Lpt = millis();
        }
    #endif
}

bool debounceStopPin(bool rawStop){
    /* Debounce STOP pin :
        a.ka : based on all US detections, how fast can i write the STOP_PIN ?
        - STOP immediately
        - only CLEAR after 2000ms of no obstacle.
    */ 
    static bool stopLatched = false;
    static unsigned long clearSince = 0;
    static bool lastPrintedState = false;

    if (rawStop) {
        stopLatched = true;
        clearSince = 0;
    } else if (stopLatched) {
        if (clearSince == 0) {
            clearSince = millis();
        }

        if (millis() - clearSince >= STOP_CLEAR_HOLD_MS) {
            stopLatched = false;
            clearSince = 0;
        }
    }

    digitalWrite(STOP_PIN_HUB, stopLatched);

    if (stopLatched != lastPrintedState) {
        if (DEBUG) Serial.println(stopLatched ? "STOP → TRIGGERED" : "STOP → CLEARED");
        lastPrintedState = stopLatched;
    }

    return stopLatched;
}

void setupUS(){
    // FRONT
    us_add(ZONE_FRONT, US_F1_TRIG, US_F1_ECHO);
    us_add(ZONE_FRONT, US_F2_TRIG, US_F2_ECHO);
    us_add(ZONE_FRONT, US_F3_TRIG, US_F3_ECHO);

    // RIGHT
    us_add(ZONE_RIGHT, US_R1_TRIG, US_R1_ECHO);
    us_add(ZONE_RIGHT, US_R2_TRIG, US_R2_ECHO);

    // LEFT
    us_add(ZONE_LEFT, US_L1_TRIG, US_L1_ECHO);
    us_add(ZONE_LEFT, US_L2_TRIG, US_L2_ECHO);

    // BACK
    // us_add(ZONE_FRONT, US_F1_TRIG, US_F1_ECHO);
    // us_add(ZONE_FRONT, US_F2_TRIG, US_F2_ECHO);
    // us_add(ZONE_FRONT, US_F3_TRIG, US_F3_ECHO);

}

// ================== 
//      SETUP 
// ==================
void setup() {
    Serial.begin(115200); // debug

    // I2C
    Wire.begin(HUB_ADDR, SDA_PIN_HUB, SCL_PIN_HUB, 50000);
    Wire.onReceive(onReceive);  // master writes commands
    Wire.onRequest(onRequest);  // master read

    // Emergency pin setup
    pinMode(STOP_PIN_HUB, OUTPUT); // emergency stop pin (defined in config_coprocessor.h)

    // -----
    setupUS();



    us_setZones(enabled_zones); // apply initial config

    Serial.println("Setup done.");
}

// ================== 
//      LOOP 
// ==================
void loop() {
    // //  if (true) return;

    // imAlive();
    bool stopState = false; // tracks the current stop pin state
    
    // 1. Update each sensor
    for (int i = 0; i < us_count(); i++) {
        updateSensorAndStop(i);    // Update sensor with hysteresis
        if (sensors[i].obstacle) stopState = true;
    }

    // 2. Apply STOP
    bool stableStop = debounceStopPin(stopState);

    // 3. UPDATE PACKET
    updatePacket(stableStop);

    // 4. Debug
    #if DEBUG
        debugSensors(stopState); // optionally print all sensor distances
        // debugStopPinState(stopState);               // print if stop pin has been toggled
    #endif


    // 5. I2C commands
    handleCommand();
    
    
    delay(10);

}

