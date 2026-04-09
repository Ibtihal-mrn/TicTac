// mainTest.cpp
#include <Arduino.h>
#include <Wire.h>

// Hardware
// #include "robot.h"
// #include "bras.h"
// #include "Relais.h"
// #include "encoders.h"
         // ultrasonic
// #include "StartSwitch.h"
// #include "TeamSwitch.h"
// #include "safety.h"
// #include "motors.h"
// #include "uart.h"

// Hub sensors Coprocessor
#include "i2c_comm.h"
#include "us.h"               //only to extract shared commands (ZONE..)

// Config & Debug prints
#include "utils.h"
#include "Debug.h"
#include "configTest.h"    //TODO: change back to config.h


// ------------
// extern Motors motors;

// Debug prints Toggle
#define DEBUG 1
#if DEBUG
    #define DBG_PRINT(x)  Serial.print(x)
    #define DBG_PRINTLN(x) Serial.println(x)
#else
    #define DBG_PRINT(x)
    #define DBG_PRINTLN(x)
#endif



// Hardware Interrupt (Ultrasonic sensors Hub)
volatile bool emergencyStop = false;
void IRAM_ATTR stopISR() { emergencyStop = true; }



// ================== SETUP ==================
void setup()
{
    debugInit(115200,
                DBG_FSM |
                DBG_I2C_HUB 
                // DBG_MOTORS |
                // DBG_SENSORS |
                // DEBUG_TEAM_SWITCH |
                // DBG_SERVO |
                // DBG_ENCODER |
                // DBG_MAGNET
                // DBG_COMMS |  
                // DBG_ENCODER |
                // DBG_LAUNCH_TGR       
                // comment to deactivate its related prints
    );
    // Serial.begin(115200);

    // I2C Setup
    Wire.begin(SDA_PIN, SCL_PIN, 100000);
    delay(200);
    i2c_scanner();

    // 
    // initUSConfig();
    

    // Stop pin init
    pinMode(STOP_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(STOP_PIN), stopISR, RISING);


    // Utils.h
    // printEsp32Info();
    // i2c_scanner();

    Serial.println("Setup Done.");
}


// ================== LOOP ==================
void loop() {

    // 1. Handle Emergency
    if (emergencyStop){ 
        Serial.print("EMERGENCY STOP TRIGGERED.");
        // stop motors

        emergencyStop = false; //reset
    }


    SensorPacket p = getData();
    #if DBG_I2C_HUB
        Serial.print(F("Danger flags: ")); Serial.println(p.danger_flags);
        Serial.print(F("Front: ")); Serial.print(p.front_mm);
        Serial.print(F(" mm, Left: ")); Serial.print(p.left_mm);
        Serial.print(F(" mm, Right: ")); Serial.print(p.right_mm);
        Serial.print(F(" mm, Back: ")); Serial.print(p.back_mm);
        Serial.print("Packet size: "); Serial.println(sizeof(SensorPacket));
    #endif

    delay(500);
}

