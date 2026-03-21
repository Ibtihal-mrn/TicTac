// mainTest.cpp
#include <Arduino.h>
#include <Wire.h>

// Hardware
// #include "robot.h"
// #include "bras.h"
// #include "Relais.h"
// #include "encoders.h"
#include "us.h"         // ultrasonic
// #include "StartSwitch.h"
// #include "TeamSwitch.h"
// #include "safety.h"
// #include "motors.h"
// #include "uart.h"

// Config & Debug prints
#include "utils.h"
#include "Debug.h"
#include "configTest.h"    //TODO: change back to config.h


// ------------
// extern Motors motors;




// Hardware Interrupt (Ultrasonic sensors Hub)
volatile bool emergencyStop = false;
void IRAM_ATTR stopISR() { emergencyStop = true; }



// ================== SETUP ==================
void setup()
{
    debugInit(115200,
                DBG_FSM 
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

    // Stop pin init
    pinMode(STOP_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(STOP_PIN), stopISR, RISING);

    // UART comms
    // uart_init(link);
    // link.write(CMD_DISABLE_ZONE);
    // link.write(0xFF);           // disable all
    // delay(10);
    // link.write(CMD_ENABLE_ZONE);
    // link.write(ZONE_FRONT);     // enable only front


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

    

}

