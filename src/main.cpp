/**
 * main.cpp — Point d'entrée du projet BLE_FSM
 *
 * Architecture dual-core ESP32 + FreeRTOS :
 *   Core 0 : tâche BLE (advertising, notifications logs, réception commandes)
 *   Core 1 : tâche FSM (machine à états du robot) et I2C (IMU, Sensor Hub)
 *
 * Communication inter-cœurs :
 *   cmdQueue (FreeRTOS queue) : commandes BLE → FSM
 *   logQueue (dans BLEBridge) : logs FSM → BLE notify → PC
 * 
 */

// main.cpp
#include <Arduino.h>
#include <Wire.h>

// ── BLE Bridge (réception logs sur PC via BLE) ──────────────────────────────
#include "BLEBridge.h"

// Hardware
#include "fsm.h"
#include "bras.h"
#include "Relais.h"
#include "encoders.h"
#include "StartSwitch.h"
#include "TeamSwitch.h"
#include "motors.h"

// Hub sensors Coprocessor
#include "i2c_comm.h"
#include "us.h"

// Config & Debug prints
#include "utils.h"
#include "Debug.h"
#include "config.h"
#include "globals.h"

// External Objects
extern Motors motors;
StartSwitch startSwitch(GPIO_NUM_38);
TeamSwitch teamSwitch((gpio_num_t)TEAM_SWITCH_PIN);

Context fsmCtx{};

// OBSTACLE flag : Hardware Interrupt (Ultrasonic sensors Hub Obstacle detected).
volatile bool emergencyStopUS = false;  //extern in globals.h
void IRAM_ATTR stopISR() { emergencyStopUS = digitalRead(STOP_PIN); }

// BLE flag
volatile bool bleStopRequested = false;

// TODO: remove/obfuscate this :
// const int RELAY_PIN = 41;
// const bool RELAY_ACTIVE_LOW = true;
// bool lastSwitchState = HIGH;   // INPUT_PULLUP
// bool sequenceDone = false;     // Pour éviter répétition



void bleTask(void* pvParameters) {
    bleSerial.println("[BLE_TASK] Started on Core 0");

    unsigned long lastHeartbeat = 0;

    for (;;) {
        // Flush les logs en attente vers le PC via BLE notify
        bleBridge.update();

        // Heartbeat toutes les 5 secondes
        if (millis() - lastHeartbeat >= 5000) {
            char hb[64];
            snprintf(hb, sizeof(hb), "[BLE] Heartbeat | connected=%d | uptime=%lus",
                      bleBridge.isConnected(), millis() / 1000);
            bleSerial.println(hb);
            lastHeartbeat = millis();
        }

        // Laisser du temps aux autres tâches (NimBLE tourne aussi sur Core 0)
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void fsmTask(void* pvParameters) {
    bleSerial.println("[FSM_TASK] Started on Core 1");

    // Initialiser la FSM avec la queue de commandes du BLEBridge
    fsm_init(fsmCtx, bleBridge.getCommandQueue()); //TODO: getCommandQueue?
    // hardware_init(ctx);

    for (;;) {
        // Un pas de la FSM
        robot_step(fsmCtx);

        // Petit yield pour ne pas affamer le watchdog
        // Note: fsm_step() contient déjà des vTaskDelay dans certains états
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


// ------------------
void setup() {
    debugInit(115200,
            DBG_FSM |
            DBG_I2C_HUB |
            DBG_PID |
            // DBG_MOTORS |
            // DBG_SENSORS |
            // DEBUG_TEAM_SWITCH |
            // DBG_SERVO |
            // DBG_ENCODER |
            // DBG_MAGNET
            // DBG_COMMS |  
            //   DBG_IMU |
            DBG_ENCODER 
            // DBG_LAUNCH_TGR       
            // comment to deactivate its related prints
    );
        
    // ======== HARDWARE INIT ==========
    // Hub GPIO Interrupt Pin
    pinMode(STOP_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(STOP_PIN), stopISR, CHANGE);
    #ifdef LED_BUILTIN
        pinMode(LED_BUILTIN, OUTPUT);
        digitalWrite(LED_BUILTIN, LOW);
    #endif


    // I2C Setup.
    Wire.begin(SDA_PIN, SCL_PIN, 100000);
    // initUSConfig();    //TODO: change init config of hub

    // Utils.h
    i2c_scanner();
    // printEsp32Info();


    // Init Hardware et Robot
    ESP32PWM::allocateTimer(0); // SERVO timer (doit rester ici)
    ESP32PWM::allocateTimer(1); // SERVO timer (doit rester ici)
    // bras_init();                // must run FIRST
    // robot_init();

    hardware_init(fsmCtx);

    // ── Initialiser le BLE Bridge (crée les queues) ──────────────────────────
    bleBridge.begin(BLE_DEVICE_NAME); bleSerial.println("[SETUP] BLE Bridge ready");

    xTaskCreatePinnedToCore(bleTask, "BLE_Task", BLE_TASK_STACK, NULL, BLE_TASK_PRIO, NULL, 0);
    xTaskCreatePinnedToCore(fsmTask, "FSM_Task", FSM_TASK_STACK, NULL, FSM_TASK_PRIO, NULL, 1);                 
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}



