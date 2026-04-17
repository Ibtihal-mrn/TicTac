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
 */

#include <Arduino.h>
#include <Wire.h>

// BLE Bridge
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
StartSwitch startSwitch((gpio_num_t)LAUNCH_TRIGGER_PIN);
TeamSwitch teamSwitch((gpio_num_t)TEAM_SWITCH_PIN);

Context fsmCtx{};

// OBSTACLE flag : Hardware Interrupt
volatile bool emergencyStopUS = false;  // extern in globals.h
void IRAM_ATTR stopISR() { emergencyStopUS = digitalRead(STOP_PIN); }

// BLE flag
volatile bool bleStopRequested = false;

static const char* heartbeatTeamName() {
    return teamSwitch.readTeam() == TeamSwitchTeam::A ? "BLUE" : "YELLOW";
}

static const char* heartbeatTiretteState() {
    return startSwitch.isInserted() ? "IN" : "OUT";
}



// ========= TACHE BLE — Core 0 =========
void bleTask(void* pvParameters) {
    bleSerial.println("[BLE_TASK] Started on Core 0");

    unsigned long lastHeartbeat = 0;

    for (;;) {
        // Flush les logs en attente vers le PC via BLE notify
        bleBridge.update();

        // Heartbeat toutes les 5 secondes
        if (millis() - lastHeartbeat >= 5000) {
            char hb[128];
            snprintf(
                hb,
                sizeof(hb),
                "[BLE] Heartbeat | connected=%d | uptime=%lus | team=%s | tirette=%s",
                bleBridge.isConnected(),
                millis() / 1000,
                heartbeatTeamName(),
                heartbeatTiretteState()
            );
            bleSerial.println(hb);
            lastHeartbeat = millis();
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


// ========= TACHE FSM — Core 1 =========
void fsmTask(void* pvParameters) {
    bleSerial.println("[FSM_TASK] Started on Core 1");

    fsm_init(fsmCtx, bleBridge.getCommandQueue());

    for (;;) {
        robot_step(fsmCtx);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


// ========= SETUP =========
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
              // DBG_MAGNET |
              // DBG_COMMS |
              // DBG_IMU |
              DBG_ENCODER
              // DBG_LAUNCH_TGR
    );

    // Hardware Interrupt
    pinMode(STOP_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(STOP_PIN), stopISR, CHANGE);

    // I2C
    Wire.begin(SDA_PIN, SCL_PIN, 100000);

    // Utils
    i2c_scanner();

    // Init Hardware
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    hardware_init(fsmCtx);

    // Init relais
    relais_init(RELAY_PIN, true);

    

    
    // ==========================================

    // BLE Bridge
    bleBridge.begin(BLE_DEVICE_NAME);
    bleSerial.println("[SETUP] BLE Bridge ready");

    // Lancer les taches FreeRTOS
    xTaskCreatePinnedToCore(bleTask, "BLE_Task", BLE_TASK_STACK, NULL, BLE_TASK_PRIO, NULL, 0);
    xTaskCreatePinnedToCore(fsmTask, "FSM_Task", FSM_TASK_STACK, NULL, FSM_TASK_PRIO, NULL, 1);


  // Init Hardware et Robot
  ESP32PWM::allocateTimer(0); // SERVO timer (doit rester ici)
  ESP32PWM::allocateTimer(1); // SERVO timer (doit rester ici)
    startSwitch.begin();
    teamSwitch.begin();
  // bras_init();                // must run FIRST
  // robot_init();

    Serial.println("Setup Done.");
}


// ========= LOOP =========
void loop() {
    // FreeRTOS gère les tâches, loop() ne fait rien
    vTaskDelay(pdMS_TO_TICKS(1000));
}