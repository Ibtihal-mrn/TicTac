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
#include <Wire.h>            // Bus I2C (partagé entre IMU et Sensor Hub)
#include <ESP32Servo.h>      // ESP32PWM::allocateTimer()
#include "BLEBridge.h"
#include "robot.h"
#include "config.h"          // Pins, adresses, config
#include "globals.h"         // Mutex I2C
#include "TeamSwitch.h"      // Team switch hardware
#include "StartSwitch.h"     // Tirette (launch trigger)

// ═══════════════════════════════════════════════════════════════════════════
//  DÉFINITION DES GLOBALES (déclarées "extern" dans globals.h)
// ═══════════════════════════════════════════════════════════════════════════
//  Ces variables sont créées ICI (une seule fois) et utilisées partout
//  via les "extern" de globals.h.

SemaphoreHandle_t i2cMutex        = nullptr;  // Mutex pour protéger le bus I2C

volatile bool emergencyStop = false;  // Flag d'arrêt d'urgence (utilisé par robot.cpp)

// Équipe lue depuis le team switch au boot
TeamSwitch teamSwitch((gpio_num_t)TEAM_SWITCH_PIN);
const char* currentTeam = "BLUE";  // par défaut, mis à jour dans setup()

// ── Tirette (start switch) ───────────────────────────────────────────────────
StartSwitch startSwitch((gpio_num_t)LAUNCH_TRIGGER_PIN);
volatile bool matchStarted = false;  // passe à true quand la tirette est retirée

// ── Contexte FSM (utilisé uniquement par Core 1) ────────────────────────────
static FsmContext fsmCtx;

// ── Taille des stacks FreeRTOS ───────────────────────────────────────────────
#define BLE_TASK_STACK  4096
#define FSM_TASK_STACK  8192   // 4096 trop juste (snprintf, Wire, Servo, etc.)
#define BLE_TASK_PRIO   1
#define FSM_TASK_PRIO   2    // FSM légèrement plus prioritaire

// Flag partagé : la tâche FSM le met à true à chaque step
volatile bool fsmAlive = false;

// ═════════════════════════════════════════════════════════════════════════════
//  TÂCHE BLE — Core 0
//  Gère : advertising, flush des logs vers le PC, heartbeat
// ═════════════════════════════════════════════════════════════════════════════
void bleTask(void* pvParameters) {
    bleSerial.println("[BLE_TASK] Started on Core 0");

    unsigned long lastHeartbeat = 0;

    for (;;) {
        // Flush les logs en attente vers le PC via BLE notify
        bleBridge.update();

        // ── Détection tirette : insertion → retrait = MATCH START ──────────
        if (!matchStarted && !startSwitch.isInserted()) {
            // La tirette vient d'être retirée
            matchStarted = true;
            bleSerial.println("[EVENT] MATCH_START");
            Serial.println("[BLE_TASK] Tirette retiree -> MATCH_START");
        }

        // Heartbeat toutes les 5 secondes
        if (millis() - lastHeartbeat >= 5000) {
            char hb[128];
            snprintf(hb, sizeof(hb),
                     "[BLE] Heartbeat | connected=%d | uptime=%lus | FSM=%s | team=%s | tirette=%s",
                     bleBridge.isConnected(), millis() / 1000,
                     fsmAlive ? "OK" : "DEAD", currentTeam,
                     startSwitch.isInserted() ? "IN" : "OUT");
            bleSerial.println(hb);
            fsmAlive = false;  // Reset — doit être remis à true par fsmTask
            lastHeartbeat = millis();
        }

        // Laisser du temps aux autres tâches (NimBLE tourne aussi sur Core 0)
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  TÂCHE FSM — Core 1
//  Gère : machine à états INIT → IDLE → DISPATCH_CMD → ...
// ═════════════════════════════════════════════════════════════════════════════
void fsmTask(void* pvParameters) {
    bleSerial.println("[FSM_TASK] Started on Core 1");

    // Initialiser la FSM avec la queue de commandes du BLEBridge
    fsm_init(fsmCtx, bleBridge.getCommandQueue());

    for (;;) {
        // Un pas de la FSM
        fsm_step(fsmCtx);
        fsmAlive = true;  // Heartbeat FSM

        // Petit yield pour ne pas affamer le watchdog
        // Note: fsm_step() contient déjà des vTaskDelay dans certains états
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  SETUP — s'exécute sur Core 1 (Arduino default)
// ═════════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("====================================");
    Serial.println("  ESP32 BLE_FSM — Dual Core Setup");
    Serial.println("====================================");

    // ── I2C Bus — DOIT être initialisé AVANT tout périphérique I2C ──────────
    //  Wire.begin(SDA, SCL) configure le bus I2C hardware de l'ESP32.
    //  Tous les périphériques I2C (IMU, Sensor Hub) partagent ce même bus.
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2C_FREQ);       // 100 kHz (standard mode)
    Serial.println("[SETUP] I2C bus initialized");
    // ── Créer le mutex I2C FreeRTOS ──────────────────────────────────────────
    //  Un mutex = un "verrou" : une seule tâche peut le prendre à la fois.
    //  i2cMutex : empêche 2 tâches d'accéder au bus I2C simultanément
    i2cMutex        = xSemaphoreCreateMutex();
    Serial.println("[SETUP] I2C mutex created");
    // ── Team Switch : lire l'équipe physique ─────────────────────────────
    teamSwitch.begin();
    currentTeam = (teamSwitch.readTeam() == TeamSwitchTeam::A) ? "BLUE" : "YELLOW";
    Serial.printf("[SETUP] Team switch: %s\n", currentTeam);

    // ── Start Switch (tirette) ───────────────────────────────────────────
    startSwitch.begin();
    Serial.printf("[SETUP] Start switch: %s\n",
                  startSwitch.isInserted() ? "INSERTED" : "NOT INSERTED");
    // ── Timers pour ESP32Servo (DOIT être fait avant bras_init) ──────────────
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);

    // ── Initialiser le BLE Bridge (crée les queues) ──────────────────────────
    bleBridge.begin(BLE_DEVICE_NAME);
    Serial.println("[SETUP] BLE Bridge initialized");
    bleSerial.println("[SETUP] BLE Bridge ready");

    // ── Créer la tâche BLE sur Core 0 ───────────────────────────────────────
    xTaskCreatePinnedToCore(
        bleTask,           // Fonction de la tâche
        "BLE_Task",        // Nom (debug)
        BLE_TASK_STACK,    // Taille du stack
        NULL,              // Paramètre
        BLE_TASK_PRIO,     // Priorité
        NULL,              // Handle (pas besoin)
        0                  // Core 0
    );
    Serial.println("[SETUP] BLE task created on Core 0");

    // ── Créer la tâche FSM sur Core 1 ───────────────────────────────────────
    xTaskCreatePinnedToCore(
        fsmTask,           // Fonction de la tâche
        "FSM_Task",        // Nom (debug)
        FSM_TASK_STACK,    // Taille du stack
        NULL,              // Paramètre
        FSM_TASK_PRIO,     // Priorité
        NULL,              // Handle
        1                  // Core 1
    );
    Serial.println("[SETUP] FSM task created on Core 1");

    bleSerial.println("[SETUP] All tasks launched. System ready.");
    Serial.println("[SETUP] Setup complete — entering idle loop");
}

// ═════════════════════════════════════════════════════════════════════════════
//  LOOP — quasi vide, tout tourne via FreeRTOS
// ═════════════════════════════════════════════════════════════════════════════
void loop() {
    // Arduino loop() tourne aussi sur Core 1, on la laisse idle
    vTaskDelay(pdMS_TO_TICKS(1000));
}






/*#include <Arduino.h>
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
        //   DBG_PID |
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
  pinMode(STOP_PIN, INPUT); //
  attachInterrupt(digitalPinToInterrupt(STOP_PIN), stopISR, CHANGE);

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



