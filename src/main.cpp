/**
 * main.cpp — Point d'entrée du projet BLE_FSM
 *
 * Architecture dual-core ESP32 + FreeRTOS :
 *   Core 0 : tâche BLE (advertising, notifications logs, réception commandes)
 *   Core 1 : tâche FSM (machine à états du robot) et I2C (IMU, IO Expander)
 *
 * Communication inter-cœurs :
 *   cmdQueue (FreeRTOS queue) : commandes BLE → FSM
 *   logQueue (dans BLEBridge) : logs FSM → BLE notify → PC
 */

#include <Arduino.h>
#include <Wire.h>            // Bus I2C (partagé entre IMU et IO Expander)
#include "BLEBridge.h"
#include "robot.h"
#include "IOExpander.h"      // Lib IO Expander (TCA9554 / PCF8574)
#include "config.h"          // Pins, adresses, config
#include "globals.h"         // Mutex I2C, IOExpanderData

// ═══════════════════════════════════════════════════════════════════════════
//  DÉFINITION DES GLOBALES (déclarées "extern" dans globals.h)
// ═══════════════════════════════════════════════════════════════════════════
//  Ces variables sont créées ICI (une seule fois) et utilisées partout
//  via les "extern" de globals.h.

SemaphoreHandle_t i2cMutex        = nullptr;  // Mutex pour protéger le bus I2C
SemaphoreHandle_t ioExpanderMutex = nullptr;  // Mutex pour protéger ioExpanderData
IOExpanderData    ioExpanderData  = {{}, false};  // Données IO Expander (pin[8] + ready)

volatile bool emergencyStop = false;  // Flag d'arrêt d'urgence (utilisé par robot.cpp)

// ── Instance IO Expander ─────────────────────────────────────────────────
//  On crée l'objet avec l'adresse I2C définie dans config.h.
//  L'objet est global pour être accessible si besoin (ex: queueWrite depuis la FSM).
IOExpander ioExpander(IOEXP_I2C_ADDR);

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

        // Heartbeat toutes les 5 secondes
        if (millis() - lastHeartbeat >= 5000) {
            char hb[96];
            snprintf(hb, sizeof(hb), "[BLE] Heartbeat | connected=%d | uptime=%lus | FSM=%s",
                     bleBridge.isConnected(), millis() / 1000,
                     fsmAlive ? "OK" : "DEAD");
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
    //  Tous les périphériques I2C (IMU, IO Expander) partagent ce même bus.
    /*Wire.begin(I2C_SDA, I2C_SCL);  // GPIO6 = SDA, GPIO7 = SCL
    Wire.setClock(I2C_FREQ);       // 100 kHz (standard mode)
    Serial.println("[SETUP] I2C bus initialized (SDA=6, SCL=7, 100kHz)");
*/
    // ── Créer les mutex FreeRTOS ─────────────────────────────────────────────
    //  Un mutex = un "verrou" : une seule tâche peut le prendre à la fois.
    //  i2cMutex : empêche 2 tâches d'accéder au bus I2C simultanément
    //  ioExpanderMutex : protège la struct IOExpanderData en lecture/écriture
    i2cMutex        = xSemaphoreCreateMutex();
    ioExpanderMutex = xSemaphoreCreateMutex();
    Serial.println("[SETUP] I2C + IOExpander mutexes created");

    // ── Initialiser le BLE Bridge (crée les queues) ──────────────────────────
    bleBridge.begin(BLE_DEVICE_NAME);
    Serial.println("[SETUP] BLE Bridge initialized");
    bleSerial.println("[SETUP] BLE Bridge ready");

    // ── IO Expander — initialisation + configuration des pins ────────────────
    //  begin() crée la queue interne FreeRTOS pour les commandes d'écriture.
    //  queueWrite(0x03, ...) configure quelles pins sont input/output :
    //    Registre 0x03 = Configuration Register du TCA9554
    //    Chaque bit = 1 pin :  1 = input,  0 = output
    //    0x0F = 0b00001111 → P0-P3 = input, P4-P7 = output
    /*if (ioExpander.begin()) {
        ioExpander.queueWrite(0x03, IOEXP_PIN_CONFIG);  // Configurer direction des pins
        Serial.printf("[SETUP] IO Expander initialized (addr=0x%02X, config=0x%02X)\n",
                      IOEXP_I2C_ADDR, IOEXP_PIN_CONFIG);
    } else {
        Serial.println("[SETUP] ERROR: IO Expander queue creation failed!");
    }
*/
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

    // ── Créer la tâche IO Expander sur Core 1 ────────────────────────────────
    //  On la met sur Core 1 (avec la FSM) car l'IMU y tourne aussi →
    //  les accès I2C restent sur le même core, ce qui simplifie le scheduling.
    //  La tâche run() boucle indéfiniment :
    //    1. Traite les commandes d'écriture en queue
    //    2. Toutes les 200ms, lit le registre 0x00 (état des pins)
    //    3. Met à jour ioExpanderData (teamSwitch, launchTrigger)
    //  Le paramètre &ioExpander est passé à taskEntry() qui appelle run().
    /*xTaskCreatePinnedToCore(
        IOExpander::taskEntry,  // Point d'entrée statique (appelle run())
        "IOExp_Task",           // Nom pour le debug
        IOEXP_TASK_STACK,       // 2048 bytes de stack (suffisant)
        &ioExpander,            // Paramètre = pointeur vers notre instance
        IOEXP_TASK_PRIO,        // Priorité 1 (inférieure à la FSM prio 2)
        NULL,                   // Pas besoin du handle
        1                       // Core 1 (avec la FSM)
    );
    Serial.println("[SETUP] IO Expander task created on Core 1");

    bleSerial.println("[SETUP] All tasks launched. System ready.");
    Serial.println("[SETUP] Setup complete — entering idle loop");
}
*/
}

// ═════════════════════════════════════════════════════════════════════════════
//  LOOP — quasi vide, tout tourne via FreeRTOS
// ═════════════════════════════════════════════════════════════════════════════
void loop() {
    // Arduino loop() tourne aussi sur Core 1, on la laisse idle
    vTaskDelay(pdMS_TO_TICKS(1000));
}
