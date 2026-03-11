/**
 * main.cpp — Point d'entrée du projet BLE_FSM
 *
 * Architecture dual-core ESP32 + FreeRTOS :
 *   Core 0 : tâche BLE (advertising, notifications logs, réception commandes)
 *   Core 1 : tâche FSM (machine à états du robot)
 *
 * Communication inter-cœurs :
 *   cmdQueue (FreeRTOS queue) : commandes BLE → FSM
 *   logQueue (dans BLEBridge) : logs FSM → BLE notify → PC
 */

#include <Arduino.h>
#include "BLEBridge.h"
#include "robot.h"

// ── Contexte FSM (utilisé uniquement par Core 1) ────────────────────────────
static FsmContext fsmCtx;

// ── Taille des stacks FreeRTOS ───────────────────────────────────────────────
#define BLE_TASK_STACK  4096
#define FSM_TASK_STACK  4096
#define BLE_TASK_PRIO   1
#define FSM_TASK_PRIO   2    // FSM légèrement plus prioritaire

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
#include "robot.h"
#include "bras.h"

// Debug prints
#include "encoders.h"
#include "ultrasonic_function.h"
#include "utils.h"
#include "Debug.h"
#include "config.h"

#include "EmergencyButton.h"  // ← Bouton urgence
#include "safety.h"           // ← Safety update
#include "motors.h"            // ← Test moteurs
extern Motors motors;  // ← Import depuis robot.cpp



// ------ helpers ------
void imAlive()
{
  static unsigned long millis_print = 0;
  if (millis() - millis_print >= 2000)
  {
    bleSerial.println("I'm alive");
    millis_print = millis();
  }
}

// ========= SETUP ===============
void setup()
{
  // Serial.begin(115200);
  debugInit(115200, // does Serial.begin()
            DBG_FSM |
                DBG_MOTORS |
                DBG_SENSORS
            // DBG_COMMS |        // comment DBG_ to deactivate its related prints
            // DBG_ENCODER |
            // DBG_LAUNCH_TGR
  );

  // ── BLE Bridge : démarrer EN PREMIER pour que la queue existe ──────────────
  bleBridge.begin(BLE_DEVICE_NAME);
  Serial.println("[DBG] BLE bridge started");

  // // I2C Init.
  Wire.begin(6, 7); // SDA, SCL
  Wire.setClock(100000);
  delay(200);

  // Utils.h
  printEsp32Info();
  // i2c_scanner();

  // Init Hardware et Robot
  ESP32PWM::allocateTimer(0); // SERVO timer (doit rester ici)
  ESP32PWM::allocateTimer(1); // SERVO timer (doit rester ici)
  bras_init();                // must run FIRST
  Serial.println("[DBG] bras_init done");
  robot_init();
  Serial.println("[DBG] robot_init done");

  bleSerial.println("Setup Done.");
  Serial.println("[DBG] setup() COMPLETE");
}

void loop()
{
  static bool runSequence = true;  static unsigned long loopCount = 0;

  // Debug : confirmer que loop() tourne
  if (loopCount % 5000 == 0) {
    Serial.printf("[DBG] loop #%lu  connected=%d\n",
                  loopCount, bleBridge.isConnected());
  }
  loopCount++;
  // ── BLE Bridge : flush logs en attente vers le PC ──────────────────────────
  bleBridge.update();

  imAlive();
  printEncodersVal();
  printUltrasonicVal();





  if (false) return;
  if (!runSequence) { return; }

  //Servo Test
  // bras_deployer();
  // delay(2000);
  // bras_retracter();
  // delay(2000);
  // bras_deployer();
  // delay(2000);
  // bras_retracter();
  // delay(2000);

  driveDistancePID(500, 254);
  delay(1000);
  driveDistancePID(-500, 254);
  delay(1000);

  rotateAnglePID(90, 200);


  runSequence = false;
}

// void loop() {
//     printUltrasonicVal();  // Décommente
//     Serial.print("Obstacle? "); 
//     Serial.println(ultrasonic_isObstacle() ? "OUI" : "NON");
//     delay(500);
// }

// void loop() {
//     Serial.println("=== DEBUG SAFETY ===");
//     Serial.print("emergencyButton: "); Serial.println(emergencyButton_isPressed() ? "OUI" : "NON");
//     Serial.print("ultrasonic obs: "); Serial.println(ultrasonic_isObstacle() ? "OUI" : "NON");
//     Serial.print("safety_update: "); Serial.println(safety_update() ? "STOP" : "OK");
//     delay(200);
// }

// void loop() {
//     Serial.println("=== DEBUG DRIVE ===");
//     Serial.print("safety_update: "); Serial.println(safety_update() ? "STOP" : "OK");
//     if (safety_update()) Serial.println("*** WOULD STOP ***");
//     delay(200);
// }

// void loop() {
//     Serial.println("START MOTORS");
//     motors.forward(150, 150);  // ← Démarre
//     delay(3000);
    
//     Serial.println("STOP TEST");
//     motors.stopMotors();       // ← Test
//     delay(3000);
    
//     Serial.println("---");
// }
*/