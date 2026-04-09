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

// main.cpp
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
#include "Relais.h"
#include "encoders.h"
#include "ultrasonic_function.h"
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

// OBSTACLE : Hardware Interrupt (Ultrasonic sensors Hub Obstacle detected).
volatile bool emergencyStop = false;  //extern in globals.h
void IRAM_ATTR stopISR() { emergencyStop = digitalRead(STOP_PIN); }


// TODO: remove/obfuscate this :
// const int RELAY_PIN = 41;
const bool RELAY_ACTIVE_LOW = true;
bool lastSwitchState = HIGH;   // INPUT_PULLUP
bool sequenceDone = false;     // Pour éviter répétition



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
// ================== SETUP ==================
void setup()
{
  debugInit(115200,
            DBG_FSM |
            DBG_I2C_HUB |
            // DBG_MOTORS |
            // DBG_SENSORS |
            // DEBUG_TEAM_SWITCH |
            // DBG_SERVO |
            // DBG_ENCODER |
            // DBG_MAGNET
            // DBG_COMMS |  
            DBG_ENCODER 
            // DBG_LAUNCH_TGR       
            // comment to deactivate its related prints
  );

  // ── BLE Bridge : démarrer EN PREMIER pour que la queue existe ──────────────
  bleBridge.begin(BLE_DEVICE_NAME);
  Serial.println("[DBG] BLE bridge started");

  // // I2C Init.
  Wire.begin(6, 7); // SDA, SCL
  Wire.setClock(100000);
  // pinMode(SWITCH_PIN, INPUT_PULLUP);
  // lastSwitchState = digitalRead(SWITCH_PIN);
  // relais_init(RELAY_PIN, RELAY_ACTIVE_LOW);

  // Init electro-aimant
  const bool RELAY_ACTIVE_LOW = true;  
  bool lastSwitchState = HIGH;   // INPUT_PULLUP
  bool sequenceDone = false;     // Pour éviter répétition
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  lastSwitchState = digitalRead(SWITCH_PIN);
  relais_init(RELAY_PIN, RELAY_ACTIVE_LOW);
  // relais_on();

  // I2C Setup.
  Wire.begin(SDA_PIN, SCL_PIN, 100000);
  // initUSConfig();    //TODO: change init config of hub
  delay(200);
  

  // Utils.h
  i2c_scanner();
  // printEsp32Info();


  // ======== HARDWARE INIT ==========
  pinMode(STOP_PIN, INPUT); //
  attachInterrupt(digitalPinToInterrupt(STOP_PIN), stopISR, CHANGE);

  // Init Hardware et Robot
  ESP32PWM::allocateTimer(0); // SERVO timer (doit rester ici)
  ESP32PWM::allocateTimer(1); // SERVO timer (doit rester ici)
  bras_init();                // must run FIRST
  Serial.println("[DBG] bras_init done");
  robot_init();
  Serial.println("[DBG] robot_init done");

  bleSerial.println("Setup Done.");
  Serial.println("[DBG] setup() COMPLETE");
  Serial.println("Setup Done.");

  startSwitch.begin();
  teamSwitch.begin();

  Serial.println("Setup Done."); 
  Serial.println("Waiting for start switch...");
  startSwitch.waitForStart();
  Serial.println("Starting sequence..."); 
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
  static bool runSequence = true;  

  if (!runSequence) return;

  // driveDistancePID(1000, 200);

  relais_on();

  return;

  if (teamSwitch.readTeam() == TeamSwitchTeam::A)
  {
    Serial.println("Equipe A");
    
    driveForward(900, 150);
        // rotateAnglePID(-180, 200);      // tourne à droite de 90°
    // driveBackward(200, 50);

    delay(500);

    rotate(-80, 10);
    // rotate(-100, 10);
    // rotate(-45, 150);
    delay(200);

    driveForward(800, 150);

    delay(1000);                   // arrêt du robot
    
    bras_deployer();              // déploie le bras
    delay(500);                   // arrêt du robot
    bras_retracter();             // rétracte le bras
    delay(500);  
  }

  else
  {
    Serial.println("Equipe B");

    driveForward(100, 50);
        // rotateAnglePID(-180, 200);      // tourne à droite de 90°
    // driveBackward(200, 50);
    rotate(200, 100);

    delay(4000);
    rotate(80, 10);
    delay(3000);
    rotate(-100, 10);
    delay(3000);
    rotate(120, 10);
    delay(3000);
    rotate(-140, 10);
    // rotate(-45, 150);

    // driveForward(1000, 255);

    // driveDistancePID(500, 200);   // avance
    // rotate(180, 10);      // tourne à droite de 90°
    // delay(500);                    // arrêt

    // rotate(-180, 10);
    // relais_on();                   // active le relais
    // delay(500);                   // attendre 5 secondes

    // driveDistancePID(500, 200);   // avance
    // delay(500);                   // ← ce delay ne sert pas d'arrêt, c'est trop long

    // relais_off();                  // désactive le relais
    // delay(500);

    }


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
