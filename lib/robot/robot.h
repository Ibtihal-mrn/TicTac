/**
 * fsm.h — Machine à états du robot (FreeRTOS, Core 1)
 *
 * États :
 *   INIT → IDLE → DISPATCH_CMD → { EXEC_MOVE, EXEC_ROTATE, EXEC_STOP }
 *                                       ↓            ↓          ↓
 *                                  DISPATCH_CMD ←────┘──────────┘
 *
 * La FSM consomme les commandes de la cmdQueue (remplie par le BLE sur Core 0).
 */

#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include "BLEBridge.h"
#include "../../src/config.h"
#include "../../src/globals.h"


#include "motors.h"
#include "encoders.h"
#include "ultrasonic_function.h"
#include "EmergencyButton.h"
#include "imu.h"

#include "control.h"
#include "kinematics.h"
#include "safety.h"

#include "utils.h"
#include "Debug.h"
// ── États de la FSM ──────────────────────────────────────────────────────────
enum class FsmState : uint8_t {
    INIT,
    IDLE,
    DISPATCH_CMD,
    EXEC_MOVE,
    EXEC_MOVE_FORWARD,
    EXEC_MOVE_BACKWARD,
    EXEC_ROTATE,
    EXEC_ROTATE_LEFT,
    EXEC_ROTATE_RIGHT,
    EXEC_STOP,
    EMERGENCY_STOP,
};

// ── Contexte partagé de la FSM ───────────────────────────────────────────────
struct FsmContext {
    FsmState       currentState   = FsmState::INIT;
    QueueHandle_t  cmdQueue       = nullptr;   // Queue de commandes (BLE → FSM)
    RobotCommand   currentCommand = {};        // Commande en cours d'exécution
    unsigned long  stateEntryTime = 0;         // millis() à l'entrée dans l'état
    uint32_t       cmdCount       = 0;         // Nombre de commandes traitées
};

// ── API FSM ──────────────────────────────────────────────────────────────────

/// Initialiser le contexte FSM
void fsm_init(FsmContext& ctx, QueueHandle_t cmdQueue);

/// Un pas de la FSM (appelé en boucle par la tâche Core 1)
void fsm_step(FsmContext& ctx);

/// Obtenir le nom textuel d'un état
const char* fsm_state_name(FsmState state);
// New
void driveDistancePID(float distance_mm, int speed);
void rotateAnglePID(float angle_deg, int speed);

// Avec US
void driveForward(float mm, int speed);
void driveBackward(float mm, int speed);
void rotate(float angle, int speed);

// Freinage
void brakeForwardMotion(int initialSpeed);

// ------------------- LEGACY -------------------------------------
void robot_init();

void robot_move_distance(float dist_mm, int speed);
void robot_rotate(float angle_deg, int speed);
void robot_rotate_gyro(float target_deg, int pwmMax);

void robot_step();

void newPIDTestForward(float mm, int speed);
