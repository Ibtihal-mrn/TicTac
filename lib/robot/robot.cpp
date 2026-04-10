#include "robot.h"
#include "../../src/config.h"
#include "../../src/globals.h"

#include "motors.h"
#include "encoders.h"
#include "us.h"
#include "imu.h"

#include "control.h"
// #include "kinematics.h"

//
#include "utils.h"
#include "Debug.h"

// I2C sensors
#include "i2c_comm.h"

// Forward declarations
void driveDistancePID(float distance_mm, int speed);
void rotateAnglePID(float angle_deg, int speed);

Motors motors(ENA, IN1, IN2, ENB, IN3, IN4);
// Variables globales pour le PID de déplacement et l'initiation
void robot_init()
{
  Serial.println("[robot_init] START");
  // Encoders
  encoders_init();
  Serial.println("[robot_init] Encoders OK");
  // Ultrasonic init fait dans ultrasonic.cpp

  emergencyButton_init();
  Serial.println("[robot_init] EmergencyButton OK");
  // safety_init(40, 50);      // 40cm seuil, sonar toutes les 50ms

  // Bras (servos)
  bras_init();
  Serial.println("[robot_init] Bras OK");

  // Launch trigger (tirette)
  pinMode(LAUNCH_TRIGGER_PIN, INPUT_PULLUP);

  // IMU — pas de MPU6050 sur ce robot
  // if (!imu_init())
  // {
  //   Serial.println("[robot_init] MPU6050 FAIL — skipping calibration.");
  // }
  // else
  // {
  //   delay(200);
  //   Serial.println("[robot_init] MPU6050 connected.");
  //   imu_calibrate(600, 2);
  //   Serial.println("[robot_init] IMU calibrated.");
  // }
  Serial.println("[robot_init] DONE");
}





// ======= PID MOVEMENT =========
// void driveForward(float mm, int speed){
//   setZones(ZONE_FRONT);
//   driveDistancePID(mm, speed);
// }

// void driveBackward(float mm, int speed){
//   setZones(ZONE_BACK);
//   driveDistancePID(-mm, speed);
// }

// void rotate(float angle, int speed){
//   // if (angle > 0) setZones(ZONE_RIGHT);
//   // else setZones(ZONE_LEFT);
//   setZones(ZONE_FRONT | ZONE_LEFT | ZONE_RIGHT | ZONE_BACK);
//   rotateAnglePID(angle, speed);
// }

// FREINAGE
void brakeForwardMotion(int initialSpeed)
{
    const uint16_t BRAKE_DT_MS = 10;
    const uint32_t BRAKE_TIMEOUT_MS = 300;
    const int MIN_BRAKE_PWM = 40;

    unsigned long start = millis();
    unsigned long tPrev = micros();

    while (millis() - start < BRAKE_TIMEOUT_MS)
    {
        if (digitalRead(STOP_PIN) == LOW) break;

        long curL, curR;
        encoders_read(&curL, &curR);

        // Estimate speed from encoder change here if you want
        // or just use a small fixed reverse PWM
        int brakePwm = MIN_BRAKE_PWM;

        motors.applyMotorOutputs(-brakePwm, -brakePwm);

        // Debug Prints
      unsigned long lp6 = 0;
      printMillis(DBG_MOTORS, "Emergency Button !\n", millis(), lp6, 1000);
      #if DBG_MOTORS
        static unsigned long LastPrint = 0;
        if (millis() - LastPrint >= 1000){
          SensorPacket p = getData();
          Serial.print(F("Danger flags: ")); Serial.println(p.danger_flags);
          Serial.print(F("Front: ")); Serial.print(p.front_mm);
          Serial.print(F(" mm, Left: ")); Serial.print(p.left_mm);
          Serial.print(F(" mm, Right: ")); Serial.print(p.right_mm);
          Serial.print(F(" mm, Back: ")); Serial.print(p.back_mm);
          LastPrint = millis();
        }
      #endif


        delay(BRAKE_DT_MS);
    }

    motors.stopMotors();
}




// AVANT et ARRIERE (+ , -)
void driveDistancePID(float distance_mm, int speed)
{
  // --- Determine direction ---
  bool forwardMotion = (distance_mm >= 0);
  long targetTicks = ticks_for_distance_mm(fabs(distance_mm)); // Compute Tick Target
  unsigned long lp1 = 0;
  printMillis(DBG_MOTORS, "Target computed\n", millis(), lp1, 1000);

  // --- Encoder Read Start Values ---
  long startL, startR;
  encoders_read(&startL, &startR);
  prevL = startL;
  prevR = startR;
  unsigned long lp2 = 0;
  printMillis(DBG_MOTORS, "Encoders computed\n", millis(), lp2, 1000);

  // --- riveDistancePID(-1000, 254);PID Setup ---
  DrivePIState st;
  control_reset(st);

  // Setup Timing
  const uint16_t DT_MS = 10;
  const float dt = DT_MS / 1000.0f;
  unsigned long tPrev = micros();

  while (true)
  {
    unsigned long now = micros();
    if ((unsigned long)(now - tPrev) < (unsigned long)DT_MS * 1000UL){ yield(); continue; }
    tPrev += (unsigned long)DT_MS * 1000UL;

    // COMMAND STOP BLE — check without blocking
    {
      RobotCommand peekCmd;
      if (xQueuePeek(bleBridge.getCommandQueue(), &peekCmd, 0) == pdTRUE
          && peekCmd.type == RobotCommandType::STOP) {
        // Consume the command so the FSM doesn't re-process it
        xQueueReceive(bleBridge.getCommandQueue(), &peekCmd, 0);
        bleSerial.println("[DRIVE] BLE STOP received — aborting move");
        break;
      }
    }

    // Ultrasons (via I2C coprocesseur)
    if (safety_update()) {
      motors.stopMotors();
      bleSerial.println("[DRIVE] Safety triggered — waiting...");
      while (safety_update()) { vTaskDelay(pdMS_TO_TICKS(50)); }
      bleSerial.println("[DRIVE] Safety cleared — resuming");
      control_reset(st);
      tPrev = micros();
      continue;
    }

    // --- Read Current encoders Values ---
    long curL, curR;
    encoders_read(&curL, &curR);
    if (DBG_ENCODER) printEncodersVal();

    // --- Compute distance traveled ---
    long distTicksL = labs(curL - startL);
    long distTicksR = labs(curR - startR);
    long avgDist = (distTicksL + distTicksR) / 2;
    if (avgDist >= targetTicks)
      break; // Target Reached

    // --- Compute deltas for PID ---
    long dL, dR;
    encoders_computeDelta(curL, curR, &dL, &dR);

    // Heading error (cumulative) - erreur de cap cumulée (position)
    long headingErr = (curL - startL) - (curR - startR);
    int pwmL = 0, pwmR = 0;
    control_driveStraight_PI(st, headingErr, dL, dR, speed, dt, pwmL, pwmR);

    // Automatically handle direction
    if (!forwardMotion)
    {
      pwmL = -pwmL;
      pwmR = -pwmR;
    } // invers PWM pour marche arriere

    // Moteurs PWM vitesse
    motors.applyMotorOutputs(pwmL, pwmR);

    // Debug print every 1s
    #if DBG_MOTORS
      static unsigned long Lpwm = 0;
      if (millis() - Lpwm >= 1000) {
        bleSerial.print("PWM L: ");
        bleSerial.print(pwmL);
        bleSerial.print(" | PWM R: ");
        bleSerial.println(pwmR);
        Lpwm = millis();
      }
    #endif
  }

  // Movement complete - stop motors
  motors.stopMotors();
  debugPrintf(DBG_MOTORS, "Target distance reached\n");
}

void rotateAnglePID(float angle_deg, int speed)
{
  // ╔═══════════════════════════════════════════════════════════════════════╗
  // ║  TEMPORAIRE — PID rotation basé sur ENCODEURS (pas d'IMU connectée) ║
  // ║  À remplacer par une version gyro quand le MPU6050 sera câblé.      ║
  // ╚═══════════════════════════════════════════════════════════════════════╝

  // ----- Setup Timing -----
  const uint16_t DT_MS = 10;
  unsigned long tPrev = micros();
  unsigned long startMs = millis();

  // ----- Encoder baseline -----
  long startL, startR;
  encoders_read(&startL, &startR);

  // ----- Target (degrés) -----
  const float targetDeg = angle_deg;  // positif = droite, négatif = gauche

  // ----- Variables -----
  int pwmLimit = 0;
  unsigned long stableStart = 0;
  float prevError = 0.0f;

  // ----- Paramètres PID (TEMPORAIRE — à tuner) -----
  const float KP = 3.0f;
  const float KI = 0.05f;
  const float KD = 0.8f;
  float integral = 0.0f;
  const float INTEGRAL_MAX = 5000.0f;

  const int PWM_MIN = 60;
  const int RAMP_STEP = 8;
  const float BRAKE_START_DEG = 30.0f;

  const float ANGLE_TOL = 2.0f;
  const uint16_t STABLE_MS = 150;
  const uint32_t MAX_ROTATE_MS = (uint32_t)(3000.0f + 25.0f * fabs(targetDeg));

  // ----- Control Loop -----
  while (true)
  {
    unsigned long now = micros();
    if ((unsigned long)(now - tPrev) < (unsigned long)DT_MS * 1000UL){ yield(); continue; }
    float dt = (float)(now - tPrev) / 1000000.0f;
    tPrev = now;
    dt = constrain(dt, 0.005f, 0.05f);

    // ── BLE STOP check ──
    {
      RobotCommand peekCmd;
      if (xQueuePeek(bleBridge.getCommandQueue(), &peekCmd, 0) == pdTRUE
          && peekCmd.type == RobotCommandType::STOP) {
        xQueueReceive(bleBridge.getCommandQueue(), &peekCmd, 0);
        bleSerial.println("[ROTATE] BLE STOP received — aborting rotation");
        break;
      }
    }

    // ── Safety (ultrasons) ──
    if (safety_update()) {
      motors.stopMotors();
      bleSerial.println("[ROTATE] Safety triggered — waiting...");
      while (safety_update()) { vTaskDelay(pdMS_TO_TICKS(50)); }
      bleSerial.println("[ROTATE] Safety cleared — resuming");
      tPrev = micros();
      continue;
    }

    // ── Timeout ──
    if (millis() - startMs >= MAX_ROTATE_MS)
    {
      bleSerial.println("[ROTATE] Timeout!");
      break;
    }

    // ── Angle estimé via encodeurs ──
    long curL, curR;
    encoders_read(&curL, &curR);
    float distL_mm = (float)(curL - startL) * mm_per_tick();
    float distR_mm = (float)(curR - startR) * mm_per_tick();
    float angleDeg = (distL_mm - distR_mm) / trackWidth_mm * (180.0f / PI);

    // ── PID ──
    float error = targetDeg - angleDeg;

    integral += error * dt;
    integral = constrain(integral, -INTEGRAL_MAX, INTEGRAL_MAX);

    float derivative = (error - prevError) / dt;
    prevError = error;

    // Ramp
    if (pwmLimit < speed)
      pwmLimit = min(pwmLimit + RAMP_STEP, speed);

    float control = KP * error + KI * integral + KD * derivative;

    // Freinage progressif
    int pwmCap = pwmLimit;
    float absErr = fabs(error);
    if (absErr < BRAKE_START_DEG)
    {
      float ratio = absErr / BRAKE_START_DEG;
      int brakeCap = PWM_MIN + (int)((speed - PWM_MIN) * ratio);
      pwmCap = min(pwmCap, brakeCap);
    }

    int pwm = (int)fabs(control);
    pwm = constrain(pwm, 0, pwmCap);
    if (pwm > 0)
      pwm = max(pwm, PWM_MIN);

    // ── Apply Direction ──
    if (control > 0)
      motors.applyMotorOutputs(pwm, -pwm);  // rotate right
    else
      motors.applyMotorOutputs(-pwm, pwm);  // rotate left

    // ── Debug (1x/s) ──
    #if DBG_MOTORS
      static unsigned long Lpwm = 0;
      if (millis() - Lpwm >= 500){
        Serial.printf("Angle: %.1f | Err: %.1f | I: %.1f | PWM: %d\n",
                       angleDeg, error, integral, pwm);
        Lpwm = millis();
      }
    #endif

    // ── Stop Condition ──
    if (fabs(error) < ANGLE_TOL)
    {
      if (stableStart == 0)
        stableStart = millis();

      if (millis() - stableStart >= STABLE_MS)
        break;
    }
    else
    {
      stableStart = 0;
    }
  }
  motors.stopMotors();
}


void robot_move_distance(float dist_mm, int pwmBaseTarget) {
  // // on garde ton système : control_computeSpeeds utilise baseSpeed
  // int oldBase = baseSpeed;
  // baseSpeed = speed;

  // long target = ticks_for_distance_mm(abs(dist_mm));

  // long startL, startR;
  // encoders_read(&startL, &startR);

  // // IMPORTANT : repartir propre pour les deltas de vitesse
  // prevL = startL;
  // prevR = startR;

  // while (true) {
  //   long curL, curR;
  //   encoders_read(&curL, &curR);

  //   // arrêt basé sur la distance (position totale)
  //   long distTicksL = labs(curL - startL);
  //   long distTicksR = labs(curR - startR);
  //   if ((distTicksL + distTicksR) / 2 >= target) break;

  //   // correction basée sur la vitesse instantanée (comme avant)
  //   long dL, dR;
  //   encoders_computeDelta(curL, curR, &dL, &dR);

  //   int speedL, speedR;
  //   control_computeSpeeds(dL, dR, speedL, speedR);

  //   motors_applySpeeds(speedL, speedR);

  //   delay(40);
  // }

  // motors_stop();
  // baseSpeed = oldBase;


  // Compute Tick Target
  long target = ticks_for_distance_mm(fabs(dist_mm));
  unsigned long lp1 = 0;
  // printMillis(DBG_MOTORS, "Target computed\n", millis(), lp1, 1000);

  // Encoder Read
  long startL, startR;
  // encoders_read(&startL, &startR);
  unsigned long lp2 = 0;
  // printMillis(DBG_MOTORS, "Encoders computed\n", millis(), lp2, 1000);

  // reset deltas encodeurs pour la vitesse
  // prevL = startL;
  // prevR = startR;

  // Reset Controller State
  DrivePIState st;
  // control_reset(st);

  // Setup Timing
  const uint16_t DT_MS = 10;
  const float dt = DT_MS / 1000.0f;
  unsigned long tPrev = micros();

  // MAIN LOOP
  // while (true) {
  //   // Fixed 10ms while Loop
  //   unsigned long now = micros();
  //   if ((unsigned long)(now - tPrev) < (unsigned long)DT_MS * 1000UL) {yield(); continue;}
  //   tPrev += (unsigned long)DT_MS * 1000UL;

  //   // STOP MOTOR CONDITIONS
  //   safety_update();
  //   if (safety_isTriggered()) {
  //     motors.stopMotors();
  //     // Blocking Loop
  //     while(safety_isTriggered()){
  //       static unsigned long lp3 = 0;
  //       printMillis(DBG_MOTORS, "Safety triggered\n", millis(), lp3, 2000);
  //       safety_update();
  //       safety_clearIfSafe();
  //       delay(20);
  //     }
  //   }


  //   // Encoders Update
  //   long curL, curR;
  //   encoders_read(&curL, &curR);

  //   long distTicksL = labs(curL - startL);
  //   long distTicksR = labs(curR - startR);
  //   if ((distTicksL + distTicksR) / 2 >= target) break; // Target Reached

  //   // PI CONTROL
  //   long dL, dR;
  //   encoders_computeDelta(curL, curR, &dL, &dR); // deltas (vitesse)

  //   // erreur de cap cumulée (position)
  //   long headingErr = (curL - startL) - (curR - startR);
  //   int pwmL, pwmR;
  //   control_driveStraight_PI(st, headingErr, dL, dR, pwmBaseTarget, dt, pwmL, pwmR);

  //   // motors_applySpeeds(pwmL, pwmR);

  //   static unsigned long lp5 = 0;
  //   if (millis() - lp5 >= 1000) {
  //       bleSerial.print("PWM L: ");
  //       bleSerial.print(pwmL);
  //       bleSerial.print(" | PWM R: ");
  //       bleSerial.println(pwmR);
  //       lp5 = millis();
  //   }

  //   motors.forward(pwmL, pwmR);
  //   // motors.forward(255, 255);
    
  // }

  // Default (target reached when outside while loop).
  // motors.stopMotors();
}




// ===========
// LEGACY — commented out (Context/Robot types removed)
/*
void hardware_init(Context& ctx) { ... }
void startMatchTimer(Context& ctx) { ... }
void checkMatchTimer(Context& ctx) { ... }
*/
// ── Launch Trigger (tirette) ─────────────────────────────────────────────────
static bool launchTrigger_isPulled() {
    // INPUT_PULLUP: cord in = GND = LOW ; cord pulled = HIGH
    return digitalRead(LAUNCH_TRIGGER_PIN) == HIGH;
}

// ── Helpers ──────────────────────────────────────────────────────────────────

const char* fsm_state_name(FsmState state) {
    switch (state) {
        case FsmState::INIT:              return "INIT";
        case FsmState::IDLE:              return "IDLE";
        case FsmState::DISPATCH_CMD:      return "DISPATCH_CMD";
        case FsmState::EXEC_MOVE:         return "EXEC_MOVE";
        case FsmState::EXEC_MOVE_FORWARD: return "EXEC_MOVE_FWD";
        case FsmState::EXEC_MOVE_BACKWARD:return "EXEC_MOVE_BWD";
        case FsmState::EXEC_ROTATE:       return "EXEC_ROTATE";
        case FsmState::EXEC_ROTATE_LEFT:  return "EXEC_ROT_LEFT";
        case FsmState::EXEC_ROTATE_RIGHT: return "EXEC_ROT_RIGHT";
        case FsmState::EXEC_STOP:         return "EXEC_STOP";
        case FsmState::EXEC_MOVE_SERVO:   return "EXEC_MOVE_SERVO";
        case FsmState::EXEC_RETRACT_SERVO:return "EXEC_RETRACT_SERVO";
        case FsmState::EMERGENCY_STOP:    return "EMERGENCY_STOP";
        default:                       return "UNKNOWN";
    }
}

static void fsm_change_state(FsmContext& ctx, FsmState newState) {
    char buf[128];
    snprintf(buf, sizeof(buf), "[FSM] %s -> %s",
             fsm_state_name(ctx.currentState), fsm_state_name(newState));
    bleSerial.println(buf);
    ctx.currentState = newState;
    ctx.stateEntryTime = millis();
}

// ── Init ─────────────────────────────────────────────────────────────────────

void fsm_init(FsmContext& ctx, QueueHandle_t cmdQueue) {
    ctx.cmdQueue       = cmdQueue;
    ctx.currentState   = FsmState::INIT;
    ctx.stateEntryTime = millis();
    ctx.cmdCount       = 0;
    memset(&ctx.currentCommand, 0, sizeof(RobotCommand));
    bleSerial.println("[FSM] Context initialized");
}

// ── Step ─────────────────────────────────────────────────────────────────────

void fsm_step(FsmContext& ctx) {

    // ── Emergency button check (from ANY state) ────────────────────────────
    // FIXME: EBTN_PIN (8) == TEAM_SWITCH_PIN (8) — conflit GPIO !
    //        Désactivé temporairement pour débloquer la FSM.
    //        Réactiver une fois le pin corrigé dans config.h.
    // if (ctx.currentState != FsmState::EMERGENCY_STOP && emergencyButton_isPressed()) {
    //     motors.stopMotors();
    //     fsm_change_state(ctx, FsmState::EMERGENCY_STOP);
    //     return;
    // }

switch (ctx.currentState) {

        // ── INIT : initialisation hardware ─────────────────────────────
        case FsmState::INIT: {
            bleSerial.println("[FSM] INIT: Initializing robot systems...");
            robot_init();   // encoders, IMU, emergency button, bras, trigger
            bleSerial.println("[FSM] INIT: Motors OK");
            bleSerial.println("[FSM] INIT: Sensors OK");
            bleSerial.println("[FSM] INIT: Ready !");
            fsm_change_state(ctx, FsmState::IDLE);
            break;
        }

        // ── IDLE : attente du launch trigger (tirette) ───────────────────
        //  Tant que la tirette n'est pas tirée, seuls PING/STATUS/RESET
        //  sont acceptés. Les commandes de mouvement sont rejetées.
        case FsmState::IDLE: {
          fsm_change_state(ctx, FsmState::DISPATCH_CMD);
            // Drain la queue — rejeter les commandes sauf PING/STATUS/RESET
            /*RobotCommand cmd;
            while (xQueueReceive(ctx.cmdQueue, &cmd, 0) == pdTRUE) {
                switch (cmd.type) {
                    case RobotCommandType::PING: {
                        char pong[64];
                        snprintf(pong, sizeof(pong), "[PONG] t=%lu ms (waiting trigger)", millis());
                        bleSerial.println(pong);
                        break;
                    }
                    case RobotCommandType::STATUS: {
                        char st[128];
                        snprintf(st, sizeof(st),
                                 "[FSM] STATUS: IDLE (waiting trigger), uptime=%lus",
                                 millis() / 1000);
                        bleSerial.println(st);
                        break;
                    }
                    case RobotCommandType::RESET:
                        bleSerial.println("[FSM] RESET: Returning to INIT");
                        fsm_change_state(ctx, FsmState::INIT);
                        return;
                    default:
                        bleSerial.println("[FSM] IDLE: Command rejected — pull trigger first");
                        break;
                }
            }*/

            // Check launch trigger
            //if (launchTrigger_isPulled()) {
            //    bleSerial.println("[FSM] Launch trigger pulled! Entering command dispatch loop");
            //    fsm_change_state(ctx, FsmState::DISPATCH_CMD);
            /*} else {
                // Log périodique pour indiquer qu'on attend
                static unsigned long lastIdleLog = 0;
                if (millis() - lastIdleLog >= 3000) {
                    bleSerial.println("[FSM] IDLE: Waiting for launch trigger...");
                    lastIdleLog = millis();
                }
            }*/
            break;
        }

        // ── DISPATCH_CMD : lire la prochaine commande de la queue ────────
        case FsmState::DISPATCH_CMD: {
            RobotCommand cmd;

            // Attendre une commande pendant 500ms max (bloquant, libère le CPU)
            if (xQueueReceive(ctx.cmdQueue, &cmd, pdMS_TO_TICKS(500)) == pdTRUE) {
                ctx.currentCommand = cmd;
                ctx.cmdCount++;

                char buf[128];
                snprintf(buf, sizeof(buf),
                         "[FSM] DISPATCH: cmd #%lu '%s' (type=%d, val=%.1f)",
                         (unsigned long)ctx.cmdCount,
                         cmd.raw,
                         (int)cmd.type,
                         cmd.value);
                bleSerial.println(buf);

                // Dispatch
                switch (cmd.type) {
                    case RobotCommandType::MOVE:
                        fsm_change_state(ctx, FsmState::EXEC_MOVE);
                        break;
                    case RobotCommandType::MOVE_FORWARD:
                        fsm_change_state(ctx, FsmState::EXEC_MOVE_FORWARD);
                        break;
                    case RobotCommandType::MOVE_BACKWARD:
                        fsm_change_state(ctx, FsmState::EXEC_MOVE_BACKWARD);
                        break;
                    case RobotCommandType::ROTATE:
                        fsm_change_state(ctx, FsmState::EXEC_ROTATE);
                        break;
                    case RobotCommandType::ROTATE_LEFT:
                        fsm_change_state(ctx, FsmState::EXEC_ROTATE_LEFT);
                        break;
                    case RobotCommandType::ROTATE_RIGHT:
                        fsm_change_state(ctx, FsmState::EXEC_ROTATE_RIGHT);
                        break;
                    case RobotCommandType::STOP:
                        fsm_change_state(ctx, FsmState::EXEC_STOP);
                        break;
                    case RobotCommandType::DEPLOY_SERVO:
                        fsm_change_state(ctx, FsmState::EXEC_MOVE_SERVO);
                        break;
                    case RobotCommandType::RETRACT_SERVO:
                        fsm_change_state(ctx, FsmState::EXEC_RETRACT_SERVO);
                        break;
                    case RobotCommandType::STATUS: {
                        char status[128];
                        snprintf(status, sizeof(status),
                                 "[FSM] STATUS: state=%s, cmdsProcessed=%lu, uptime=%lus",
                                 fsm_state_name(ctx.currentState),
                                 (unsigned long)ctx.cmdCount,
                                 millis() / 1000);
                        bleSerial.println(status);
                        break;
                    }
                    case RobotCommandType::RESET:
                        bleSerial.println("[FSM] RESET: Returning to INIT");
                        fsm_change_state(ctx, FsmState::INIT);
                        break;
                    case RobotCommandType::PING: {
                        char pong[64];
                        snprintf(pong, sizeof(pong), "[PONG] t=%lu ms", millis());
                        bleSerial.println(pong);
                        break;
                    }
                    default:
                        bleSerial.println("[FSM] DISPATCH: Unknown command, ignoring");
                        break;
                }
            }
            // Si pas de commande (timeout), on re-boucle en DISPATCH_CMD
            break;
        }

        // ── EXEC_MOVE : mouvement générique (utilise cmd.value) ───────
        case FsmState::EXEC_MOVE: {
            float dist = ctx.currentCommand.value != 0 ? ctx.currentCommand.value : 200;
            bleSerial.println("[FSM] EXEC_MOVE");
            driveDistancePID(dist, 254);
            fsm_change_state(ctx, FsmState::DISPATCH_CMD);
            break;
        }

        // ── EXEC_MOVE_FORWARD : avancer ──────────────────────────────────
        case FsmState::EXEC_MOVE_FORWARD: {
            float dist = ctx.currentCommand.value != 0 ? ctx.currentCommand.value : 200;
            bleSerial.println("[FSM] EXEC_MOVE_FORWARD");
            driveDistancePID(fabs(dist), 254);  // distance positive = avant
            fsm_change_state(ctx, FsmState::DISPATCH_CMD);
            break;
        }

        // ── EXEC_MOVE_BACKWARD : reculer ─────────────────────────────────
        case FsmState::EXEC_MOVE_BACKWARD: {
            float dist = ctx.currentCommand.value != 0 ? ctx.currentCommand.value : 200;
            bleSerial.println("[FSM] EXEC_MOVE_BACKWARD");
            driveDistancePID(-fabs(dist), 254); // distance négative = arrière
            fsm_change_state(ctx, FsmState::DISPATCH_CMD);
            break;
        }

        // ── EXEC_ROTATE : rotation générique (utilise cmd.value) ─────────
        case FsmState::EXEC_ROTATE: {
            float angle = ctx.currentCommand.value != 0 ? ctx.currentCommand.value : 90;
            bleSerial.println("[FSM] EXEC_ROTATE");
            rotateAnglePID(angle, 200);
            fsm_change_state(ctx, FsmState::DISPATCH_CMD);
            break;
        }

        // ── EXEC_ROTATE_LEFT : tourner à gauche ─────────────────────────
        case FsmState::EXEC_ROTATE_LEFT: {
            float angle = ctx.currentCommand.value != 0 ? ctx.currentCommand.value : 90;
            bleSerial.println("[FSM] EXEC_ROTATE_LEFT");
            rotateAnglePID(-fabs(angle), 200);   // angle négatif = gauche
            fsm_change_state(ctx, FsmState::DISPATCH_CMD);
            break;
        }

        // ── EXEC_ROTATE_RIGHT : tourner à droite ────────────────────────
        case FsmState::EXEC_ROTATE_RIGHT: {
            float angle = ctx.currentCommand.value != 0 ? ctx.currentCommand.value : 90;
            bleSerial.println("[FSM] EXEC_ROTATE_RIGHT");
            rotateAnglePID(fabs(angle), 200);  // angle positif = droite
            fsm_change_state(ctx, FsmState::DISPATCH_CMD);
            break;
        }

        // ── EXEC_STOP : arrêter les moteurs ─────────────────────────────
        case FsmState::EXEC_STOP: {
            motors.stopMotors();
            bleSerial.println("[FSM] EXEC_STOP: Motors stopped");
            fsm_change_state(ctx, FsmState::DISPATCH_CMD);
            break;
        }
        
        // ── EXEC_MOVE_SERVO : déplacer le servo ─────────────────────────────
        case FsmState::EXEC_MOVE_SERVO: {
            float dist = ctx.currentCommand.value != 0 ? ctx.currentCommand.value : 200;
            bleSerial.println("[FSM] EXEC_MOVE_SERVO");
            bras_deployer();
            fsm_change_state(ctx, FsmState::DISPATCH_CMD);
            break;
        }

        // ── EXEC_RETRACT_SERVO : rétracter le servo ─────────────────────────────
        case FsmState::EXEC_RETRACT_SERVO: {
            float dist = ctx.currentCommand.value != 0 ? ctx.currentCommand.value : 200;
            bleSerial.println("[FSM] EXEC_RETRACT_SERVO");
            bras_retracter();
            fsm_change_state(ctx, FsmState::DISPATCH_CMD);
            break;
        }

        // ── EMERGENCY_STOP : arrêt d'urgence — reste ici jusqu'au RESET ─
        case FsmState::EMERGENCY_STOP: {
            motors.stopMotors();
            bleSerial.println("[FSM] !!! EMERGENCY STOP !!!");
            bleSerial.println("[FSM] All systems halted. Release button + send RESET to recover.");

            // Boucle bloquante : on ne sort que sur RESET + bouton relâché
            while (true) {
                motors.stopMotors();  // sécurité : moteurs coupés en permanence
                RobotCommand cmd;
                if (xQueueReceive(ctx.cmdQueue, &cmd, pdMS_TO_TICKS(200)) == pdTRUE) {
                    if (cmd.type == RobotCommandType::RESET) {
                        if (!emergencyButton_isPressed()) {
                            bleSerial.println("[FSM] RESET received — exiting EMERGENCY STOP");
                            fsm_change_state(ctx, FsmState::INIT);
                            return;
                        } else {
                            bleSerial.println("[FSM] RESET ignored: release emergency button first!");
                        }
                    } else {
                        // Toute autre commande est ignorée
                        bleSerial.println("[FSM] Command ignored in EMERGENCY_STOP. Send RESET.");
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            break;
        }
    }
}
