#include "robot.h"

Motors motors(ENA, IN1, IN2, ENB, IN3, IN4);


void robot_init() {
  // Encoders
  encoders_init();
  // Ultrasonic init fait dans ultrasonic.cpp
  
  emergencyButton_init();
  // safety_init(40, 50);      // 40cm seuil, sonar toutes les 50ms

  // IMU
  if (!imu_init()) { Serial.println("MPU6050 FAIL.");
  } else {
    delay(200);
    Serial.println("MPU6050 connected.");
    imu_calibrate(600, 2); // ~1.2s, robot immobile
    // Serial.println("IMU calibrated");
  }
}

void robot_stop(){ 
  motors.stopMotors(); 
}
void robot_test() {
  motors.forward(150, 150);
  delay(2000);
  motors.stopMotors();

  // while(true);
}

// AVANT et ARRIERE (+ , -)
void driveDistancePID(float distance_mm, int speed) {
  
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

  while (true) {
    unsigned long now = micros();
    if ((unsigned long)(now - tPrev) < (unsigned long)DT_MS * 1000UL) {yield(); continue;}
    tPrev += (unsigned long)DT_MS * 1000UL;

    // Serial.print("Safety check: "); Serial.println(safety_update() ? "STOP" : "."); 
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


    // STOP MOTOR CONDITIONS
    if (safety_update()) {
      static unsigned long lp3 = 0; 
      motors.stopMotors();
      if (DBG_MOTORS) Serial.print("Safety triggered\n");
      continue;
    }

    // --- Read Current encoders Values ---
    long curL, curR;
    encoders_read(&curL, &curR);

    // --- Compute distance traveled ---
    long distTicksL = labs(curL - startL);
    long distTicksR = labs(curR - startR);
    long avgDist = (distTicksL + distTicksR) / 2;
    if (avgDist >= targetTicks) break; // Target Reached

    // --- Compute deltas for PID ---
    long dL, dR;
    encoders_computeDelta(curL, curR, &dL, &dR);

    // Heading error (cumulative) - erreur de cap cumulée (position)
    long headingErr = (curL - startL) - (curR - startR);
    int pwmL=0, pwmR=0;
    control_driveStraight_PI(st, headingErr, dL, dR, speed, dt, pwmL, pwmR);

    // Automatically handle direction
        if (!forwardMotion) { pwmL = -pwmL; pwmR = -pwmR; } //invers PWM pour marche arriere

    // Moteurs PWM vitesse
    motors.applyMotorOutputs(pwmL, pwmR);
      // Debug print every 1s
        static unsigned long Lpwm = 0;
        if (DBG_MOTORS && millis() - Lpwm >= 1000) {
            bleSerial.print("PWM L: ");
            bleSerial.print(pwmL);
            bleSerial.print(" | PWM R: ");
            bleSerial.println(pwmR);
            Lpwm = millis();
        }
  }
  
  // Movement complete - stop motors
  motors.stopMotors();
    debugPrintf(DBG_MOTORS, "Target distance reached\n");
}

void rotateAnglePID(float angle_deg, int speed) {
  // Important : 
  //    - this is a PD controller (no I-term), rate is dirctly used as D-term (gyro rate = damping).

  // ----- Setup Timing -----
  const uint16_t DT_MS = 10;           // Loop runs every 10ms
  const float dt = DT_MS / 1000.0f;   // dt = 0.01s
  unsigned long tPrev = micros();    // Control freq = 100Hz

  // ----- Variables -------
  float angle = 0.0f;
  int pwmLimit = 0;
  unsigned long stableStart = 0;

  // ---- Parameters (à ajuster) ------
  const float KP = 2.2f;
  const float KD = 0.25f;

  const int PWM_MIN = 55;
  const int RAMP_STEP = 8;

  const float ANGLE_TOL = 1.5f;
  const float RATE_TOL  = 8.0f;
  const uint16_t STABLE_MS = 120;

  // ----- Control Loop -----
  while (true) {
    // Fixed 100Hz timing -----
    unsigned long now = micros();
    if ((unsigned long)(now - tPrev) < (unsigned long)DT_MS * 1000UL) {yield(); continue;}
    tPrev += (unsigned long)DT_MS * 1000UL;

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
    // Read Gyro -----
    float rate = imu_readGyroZ_dps();   // deg/s
    angle += rate * dt;                 // integrate

    if (DBG_MOTORS) { Serial.print("Angle: "); Serial.print(angle); Serial.print(" | Rate: "); Serial.println(rate); }

    // Compute Error -----
    float error = angle_deg - angle;

    // Ramp PWM Limit -----
    if (pwmLimit < speed)
        pwmLimit = min(pwmLimit + RAMP_STEP, speed);

    // PD Control -----
    float control = KP * error - KD * rate;

    // Convert to PWM -----
    int pwm = (int)fabs(control);
    pwm = constrain(pwm, 0, pwmLimit);
    if (pwm > 0) pwm = max(pwm, PWM_MIN);

    // ----- Apply Direction -----
    if (control > 0)
        motors.applyMotorOutputs(-pwm, pwm);   // rotate right
    else
        motors.applyMotorOutputs(pwm, -pwm);      // rotate left

    // ----- Debug -----
    if (DBG_MOTORS)
    {
        Serial.print("Angle: "); Serial.print(angle);
        Serial.print(" | Error: "); Serial.print(error);
        Serial.print(" | Rate: "); Serial.print(rate);
        Serial.print(" | PWM: "); Serial.println(pwm);
    }

    // ----- Stop Condition -----
    if (fabs(error) < ANGLE_TOL && fabs(rate) < RATE_TOL)
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
  // Movement complete - stop motors
  motors.stopMotors();
    debugPrintf(DBG_MOTORS, "Target Angle reached\n");
}





// ----- LEGACY--------
void robot_step() { // On va aussi plus l'utiliser normalement
  long left, right;
  encoders_read(&left, &right);

  long dL, dR;
  encoders_computeDelta(left, right, &dL, &dR);

  int speedL, speedR;
  control_computeSpeeds(dL, dR, speedL, speedR);

  // motors_applySpeeds(speedL, speedR);
}

void robot_rotate(float angle_deg, int speed){
  //   long targetTicks = ticks_for_rotation_deg(angle_deg);

  //   long startL, startR;
  //   encoders_read(&startL, &startR);

  //   // choix du sens
  //   if (angle_deg > 0) {
  //       motors_rotateRight(speed);   // droite = angle positif
  //   } else {
  //       motors_rotateLeft(speed);    // gauche = angle négatif
  //   }

  //   while (true) {
  //       long curL, curR;
  //       encoders_read(&curL, &curR);

  //       long dL = labs(curL - startL);
  //       long dR = labs(curR - startR);

  //       if ((dL + dR) / 2 >= labs(targetTicks)) {
  //       break;
  //       }
  //   }

  // motors_stop();
  const uint16_t DT_MS = 10;
  unsigned long tPrev = micros();

  long targetTicks = ticks_for_rotation_deg(angle_deg);

  long startL, startR;
  encoders_read(&startL, &startR);

  // if (angle_deg > 0) motors_rotateRight(speed);
  // else              motors_rotateLeft(speed);

  

  while (true) {
    unsigned long now = micros();
    if ((unsigned long)(now - tPrev) < (unsigned long)DT_MS * 1000UL) continue;
    tPrev += (unsigned long)DT_MS * 1000UL;

    safety_update();
    if (safety_isTriggered()) {
      // motors_stop();
      return;   // arrêt immédiat
    }


    long curL, curR;
    encoders_read(&curL, &curR);

    long dL = labs(curL - startL);
    long dR = labs(curR - startR);

    if ((dL + dR) / 2 >= labs(targetTicks)) break;
  }

  // motors_stop();
}

void robot_rotate_gyro(float target_deg, int pwmMax) {
  const uint16_t DT_MS = 10;
  const float dt = DT_MS / 1000.0f;

  // Gains (départs, à tuner)
  const float KP = 2.2f;   // PWM par degré d'erreur
  const float KD = 0.25f;  // PWM par (deg/s) pour amortir

  const int PWM_MIN = 55;   // PWM mini qui fait tourner (à ajuster)
  const int DEAD_PWM = 0;   // laisse 0 ou PWM_MIN selon ton robot

  // Conditions d'arrêt
  const float ANGLE_TOL = 1.5f;   // degrés
  const float RATE_TOL  = 8.0f;   // deg/s
  const uint16_t STABLE_MS = 120; // durée stable avant stop

  // Rampe PWM max (évite patinage)
  const int RAMP_STEP = 8; // par 10ms

  // Signe: + target => tourne à droite (comme ton code)
  float angle = 0.0f;
  unsigned long tPrev = micros();
  unsigned long stableStart = 0;

  int pwmLimit = 0;

  while (true) {
    unsigned long now = micros();
    if ((unsigned long)(now - tPrev) < (unsigned long)DT_MS * 1000UL) continue;
    tPrev += (unsigned long)DT_MS * 1000UL;

    // safety_update();
    // if (safety_isTriggered()) {
    //   // motors_stop();
    //   return;   // arrêt immédiat
    // }


    // lecture gyro
    float rate = imu_readGyroZ_dps(); // deg/s (bias retiré)
    angle += rate * dt;

    float err = target_deg - angle;

    // Rampe de la limite PWM
    if (pwmLimit < pwmMax) pwmLimit = min(pwmLimit + RAMP_STEP, pwmMax);

    // PD
    float u = KP * err - KD * rate;

    int pwm = (int)fabs(u);
    pwm = constrain(pwm, 0, pwmLimit);

    if (pwm > 0) pwm = max(pwm, PWM_MIN);
    else pwm = DEAD_PWM;

    // applique sens selon u
    // if (u > 0) motors_rotateRight(pwm);
    // else       motors_rotateLeft(pwm);

    // arrêt : proche de la cible ET vitesse faible pendant STABLE_MS
    if (fabs(err) < ANGLE_TOL && fabs(rate) < RATE_TOL) {
      if (stableStart == 0) stableStart = millis();
      if (millis() - stableStart >= STABLE_MS) break;
    } else {
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
    if (ctx.currentState != FsmState::EMERGENCY_STOP && emergencyButton_isPressed()) {
        motors.stopMotors();
        fsm_change_state(ctx, FsmState::EMERGENCY_STOP);
        return;
    }

switch (ctx.currentState) {

        // ── INIT : initialisation hardware (placeholder) ─────────────────
        case FsmState::INIT: {
            bleSerial.println("[FSM] INIT: Initializing robot systems...");
            bleSerial.println("[FSM] INIT: Motors OK (simulated)");
            bleSerial.println("[FSM] INIT: Sensors OK (simulated)");
            bleSerial.println("[FSM] INIT: Ready !");
            fsm_change_state(ctx, FsmState::IDLE);
            break;
        }

        // ── IDLE : attente, on passe directement en DISPATCH ─────────────
        case FsmState::IDLE: {
            bleSerial.println("[FSM] IDLE: Entering command dispatch loop");
            fsm_change_state(ctx, FsmState::DISPATCH_CMD);
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
            rotateAnglePID(-fabs(angle), 200);  // angle négatif = gauche
            fsm_change_state(ctx, FsmState::DISPATCH_CMD);
            break;
        }

        // ── EXEC_ROTATE_RIGHT : tourner à droite ────────────────────────
        case FsmState::EXEC_ROTATE_RIGHT: {
            float angle = ctx.currentCommand.value != 0 ? ctx.currentCommand.value : 90;
            bleSerial.println("[FSM] EXEC_ROTATE_RIGHT");
            rotateAnglePID(fabs(angle), 200);   // angle positif = droite
            fsm_change_state(ctx, FsmState::DISPATCH_CMD);
            break;
        }

        // ── EXEC_STOP : arrêter les moteurs ─────────────────────────────
        case FsmState::EXEC_STOP: {
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
            bleSerial.println("[FSM] All systems halted. Send RESET to recover.");

            // Boucle bloquante : on ne sort que sur RESET
            while (true) {
                motors.stopMotors();  // sécurité : moteurs coupés en permanence
                RobotCommand cmd;
                if (xQueueReceive(ctx.cmdQueue, &cmd, pdMS_TO_TICKS(200)) == pdTRUE) {
                    if (cmd.type == RobotCommandType::RESET) {
                        bleSerial.println("[FSM] RESET received — exiting EMERGENCY STOP");
                        fsm_change_state(ctx, FsmState::INIT);
                        return;
                    }
                    // Toute autre commande est ignorée
                    bleSerial.println("[FSM] Command ignored in EMERGENCY_STOP. Send RESET.");
                }
                vTaskDelay(pdMS_TO_TICKS(50));
            }
            break;
        }
    }
}









