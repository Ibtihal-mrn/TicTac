#include "robot.h"
#include "../../src/config.h"
#include "../../src/globals.h"

#include "encoders.h"
#include "motors.h"
#include "control.h"
#include "kinematics.h"
#include "imu.h"
#include "safety.h"
#include "ultrasonic.h"
#include "utils.h"
#include "Debug.h"

Motors motors(ENA, IN1, IN2, ENB, IN3, IN4);


void robot_init() {
  encoders_init();
  // ultrasonic_init(13, 10);  // trig, echo
  // safety_init(40, 50);      // 40cm seuil, sonar toutes les 50ms

  // IMU
  // if (!imu_init()) { Serial.println("MPU6050 FAIL.");
  // } else {
  //   delay(200);
  //   Serial.println("MPU6050 connected.");
  //   imu_calibrate(600, 2); // ~1.2s, robot immobile
  //   // Serial.println("IMU calibrated");
  // }
}

void robot_test() {
  motors.forward(150, 150);
  delay(2000);
  motors.stopMotors();

  // while(true);
}



void robot_step() { // On va aussi plus l'utiliser normalement
  long left, right;
  encoders_read(&left, &right);

  long dL, dR;
  encoders_computeDelta(left, right, &dL, &dR);

  int speedL, speedR;
  control_computeSpeeds(dL, dR, speedL, speedR);

  // motors_applySpeeds(speedL, speedR);
}

void robot_stop(){ motors.stopMotors(); }

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

    safety_update();
    if (safety_isTriggered()) {
      // motors_stop();
      return;   // arrêt immédiat
    }


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

    // (optionnel) sécurité timeout
    // if (millis() - startMs > 4000) break;
  }

  // motors_stop();
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


  // Setup Timing
  const uint16_t DT_MS = 10;
  const float dt = DT_MS / 1000.0f;

  // Compute Tick Target
  long target = ticks_for_distance_mm(fabs(dist_mm));
  unsigned long lp1 = 0;
  printMillis(DBG_MOTORS, "Target computed\n", millis(), lp1, 1000);

  // Encoder Read
  long startL, startR;
  encoders_read(&startL, &startR);
  unsigned long lp2 = 0;
  printMillis(DBG_MOTORS, "Encoders computed\n", millis(), lp2, 1000);

  // reset deltas encodeurs pour la vitesse
  prevL = startL;
  prevR = startR;

  // Reset Controller State
  DrivePIState st;
  control_reset(st);

  unsigned long tPrev = micros();

  // MAIN LOOP
  while (true) {
    // Fixed 10ms while Loop
    unsigned long now = micros();
    if ((unsigned long)(now - tPrev) < (unsigned long)DT_MS * 1000UL) {yield(); continue;}
    tPrev += (unsigned long)DT_MS * 1000UL;

    // STOP MOTOR CONDITIONS
    safety_update();
    if (safety_isTriggered()) {
      motors.stopMotors();
      // Blocking Loop
      while(safety_isTriggered()){
        static unsigned long lp3 = 0;
        printMillis(DBG_MOTORS, "Safety triggered\n", millis(), lp3, 2000);
        safety_update();
        safety_clearIfSafe();
        delay(20);
      }
    }


    // Encoders Update
    long curL, curR;
    encoders_read(&curL, &curR);

    long distTicksL = labs(curL - startL);
    long distTicksR = labs(curR - startR);
    if ((distTicksL + distTicksR) / 2 >= target) break; // Target Reached
    static unsigned long lp4 = 0;
    printMillis(DBG_MOTORS, "Target not reached...\n", millis(), lp4, 500);

    // PI CONTROL
    long dL, dR;
    encoders_computeDelta(curL, curR, &dL, &dR); // deltas (vitesse)

    // erreur de cap cumulée (position)
    long headingErr = (curL - startL) - (curR - startR);
    int pwmL, pwmR;
    control_driveStraight_PI(st, headingErr, dL, dR, pwmBaseTarget, dt, pwmL, pwmR);

    // motors_applySpeeds(pwmL, pwmR);

    static unsigned long lp5 = 0;
    if (millis() - lp5 >= 2000) {
        Serial.print("PWM L: ");
        Serial.print(pwmL);
        Serial.print(" | PWM R: ");
        Serial.println(pwmR);
        lp5 = millis();
    }

    // motors.forward(pwmL, pwmR);
    motors.forward(255, 255);
    
  }

  // Default (target reached when outside while loop).
  motors.stopMotors();
}



void robot_forward_distance(float dist_mm, int speed) {
  
  // Compute Tick Target
  long target = ticks_for_distance_mm(fabs(dist_mm));
  unsigned long lp1 = 0;
  printMillis(DBG_MOTORS, "Target computed\n", millis(), lp1, 1000);

  // Encoder Read
  long startL, startR;
  encoders_read(&startL, &startR);
  unsigned long lp2 = 0;
  printMillis(DBG_MOTORS, "Encoders computed\n", millis(), lp2, 1000);

  // reset deltas encodeurs pour la vitesse
  prevL = startL;
  prevR = startR;

  // Setup Timing
  const uint16_t DT_MS = 10;
  const float dt = DT_MS / 1000.0f;
  unsigned long tPrev = micros();

  while (true) {

    unsigned long now = micros();
    if ((unsigned long)(now - tPrev) < (unsigned long)DT_MS * 1000UL) {yield(); continue;}
    tPrev += (unsigned long)DT_MS * 1000UL;

    motors.forward(150,150);

    long curL, curR;
    encoders_read(&curL, &curR);

    long distTicksL = labs(curL - startL);
    long distTicksR = labs(curR - startR);

    if ((distTicksL + distTicksR) / 2 >= target) break;
  }
}

// ===========

void hardware_init(Context& ctx) {
  // To be called in setup() in main.cpp

  // motors_init();
  // encoders_init();
  // ultrasonic_init(13, 10);  // trig, echo
  // safety_init(40, 50);      // 40cm seuil, sonar toutes les 50ms

  // IMU
  if (!imu_init()) { Serial.println("MPU6050 FAIL.");
  } else {
    delay(200);
    Serial.println("MPU6050 connected.");
    imu_calibrate(600, 2); // ~1.2s, robot immobile
    // Serial.println("IMU calibrated");
  }

  // Init Match Timer
  ctx.matchActive = false;
  ctx.matchDurationMs = MATCH_DURATION_MS;
  ctx.matchStartMs = 0;
    debugPrintf(DBG_FSM, "FSM -> INIT");
    ctx.currentAction = Robot::INIT;
}

void startMatchTimer(Context& ctx) {
    ctx.matchActive = true;
    ctx.matchStartMs = millis();
    ctx.stateStartMs = millis();
    ctx.matchDurationMs = MATCH_DURATION_MS;
}
void checkMatchTimer(Context& ctx) {
    if (ctx.matchActive && millis() - ctx.matchStartMs >= ctx.matchDurationMs) {
        ctx.matchActive = false;
        debugPrintf(DBG_FSM, "Match timer elapsed -> TIMER_END");
        ctx.currentAction = Robot::TIMER_END;
    }
}




void robot_step(Context& ctx) {

    // Periodic Checks
    checkMatchTimer(ctx);



    // Main step.
    switch (ctx.currentAction) {
        case Robot::INIT:
            // robot_init();  CALLED IN SETUP()
            // ctx.commandQueue.push({RobotCommandType::TUNE_PID, (float)TUNE_BOTH});
            debugPrintf(DBG_FSM, "FSM -> IDLE");
            ctx.currentAction = Robot::IDLE;
            break;

        case Robot::IDLE:
            // ADD LaunchTrigger HERE

            // start 100sec timer
            startMatchTimer(ctx);

            debugPrintf(DBG_FSM, "FSM -> DISPATCH_CMD");
            ctx.currentAction = Robot::DISPATCH_CMD;
            break;

        case Robot::DISPATCH_CMD:
            // parse command and set next action (e.g. EXEC_MOVE, EXEC_ROTATE, etc.)

            // 1. Empty Queue
            // if (ctx.commandQueue.empty()) { break; }

            // 2. Get next command
            // ctx.currentCommand = ctx.commandQueue.front();
            // ctx.commandQueue.pop();

            // 3. Dispatch to movement
            // switch (ctx.currentCommand.type) {
            //     case  RobotCommandType::MOVE_FORWARD_CM:
            //         movement.startForward(ctx.currentCommand.value);
            //         fsmChangeAction(ctx, FsmAction::EXEC_MOVE);
            //         break;

            //     case RobotCommandType::ROTATE_DEG:
            //         movement.startRotate(ctx.currentCommand.value);
            //         fsmChangeAction(ctx, FsmAction::EXEC_ROTATE);
            //         break;

            //     case RobotCommandType::TUNE_PID:
            //         fsmChangeAction(ctx, FsmAction::TUNE_PID);
            //         break;

            //     // case STEPPER_UP:

            //     default: fsmChangeAction(ctx, FsmAction::DISPATCH_CMD); break;
            // }
            // break;
          // }
            break;

        case Robot::EXEC_MOVE:
            
            break;

        case Robot::EXEC_ROTATE:
            // robot_rotate(...);
            break;


        case Robot::TUNE_PID:
            // adjust PID parameters
            break;

        case Robot::TIMER_END:
            motors.stopMotors();
            break;

        case Robot::EMERGENCY_STOP:
            motors.stopMotors();
            break;
    }
}









