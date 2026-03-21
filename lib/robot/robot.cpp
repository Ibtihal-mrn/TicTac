#include "robot.h"
#include "../../src/config.h"
#include "../../src/globals.h"

#include "motors.h"
#include "encoders.h"
#include "us.h"
#include "imu.h"

#include "control.h"
#include "kinematics.h"

//
#include "utils.h"
#include "Debug.h"

// I2C sensors
#include "i2c_comm.h"




Motors motors(ENA, IN1, IN2, ENB, IN3, IN4);


// TODO: Replace by hardware_init()
void robot_init()
{
  // Encoders
  encoders_init();

  // IMU
  if (!imu_init())
  {
    Serial.println("MPU6050 FAIL.");
  }
  else
  {
    delay(200);
    Serial.println("MPU6050 connected.");
    imu_calibrate(600, 2); // ~1.2s, robot immobile
    // Serial.println("IMU calibrated");
  }
}





// ======= PID MOVEMENT =========

void driveFroward(float mm, int speed){
  setZones(ZONE_FRONT);
  driveDistancePID(mm, speed);
}

void driveBackward(float mm, int speed){
  setZones(ZONE_BACK);
  driveDistancePID(mm, speed);
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

    // ========== STOP MOTOR CONDITIONS ============
    if (emergencyStop)
    {
      motors.stopMotors();
      unsigned long lp6 = 0;
      printMillis(DBG_MOTORS, "Emergency Button !\n", millis(), lp6, 1000);
      while (emergencyStop) { yield(); }

      // Reset PID values after interruption
      control_reset(st);

      // Reset timing
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

    // ----- Debug -----
    #if DBG_MOTORS
      static unsigned long Lpwm = 0;
      if (millis() - Lpwm >= 1000){
        Serial.print("PWM L: "); Serial.print(pwmL);
        Serial.print(" | PWM R: "); Serial.println(pwmR);
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
  // Important :
  //    - this is a PD controller (no I-term), rate is dirctly used as D-term (gyro rate = damping).

  // ----- Setup Timing -----
  const uint16_t DT_MS = 10;      // Loop runs every 10ms
  unsigned long tPrev = micros(); // Control freq = 100Hz
  unsigned long startMs = millis();
  const float targetDeg = angle_deg * ROTATE_TARGET_SCALE;

  // ----- Variables -------
  float angle = 0.0f;
  int pwmLimit = 0;
  unsigned long stableStart = 0;

  // ---- Parameters (à ajuster) ------
  const float KP = ROTATE_KP;
  const float KD = ROTATE_KD;

  const int PWM_MIN = 55;
  const int RAMP_STEP = 8;
  const float BRAKE_START_DEG = 45.0f;

  const float ANGLE_TOL = 1.5f;
  const float RATE_TOL = 8.0f;
  const uint16_t STABLE_MS = 120;
  const uint32_t MAX_ROTATE_MS = (uint32_t)(2500.0f + 22.0f * fabs(targetDeg));

  // ----- Control Loop -----
  while (true)
  {
    // Fixed 100Hz timing -----
    unsigned long now = micros();
    if ((unsigned long)(now - tPrev) < (unsigned long)DT_MS * 1000UL){ yield(); continue; }
    tPrev += (unsigned long)DT_MS * 1000UL;

    // ========== STOP MOTOR CONDITIONS ============
    if (emergencyStop)
    {
      motors.stopMotors();
      unsigned long lp6 = 0;
      printMillis(DBG_MOTORS, "Emergency Button !\n", millis(), lp6, 1000);
      while (emergencyStop) { yield(); }

      // Reset PID values after interruption
      // control_reset(st);

      // Reset timing
      tPrev = micros();

      continue;
    }


    float dt = (float)(now - tPrev) / 1000000.0f;
    tPrev = now;
    dt = constrain(dt, 0.005f, 0.03f);

    if (millis() - startMs >= MAX_ROTATE_MS)
    {
      debugPrintf(DBG_MOTORS, "rotateAnglePID timeout\n");
      break;
    }

    // Read Gyro -----
    float rate = imu_readGyroZ_dps(); // deg/s
    angle += rate * dt;               // integrate

    // Compute Error -----
    float error = targetDeg - angle;

    // Ramp PWM Limit -----
    if (pwmLimit < speed)
      pwmLimit = min(pwmLimit + RAMP_STEP, speed);

    // PD Control -----
    float control = KP * error - KD * rate;

    // Freinage progressif en approche de la cible
    int pwmCap = pwmLimit;
    float absErr = fabs(error);
    if (absErr < BRAKE_START_DEG)
    {
      float ratio = absErr / BRAKE_START_DEG; // 1 loin -> 0 proche cible
      int brakeCap = PWM_MIN + (int)((speed - PWM_MIN) * ratio);
      pwmCap = min(pwmCap, brakeCap);
    }

    // Convert to PWM -----
    int pwm = (int)fabs(control);
    pwm = constrain(pwm, 0, pwmCap);
    if (pwm > 0)
      pwm = max(pwm, PWM_MIN);

    // ----- Apply Direction -----
    if (control > 0)
      motors.applyMotorOutputs(-pwm, pwm); // rotate right
    else
      motors.applyMotorOutputs(pwm, -pwm); // rotate left

    // ----- Debug -----
    #if DBG_MOTORS
      static unsigned long Lpwm = 0;
      if (millis() - Lpwm >= 1000){
        Serial.print("Angle: ");
        Serial.print(angle);
        Serial.print(" | Error: ");
        Serial.print(error);
        Serial.print(" | Rate: ");
        Serial.print(rate);
        Serial.print(" | PWM: ");
        Serial.println(pwm);
        Lpwm = millis();
      }
    #endif

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




// ===========

void hardware_init(Context &ctx)
{
  // To be called in setup() in main.cpp

  // motors_init();
  // encoders_init();
  // ultrasonic_init(13, 10);  // trig, echo
  // safety_init(40, 50);      // 40cm seuil, sonar toutes les 50ms

  // IMU
  if (!imu_init())
  {
    Serial.println("MPU6050 FAIL.");
  }
  else
  {
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

void startMatchTimer(Context &ctx)
{
  ctx.matchActive = true;
  ctx.matchStartMs = millis();
  ctx.stateStartMs = millis();
  ctx.matchDurationMs = MATCH_DURATION_MS;
}
void checkMatchTimer(Context &ctx)
{
  if (ctx.matchActive && millis() - ctx.matchStartMs >= ctx.matchDurationMs)
  {
    ctx.matchActive = false;
    debugPrintf(DBG_FSM, "Match timer elapsed -> TIMER_END");
    ctx.currentAction = Robot::TIMER_END;
  }
}

void robot_step(Context &ctx)
{

  // Periodic Checks
  checkMatchTimer(ctx);

  // BLE connect

  // Main step.
  switch (ctx.currentAction)
  {
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