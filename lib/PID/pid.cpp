#include "pid.h"
#include "Debug.h"
#include <math.h>

const PID DISTANCE_PID_DEFAULT(0.6f, 0.03f, 0.05f);
const PID ANGLE_PID_DEFAULT(5.5f, 0.0f, 0.22f);

PIDController::PIDController(Motors& motors)
    : motors_(motors),
    distancePid_(DISTANCE_PID_DEFAULT),
    anglePid_(ANGLE_PID_DEFAULT),
    mode_(Mode::Idle),
    targetDistanceMm_(0.0f),
    traveledDistanceMm_(0.0f),
    targetAngleDeg_(0.0f),
    headingDeg_(0.0f),
    maxPwm_(0),
    lastUpdateUs_(0),
    stableSinceMs_(0),
    motionStartMs_(0)
{
}

// ------ Helpers ------
void PIDController::setDistancePID(const PID& pid) { distancePid_ = pid; }

void PIDController::setAnglePID(const PID& pid) { anglePid_ = pid; }

void PIDController::resetState() {
    traveledDistanceMm_ = 0.0f;
    headingDeg_ = 0.0f;
    stableSinceMs_ = 0;
    lastUpdateUs_ = micros();

    distancePid_.integral = 0.0f;
    distancePid_.previousError = 0.0f;
    anglePid_.integral = 0.0f;
    anglePid_.previousError = 0.0f;

    encoders_reset();
}

static float calcTrimHintFromTicks(long leftTicks, long rightTicks) {
    const long total = labs(leftTicks) + labs(rightTicks);
    if (total <= 0) return 0.0f;

    const float balance = (float)(rightTicks - leftTicks) / (float)total;
    return balance * 30.0f;
}

static float applyMinPwm(float value, float minPwm) {
    if (fabsf(value) > 0.01f && fabsf(value) < minPwm) {
        return copysignf(minPwm, value);
    }
    return value;
}


// ===== Start Movement =========
void PIDController::startLinear(float distanceMm, int maxPwm) {
    targetDistanceMm_ = distanceMm;
    targetAngleDeg_ = 0.0f;
    maxPwm_ = constrain(maxPwm, 0, PWM_MAX);
    mode_ = Mode::Linear;
    resetState();
    motionStartMs_ = millis();
}

void PIDController::startRotate(float angleDeg, int maxPwm) {
    targetDistanceMm_ = 0.0f;
    targetAngleDeg_ = angleDeg;
    maxPwm_ = constrain(maxPwm, 0, PWM_MAX);
    mode_ = Mode::Rotate;
    resetState();
    motionStartMs_ = millis();
}

void PIDController::abort() {
    stopMotors_();
    mode_ = Mode::Idle;
}

bool PIDController::isBusy() const {
    return mode_ != Mode::Idle;
}

PIDController::Mode PIDController::mode() const {
    return mode_;
}

void PIDController::stopMotors_() {
    motors_.stopMotors();
}

float PIDController::nextDt_() {
    unsigned long nowUs = micros();
    if (lastUpdateUs_ == 0) {
        lastUpdateUs_ = nowUs;
        return 0.01f;
    }

    float dt = (nowUs - lastUpdateUs_) / 1000000.0f;
    lastUpdateUs_ = nowUs;

    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.05f) dt = 0.05f;
    return dt;
}

float PIDController::updatePID_(PID& pid, float error, float dt) {
    pid.integral += error * dt;
    pid.integral = constrain(pid.integral, -INTEGRAL_CLAMP, INTEGRAL_CLAMP);

    float derivative = 0.0f;
    if (dt > 0.0f) {
        derivative = (error - pid.previousError) / dt;
    }

    pid.previousError = error;
    return pid.kp * error + pid.ki * pid.integral + pid.kd * derivative;
}

bool PIDController::update() {
    if (mode_ == Mode::Idle) return true;
    
    // Movement TIMEOUT   //TODO: dynamically compute timeout on distance and speed
    if (motionStartMs_ != 0 && (millis() - motionStartMs_ >= MOTION_TIMEOUT_MS)) {
        stopMotors_();
        mode_ = Mode::Idle;
        stableSinceMs_ = 0;
        return true;
    }


    // Read Encoders, Gyro and convert ticks to distance
    const float dt = nextDt_(); // time since last update

    long leftTicks = 0;
    long rightTicks = 0;
    encoders_read(&leftTicks, &rightTicks);

    long deltaLeft = 0;
    long deltaRight = 0;
    encoders_computeDelta(leftTicks, rightTicks, &deltaLeft, &deltaRight);

    const float gyroRateDps = imu_readGyroZ_dps();
    headingDeg_ += gyroRateDps * dt;


    // ====================== LINEAR =========================
    if (mode_ == Mode::Linear) {
        const float motionSign = (targetDistanceMm_ >= 0.0f) ? 1.0f : -1.0f;
    
        const float deltaDistanceMm = 0.5f * (deltaLeft + deltaRight) * mm_per_tick();
        traveledDistanceMm_ += motionSign * deltaDistanceMm;
    
        const float distanceError = fabsf(targetDistanceMm_) - fabsf(traveledDistanceMm_);
        const float headingError = -headingDeg_;
    
        const float baseSpeed = motionSign * 150.0f;
    
        float steerCmd = updatePID_(anglePid_, headingError, dt);
        steerCmd = constrain(steerCmd, -40.0f, 40.0f);
    
        float leftCmd  = baseSpeed - steerCmd;
        float rightCmd = baseSpeed + steerCmd;
    
        motors_.applyMotorOutputs(leftCmd, rightCmd);
    
        if (fabsf(distanceError) <= DONE_DISTANCE_MM) {
            stopMotors_();
            mode_ = Mode::Idle;
            return true;
        }
    
        return false;
    }
    // OLD impl
    // if (false) {
    //     // 1. Total Travelled distance
    //     const float deltaDistanceMm = 0.5f * (deltaLeft + deltaRight) * mm_per_tick();
    //     traveledDistanceMm_ += deltaDistanceMm;
    
    //     // 2. Total Error (distance left to go)
    //     const float distanceError = targetDistanceMm_ - traveledDistanceMm_;
    //     const float headingError = -headingDeg_; // how much we drifted : target heading = 0
    
    //     // 3. PID compute
    //     float linearCmd  = updatePID_(distancePid_, distanceError, dt);
    //     float angularCmd = updatePID_(anglePid_, headingError, dt);
    
    //     // 4. Reserve steering headroom
    //     const float steerHeadroom = 50.0f;
    //     const float linearLimit = max(0.0f, (float)maxPwm_ - steerHeadroom);
    
    //     linearCmd = constrain(linearCmd, -linearLimit, linearLimit);
    //     angularCmd = constrain(angularCmd, -steerHeadroom, steerHeadroom);
    
    //     // 5. Apply a floor to the forward command, not the wheels
    //     float forwardCmd = linearCmd;
    //     const bool farFromTarget = fabsf(distanceError) > DONE_DISTANCE_MM;
    
    //     if (farFromTarget) {
    //         forwardCmd = applyMinPwm(forwardCmd, min((float)PWM_MIN_LINEAR, linearLimit));
    //     }
    
    //     // 6. Mix forward + steering into wheel outputs
    //     float leftCmd  = forwardCmd - angularCmd;
    //     float rightCmd = (forwardCmd + angularCmd) * 1.03;
    
    //     // 7. Optional trim — start at 0 for now
    //     const float trim = 0.0f;
    //     leftCmd  -= trim;
    //     rightCmd += trim;
    
    //     // 8. Clamp final wheel outputs
    //     leftCmd  = constrain(leftCmd,  -maxPwm_, maxPwm_);
    //     rightCmd = constrain(rightCmd, -maxPwm_, maxPwm_);
    
    //     // 9. Debug prints
    //     #if DBG_PID
    //     static unsigned long lastPidPrintMs = 0;
    //     if (millis() - lastPidPrintMs >= 500) {
    //         const float tickRatio = (fabsf((float)rightTicks) > 0.001f)
    //             ? ((float)leftTicks / (float)rightTicks)
    //             : 0.0f;

    //         const float deltaRatio = (fabsf((float)deltaRight) > 0.001f)
    //             ? ((float)deltaLeft / (float)deltaRight)
    //             : 0.0f;

    //         const float trimHint = calcTrimHintFromTicks(leftTicks, rightTicks);

    //         Serial.print("[PID LIN] ");
    //         Serial.print("dt="); Serial.print(dt, 3);
    //         Serial.print(" lt="); Serial.print(leftTicks);
    //         Serial.print(" rt="); Serial.print(rightTicks);
    //         Serial.print(" dL="); Serial.print(deltaLeft);
    //         Serial.print(" dR="); Serial.print(deltaRight);
    //         Serial.print(" targetD="); Serial.print(targetDistanceMm_);
    //         Serial.print(" travelledD="); Serial.print(traveledDistanceMm_);
    //         Serial.print(" distError="); Serial.print(distanceError);

    //         Serial.print(" head="); Serial.print(headingDeg_, 2);
    //         Serial.print(" headingError="); Serial.print(headingError, 2);
    //         Serial.print(" gyro="); Serial.print(gyroRateDps, 2);
    //         Serial.print(" linearCmd="); Serial.println(linearCmd, 2);
    //         Serial.print(" angularCmd="); Serial.println(angularCmd, 2);
    //         Serial.print(" forwardCmd="); Serial.print(forwardCmd, 2);
    //         Serial.print(" farFromTarget="); Serial.print(farFromTarget);
    //         Serial.print(" floorPwm="); Serial.print(min((float)PWM_MIN_LINEAR, linearLimit));
    //         Serial.print(" tickRatio="); Serial.print(tickRatio, 3);
    //         Serial.print(" deltaRatio="); Serial.print(deltaRatio, 3);
    //         Serial.print(" trimHint="); Serial.print(trimHint, 2);
    //         Serial.print(" Lcmd="); Serial.print(leftCmd);
    //         Serial.print(" Rcmd="); Serial.println(rightCmd);
    //         lastPidPrintMs = millis();
    //     }
    //     #endif
    
    //     // 10. Apply motors output
    //     motors_.applyMotorOutputs(leftCmd, rightCmd);
    
    //     // 11. Finish Condition
    //     if (fabs(distanceError) <= DONE_DISTANCE_MM && fabs(headingError) <= 2.0f) {
    //         if (stableSinceMs_ == 0) stableSinceMs_ = millis();
    //         if (millis() - stableSinceMs_ >= STABLE_MS) {
    //             stopMotors_();
    //             mode_ = Mode::Idle;
    //             return true;
    //         }
    //     } else {
    //         stableSinceMs_ = 0;
    //     }
    
    //     // 12.
    //     return false;
    // }
        



    // ====================== ROTATE =========================
    if (mode_ == Mode::Rotate) {
    // 2. 
    const float angleError = targetAngleDeg_ - headingDeg_;

    float turnCmd = updatePID_(anglePid_, angleError, dt);
    turnCmd = constrain(turnCmd, -maxPwm_, maxPwm_);

    float leftCmd  = constrain(-turnCmd, -maxPwm_, maxPwm_);
    float rightCmd = constrain( turnCmd, -maxPwm_, maxPwm_);

    const bool needMotion =
        fabsf(angleError) > DONE_ANGLE_DEG ||
        fabsf(gyroRateDps) > DONE_RATE_DPS;

    const float floorPwm = min((float)PWM_MIN_ROTATE, (float)maxPwm_);

    if (needMotion) {
        leftCmd  = applyMinPwm(leftCmd,  floorPwm);
        rightCmd = applyMinPwm(rightCmd, floorPwm);
    } else {
        leftCmd = 0.0f;
        rightCmd = 0.0f;
    }

    // if (fabs(angleError) > DONE_ANGLE_DEG) {
    //     if (fabs(turnCmd) > 0.01f && fabs(turnCmd) < PWM_MIN_ROTATE) {
    //         turnCmd = copysign(PWM_MIN, turnCmd);
    //     }
    // }

    #if DBG_PID
        static unsigned long lastPidPrintMs = 0;
        if (millis() - lastPidPrintMs >= 500) {
            Serial.print("[PID ROT] ");
            Serial.print("dt="); Serial.print(dt, 3);
            Serial.print(" lt="); Serial.print(leftTicks);
            Serial.print(" rt="); Serial.print(rightTicks);
            Serial.print(" dL="); Serial.print(deltaLeft);
            Serial.print(" dR="); Serial.print(deltaRight);
            Serial.print(" head="); Serial.print(headingDeg_, 2);
            Serial.print(" angErr="); Serial.print(angleError, 2);
            Serial.print(" gyro="); Serial.print(gyroRateDps, 2);
            Serial.print(" turnCmd="); Serial.println(turnCmd, 2);
            Serial.print(" needMotion="); Serial.print(needMotion);
            Serial.print(" floorPwm="); Serial.print(floorPwm);
            Serial.print(" Lcmd="); Serial.print(leftCmd);
            Serial.print(" Rcmd="); Serial.print(rightCmd);
            lastPidPrintMs = millis();
        }
    #endif

    motors_.applyMotorOutputs(leftCmd, rightCmd);

    if (fabs(angleError) <= DONE_ANGLE_DEG && fabs(gyroRateDps) <= DONE_RATE_DPS) {
        if (stableSinceMs_ == 0) stableSinceMs_ = millis();
        if (millis() - stableSinceMs_ >= STABLE_MS) {
            stopMotors_();
            mode_ = Mode::Idle;
            return true;
        }
    } else {
        stableSinceMs_ = 0;
    }

    return false;
}

    return true;
}


// ------ dbg prints ------
void PIDController::printLinearDebug(
    float dt,
    long leftTicks,
    long rightTicks,
    long deltaLeft,
    long deltaRight,
    float targetDistanceMm,
    float traveledDistanceMm,
    float distanceError,
    float headingDeg,
    float headingError,
    float linearCmd,
    float angularCmd,
    float leftCmd,
    float rightCmd,
    int maxPwm
    ) {
    Serial.print("[PID LIN] ");
    Serial.print("dt="); Serial.print(dt, 3);
    Serial.print(" lt="); Serial.print(leftTicks);
    Serial.print(" rt="); Serial.print(rightTicks);
    Serial.print(" dL="); Serial.print(deltaLeft);
    Serial.print(" dR="); Serial.print(deltaRight);
    Serial.print(" target="); Serial.print(targetDistanceMm, 2);
    Serial.print(" dist="); Serial.print(traveledDistanceMm, 2);
    Serial.print(" distErr="); Serial.print(distanceError, 2);
    Serial.print(" head="); Serial.print(headingDeg, 2);
    Serial.print(" headErr="); Serial.print(headingError, 2);
    Serial.print(" linCmd="); Serial.print(linearCmd, 2);
    Serial.print(" angCmd="); Serial.print(angularCmd, 2);
    Serial.print(" Lcmd="); Serial.print(leftCmd, 2);
    Serial.print(" Rcmd="); Serial.print(rightCmd, 2);
    Serial.print(" maxPwm="); Serial.println(maxPwm);
}












//----end