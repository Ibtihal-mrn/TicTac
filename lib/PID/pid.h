#pragma once
#include <Arduino.h>
#include "../motors/motors.h"
#include "../encoders/encoders.h"
#include "../MPU6050/imu.h"

struct PID {
    float kp, ki, kd;
    float integral, previousError;

    PID(float _kp = 0.0f, float _ki = 0.0f, float _kd = 0.0f)
        : kp(_kp), ki(_ki), kd(_kd), integral(0.0f), previousError(0.0f) {}
};

extern const PID DISTANCE_PID_DEFAULT;
extern const PID ANGLE_PID_DEFAULT;

class PIDController {
public:
    enum class Mode {
        Idle,
        Linear,
        Rotate
    };

    explicit PIDController(Motors& motors);

    void setDistancePID(const PID& pid);
    void setAnglePID(const PID& pid);

    void startLinear(float distanceMm, int maxPwm);
    void startRotate(float angleDeg, int maxPwm);

    void abort();
    bool update();        // true = mouvement terminé
    bool isBusy() const;
    Mode mode() const;

    static void printLinearDebug( float dt,
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
        int maxPwm);

private:
    Motors& motors_;
    PID distancePid_;
    PID anglePid_;

    Mode mode_;
    float targetDistanceMm_;
    float traveledDistanceMm_;
    float targetAngleDeg_;
    float headingDeg_;
    int maxPwm_;
    unsigned long lastUpdateUs_;
    unsigned long stableSinceMs_;

    // PID min/max values
    static constexpr int PWM_MIN_LINEAR = 40;
    static constexpr int PWM_MIN_ROTATE = 80;
    static constexpr int PWM_MAX = 255;


    static constexpr float DONE_DISTANCE_MM = 10.0f;    // linear precision
    static constexpr float DONE_ANGLE_DEG = 5.0f;       // angle  precision
    static constexpr float DONE_RATE_DPS = 8.0f;
    static constexpr float INTEGRAL_CLAMP = 300.0f;
    static constexpr unsigned long STABLE_MS = 120;

    void resetState();
    void stopMotors_();
    float nextDt_();
    static float updatePID_(PID& pid, float error, float dt);
};