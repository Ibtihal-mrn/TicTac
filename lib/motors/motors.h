#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>
#include <Wire.h>
#include <L298NX2.h>  // Motor Driver

class Motors {
public:
    Motors(uint8_t ena, uint8_t in1, uint8_t in2,
            uint8_t enb, uint8_t in3, uint8_t in4);

    void startForward(float distanceCm);
    void startRotate(float angleDeg);

    void forward(int speed);
    void backward(int speed);
    void rotateRight(int speed);
    
    

    // void update(const SensorsData &sensors);
    bool isDone() const;
    void stopMotors();
    // void setDistancePID(float kp, float ki, float kd);
    // void setAnglePID(float kp, float ki, float kd);

private:
    L298NX2 motors; // ONLY pass IN pins, not EN (library bug)
    uint8_t ENA;
    uint8_t ENB;
    // PID pidDistance;
    // PID pidAngle;

    // MovementTarget target;

    unsigned long lastUpdateUs;
    float startDistance, startYaw;

    // void resetPID(PID &pid);
    // float computePID(PID &pid, float error, float dt, 
    //         float &pTerm, float &iTerm, float &dTerm);

    void applyMotorOutputs(float leftCmd, float rightCmd);
    
};




// #include <Adafruit_MotorShield.h>

// ---------- MOTEURS ADATRUFS ----------
// extern Adafruit_MotorShield AFMS;
// extern Adafruit_DCMotor *motorL;
// extern Adafruit_DCMotor *motorR;

// // inverser si nécessaire
// extern bool invertLeft;
// extern bool invertRight;

// // ---------- PARAMETRES ----------
// extern int baseSpeed;  // vitesse de base
// extern float Kp;       // gain correction trajectoire

// extern int trimL;
// extern int trimR;

// void motors_init(void);
// void motors_applySpeeds(int speedL, int speedR);
// void motors_forward(int speedL, int speedR);
// void motors_backward(int speedL, int speedR);
// void motors_stop();
// void motors_rotateRight(int speed);
// void motors_rotateLeft(int speed);

#endif