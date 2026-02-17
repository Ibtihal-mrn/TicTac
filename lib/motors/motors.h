// motors.h
#pragma once
#include <Arduino.h>

class Motors {
public:
    // Constructor: pass EN/IN pins
    Motors(uint8_t enaPin, uint8_t in1Pin, uint8_t in2Pin,
            uint8_t enbPin, uint8_t in3Pin, uint8_t in4Pin);

    // Control functions
    void stopMotors();
    void forward(float leftSpeed, float rightSpeed);
    void backward(float leftSpeed, float rightSpeed);
    void rotateRight(float speed);
    void rotateLeft(float speed);

    // Directly apply PID/other outputs
    void applyMotorOutputs(float leftCmd, float rightCmd);

private:
    uint8_t enaPin_, enbPin_;
    uint8_t in1_, in2_, in3_, in4_;

    void setMotorLeftSpeed(float speed);
    void setMotorRightSpeed(float speed);
};
