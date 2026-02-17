// motors.cpp
#include "motors.h"
#include "../../src/config.h"



// ---------- Constructor ----------
Motors::Motors(uint8_t enaPin, uint8_t in1Pin, uint8_t in2Pin,
                uint8_t enbPin, uint8_t in3Pin, uint8_t in4Pin)
    : enaPin_(enaPin), enbPin_(enbPin),
      in1_(in1Pin), in2_(in2Pin),
      in3_(in3Pin), in4_(in4Pin)
{
    pinMode(enaPin_, OUTPUT);
    pinMode(enbPin_, OUTPUT);

    pinMode(in1_, OUTPUT);
    pinMode(in2_, OUTPUT);
    pinMode(in3_, OUTPUT);
    pinMode(in4_, OUTPUT);

    stopMotors(); // ensure motors are stopped at startup
}

// ---------- Motor Control ----------
void Motors::stopMotors() {
    digitalWrite(in1_, LOW);
    digitalWrite(in2_, LOW);
    digitalWrite(in3_, LOW);
    digitalWrite(in4_, LOW);

    analogWrite(enaPin_, 0);
    analogWrite(enbPin_, 0);
}

void Motors::forward(float leftSpeed, float rightSpeed) {
    applyMotorOutputs(leftSpeed, rightSpeed);
}

void Motors::backward(float leftSpeed, float rightSpeed) {
    applyMotorOutputs(-leftSpeed, -rightSpeed);
}

void Motors::rotateRight(float speed) {
    applyMotorOutputs(-speed, speed);
}

void Motors::rotateLeft(float speed) {
    applyMotorOutputs(speed, -speed);
}

// ---------- Apply PID / direct command ----------
void Motors::applyMotorOutputs(float leftCmd, float rightCmd) {
    setMotorLeftSpeed(leftCmd);
    setMotorRightSpeed(rightCmd);
}

// ---------- Private helpers ----------
void Motors::setMotorLeftSpeed(float speed) {
    int pwm = constrain(abs((int)speed), 0, 255);
    if (speed > 0) {
        digitalWrite(in1_, HIGH);
        digitalWrite(in2_, LOW);
    } else if (speed < 0) {
        digitalWrite(in1_, LOW);
        digitalWrite(in2_, HIGH);
    } else {
        digitalWrite(in1_, LOW);
        digitalWrite(in2_, LOW);
    }
    analogWrite(enaPin_, pwm);
}

void Motors::setMotorRightSpeed(float speed) {
    int pwm = constrain(abs((int)speed), 0, 255);
    if (speed > 0) {
        digitalWrite(in3_, HIGH);
        digitalWrite(in4_, LOW);
    } else if (speed < 0) {
        digitalWrite(in3_, LOW);
        digitalWrite(in4_, HIGH);
    } else {
        digitalWrite(in3_, LOW);
        digitalWrite(in4_, LOW);
    }
    analogWrite(enbPin_, pwm);
}



