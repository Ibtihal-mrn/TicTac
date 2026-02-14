#include "motors.h"




// ---------- Constructor -----------
Motors::Motors(uint8_t enaPin, uint8_t in1Pin, uint8_t in2Pin,uint8_t enbPin, uint8_t in3Pin, uint8_t in4Pin)
    : motors(enaPin, in1Pin, in2Pin, enbPin, in3Pin, in4Pin) // pins
      // pidDistance(DISTANCE_PID_DEFAULT.kp, DISTANCE_PID_DEFAULT.ki, DISTANCE_PID_DEFAULT.kd), // default values from config.h
      // pidAngle(ANGLE_PID_DEFAULT.kp, ANGLE_PID_DEFAULT.ki, ANGLE_PID_DEFAULT.kd)
{
    // target.active = false;
    lastUpdateUs = micros();
}

// ----------------------------
void Motors::stopMotors() { motors.stop(); }

void Motors::startForward(float distanceCm) {
    // target.distanceCm = distanceCm;
    // target.angleDeg = 0.0f;
    // target.active = true;

    // startDistance = 0;
    // startYaw = 0;

    // resetPID(pidDistance);
    // resetPID(pidAngle);
}

void Motors::startRotate(float angleDeg) {
    // target.distanceCm = 0.0f;
    // target.angleDeg = angleDeg;
    // target.active = true;

    // startDistance = 0;
    // startYaw = 0;

    // resetPID(pidDistance);
    // resetPID(pidAngle);
}

void Motors::forward(int speed) {
    applyMotorOutputs(speed, speed);
}

void Motors::backward(int speed) {
    applyMotorOutputs(-speed, -speed);
}

void Motors::rotateRight(int speed) {
    applyMotorOutputs(-speed, speed);
} 

void Motors::rotateLeft(int speed) {
    applyMotorOutputs(speed, -speed);
}

// private:
void Motors::applyMotorOutputs(float leftCmd, float rightCmd)
{
    int leftPWM  = constrain(abs((int)leftCmd),  0, 255);
    int rightPWM = constrain(abs((int)rightCmd), 0, 255);

    motors.setSpeedA(leftPWM);
    motors.setSpeedB(rightPWM);

    if (leftCmd > 0)
        motors.forwardA();
    else if (leftCmd < 0)
        motors.backwardA();
    else
        motors.stopA();

    if (rightCmd > 0)
        motors.forwardB();
    else if (rightCmd < 0)
        motors.backwardB();
    else
        motors.stopB();
    
    // ---- Debug prints ----
    // if (leftPWM == 255 || rightPWM == 255) { debugPrintF(DBG_MOVEMENT, F("PWM SATURATION")); }
}






// =====================================================


// Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Adafruit_DCMotor *motorL = NULL;
// Adafruit_DCMotor *motorR = NULL;

// inverser si nécessaire
// bool invertLeft = false;
// bool invertRight = false;

// ---------- PARAMETRES ----------
// int baseSpeed = 140; // vitesse de base
// float Kp = 0.5;      // gain correction trajectoire

// int trimL = 0;
// int trimR = -4;

// void motors_init(void) {
  // Wire.begin();
  // AFMS.begin();

  // motorL = AFMS.getMotor(2); // moteur gauche
  // motorR = AFMS.getMotor(1); // moteur droit

  // moteurs initialisés à l'arrêt
  // motorL->setSpeed(0);
  // motorL->run(FORWARD);

  // motorR->setSpeed(0);
  // motorR->run(FORWARD);
  // motors_stop();
// }

// void motors_applySpeeds(int speedL, int speedR) { // On va normalement plus l'utiliser
  // motorL->run(FORWARD);
  // motorR->run(FORWARD);

  // motorL->setSpeed(speedL);
  // motorR->setSpeed(speedR);
// }

// void motors_stop() {
  // motorL->setSpeed(0);
  // motorR->setSpeed(0);
  // motorL->run(RELEASE); // Release permet de couper le pont H, arrêt propre
  // motorR->run(RELEASE);
// }

// void motors_forward(int speedL, int speedR) {
  // speedL = constrain(speedL, 0, 255);
  // speedR = constrain(speedR, 0, 255);

  // motorL->setSpeed(speedL);
  // motorR->setSpeed(speedR);

  // motorL->run(FORWARD);
  // motorR->run(FORWARD);
// }

// void motors_rotateRight(int speed) {
  // speed = constrain(speed, 0, 255);
  // motorL->setSpeed(speed);
  // motorR->setSpeed(speed);
  // motorL->run(BACKWARD);
  // motorR->run(FORWARD);
// }

// void motors_rotateLeft(int speed) {
  // speed = constrain(speed, 0, 255);
  // motorL->setSpeed(speed);
  // motorR->setSpeed(speed);
  // motorL->run(FORWARD);
  // motorR->run(BACKWARD);
// }


// void motors_backward(int speedL, int speedR) {
  // speedL = constrain(speedL, 0, 255);
  // speedR = constrain(speedR, 0, 255);

  // motorL->setSpeed(speedL);
  // motorR->setSpeed(speedR);

  // motorL->run(BACKWARD);
  // motorR->run(BACKWARD);
// }
