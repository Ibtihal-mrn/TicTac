#include <Arduino.h>
#include "encoders.h"
#include "motors.h"
#include "control.h"
#include "robot.h"
#include "bras.h"
#include "kinematics.h"
#include "emergencyButton.h"
#include "ultrasonic.h"   // Capteur ultrason

// Dernières positions des encodeurs
static long last_left = 0;
static long last_right = 0;

// Latch d'arrêt d'urgence
static bool emergencyStopActive = false;

// Latch arrêt obstacle
static bool obstacleStopActive = false;
const int SEUIL_OBSTACLE = 40; // cm

void setup() {
    Serial.begin(9600);
    robot_init();
    bras_init();
    emergencyButton_init();
    ultrasonic_init(13, 10);  // trig, echo

    encoders_read(&last_left, &last_right);
}

// --- WRAPPER pour déplacement avec interruption d'urgence ---
bool moveDistanceSafe(float dist_mm, int speed) {
    long target = ticks_for_distance_mm(abs(dist_mm));
    long startL, startR;
    encoders_read(&startL, &startR);
    prevL = startL;
    prevR = startR;

    while (true) {
        long curL, curR;
        encoders_read(&curL, &curR);

        // Lire la distance une seule fois
        int distance = ultrasonic_readDistance();
        bool stopObstacle = (distance > 0 && distance <= SEUIL_OBSTACLE);

        // Vérification d'urgence
        if (emergencyButton_isPressed() || stopObstacle) {
            robot_stop();
            emergencyStopActive = emergencyButton_isPressed();
            obstacleStopActive = stopObstacle;
            return false;  // déplacement interrompu
        }

        long distTicksL = labs(curL - startL);
        long distTicksR = labs(curR - startR);
        if ((distTicksL + distTicksR) / 2 >= target) break;

        long dL, dR;
        encoders_computeDelta(curL, curR, &dL, &dR);

        int speedL, speedR;
        control_computeSpeeds(dL, dR, speedL, speedR);

        motors_applySpeeds(speedL, speedR);
        delay(40);
    }
    motors_stop();
    return true;
}

// --- WRAPPER pour rotation avec interruption d'urgence ---
bool rotateSafe(float angle_deg, int speed) {
    long targetTicks = ticks_for_rotation_deg(angle_deg);
    long startL, startR;
    encoders_read(&startL, &startR);

    if (angle_deg > 0) motors_rotateRight(speed);
    else motors_rotateLeft(speed);

    while (true) {
        long curL, curR;
        encoders_read(&curL, &curR);

        // Lire la distance une seule fois
        int distance = ultrasonic_readDistance();
        bool stopObstacle = (distance > 0 && distance <= SEUIL_OBSTACLE);

        // Vérification d'urgence
        if (emergencyButton_isPressed() || stopObstacle) {
            robot_stop();
            emergencyStopActive = emergencyButton_isPressed();
            obstacleStopActive = stopObstacle;
            return false; // rotation interrompue
        }

        long dL = labs(curL - startL);
        long dR = labs(curR - startR);
        if ((dL + dR) / 2 >= labs(targetTicks)) break;
        delay(40);
    }
    motors_stop();
    return true;
}

void loop() {
    static bool runSequence = true;  

    if (!runSequence) return;

    // Vérification d'urgence avant toute action
    int distance = ultrasonic_readDistance();
    emergencyStopActive = emergencyButton_isPressed();
    obstacleStopActive = (distance > 0 && distance <= SEUIL_OBSTACLE);

    if (emergencyStopActive || obstacleStopActive) {
        robot_stop();
        // Rester arrêté tant que bouton ou obstacle présent
        while (emergencyButton_isPressed() || (ultrasonic_readDistance() > 0 && ultrasonic_readDistance() <= SEUIL_OBSTACLE)) {
            delay(50);
        }
        // Réinitialisation
        emergencyStopActive = false;
        obstacleStopActive = false;
        encoders_read(&last_left, &last_right);
        return;
    }

    // --- Déplacer robot 1 ---
    if (!moveDistanceSafe(2000, 140)) return;

    Serial.print("ticksL="); Serial.print(ticksL);
    Serial.print(" ticksR="); Serial.println(ticksR);
    delay(2000);

    // --- Déployer bras ---
    bras_deployer();
    delay(2000);

    // --- Rotation ---
    if (!rotateSafe(-230, 140)) return;
    delay(2000);

    // --- Déplacer robot 2 ---
    if (!moveDistanceSafe(1000, 140)) return;
    delay(2000);

    // --- Rétracter bras ---
    bras_retracter();

    // Arrêt final
    robot_stop();
    runSequence = false;
}
