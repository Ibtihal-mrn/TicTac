#include <Arduino.h>
#include "encoders.h"
#include "motors.h"
#include "control.h"
#include "robot.h"
#include "bras.h"
#include "kinematics.h"
#include "emergencyButton.h"
#include "ultrasonic.h"  // 3 capteurs NewPing pins 7,10,13 ANTI-ACCOUPS

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
    ultrasonic_init();  // Pins 7,10,13 filtré anti-bruit

    encoders_read(&last_left, &last_right);
}

// --- WRAPPER pour déplacement avec interruption d'urgence ---
bool moveDistanceSafe(float dist_mm, int speed) {
    long target = ticks_for_distance_mm(abs(dist_mm));
    long startL, startR;
    encoders_read(&startL, &startR);
    prevL = startL;
    prevR = startR;
    
    static unsigned long lastUSCheck = 0;  // ANTI-INTERFERENCE

    while (true) {
        long curL, curR;
        encoders_read(&curL, &curR);

        // Check US 100ms seulement (10Hz anti-bruit)
        bool stopObstacle = false;
        if (millis() - lastUSCheck > 100) {
            stopObstacle = ultrasonic_isObstacle(SEUIL_OBSTACLE);
            lastUSCheck = millis();
        }

        // Vérification d'urgence
        if (emergencyButton_isPressed() || stopObstacle) {
            robot_stop();
            emergencyStopActive = emergencyButton_isPressed();
            obstacleStopActive = stopObstacle;
            return false;
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
    
    static unsigned long lastUSCheck = 0;  // ANTI-INTERFERENCE

    while (true) {
        long curL, curR;
        encoders_read(&curL, &curR);

        // Check US 100ms seulement
        bool stopObstacle = false;
        if (millis() - lastUSCheck > 100) {
            stopObstacle = ultrasonic_isObstacle(SEUIL_OBSTACLE);
            lastUSCheck = millis();
        }

        // Vérification d'urgence
        if (emergencyButton_isPressed() || stopObstacle) {
            robot_stop();
            emergencyStopActive = emergencyButton_isPressed();
            obstacleStopActive = stopObstacle;
            return false;
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
    static unsigned long lastUSCheck = 0;  // ANTI-INTERFERENCE

    if (!runSequence) return;

    // Vérification d'urgence (100ms)
    bool stopObstacle = false;
    if (millis() - lastUSCheck > 100) {
        stopObstacle = ultrasonic_isObstacle(SEUIL_OBSTACLE);
        lastUSCheck = millis();
    }
    emergencyStopActive = emergencyButton_isPressed();

    if (emergencyStopActive || stopObstacle) {
        robot_stop();
        while (emergencyButton_isPressed() || ultrasonic_isObstacle(SEUIL_OBSTACLE)) {
            delay(50);
        }
        emergencyStopActive = false;
        obstacleStopActive = false;
        encoders_read(&last_left, &last_right);
        return;
    }

    // Debug (optionnel)
    Serial.print("Dist[0,1,2]: ");
    Serial.print(ultrasonic_readDistance(0)); Serial.print(",");
    Serial.print(ultrasonic_readDistance(1)); Serial.print(",");
    Serial.println(ultrasonic_readDistance(2));

    // Sequence robot intacte...
    if (!moveDistanceSafe(2000, 140)) return;
    Serial.print("ticksL="); Serial.print(ticksL);
    Serial.print(" ticksR="); Serial.println(ticksR);
    delay(2000);

    bras_deployer();
    delay(2000);

    if (!rotateSafe(-230, 140)) return;
    delay(2000);

    if (!moveDistanceSafe(1000, 140)) return;
    delay(2000);

    bras_retracter();
    robot_stop();
    runSequence = false;
}
