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
const int SEUIL_OBSTACLE = 10; // cm

void setup() {
  Serial.begin(9600);
  robot_init();
  bras_init();
  emergencyButton_init();
  ultrasonic_init(11, 12);  // trig, echo

  // Initialiser les dernières positions
  encoders_read(&last_left, &last_right);
}

void loop() {
  static bool runSequence = true;  

  if (!runSequence) {
    return; 
  }

  // --- Vérification d'urgence (bouton et ultrason) ---
  int distance = ultrasonic_readDistance();

  // Bouton d'urgence
  if (emergencyButton_isPressed()) {
      emergencyStopActive = true;
  }

  // Obstacle détecté
  if (distance > 0 && distance <= SEUIL_OBSTACLE) {
      obstacleStopActive = true;
  }

  // Mode arrêt prioritaire : bouton > obstacle
  if (emergencyStopActive || obstacleStopActive) {
      robot_stop();

      // Sortie du mode urgence si bouton relâché
      if (emergencyStopActive && !emergencyButton_isPressed()) {
          emergencyStopActive = false;
          encoders_read(&last_left, &last_right);
      }

      // Sortie du mode obstacle si obstacle disparu
      if (obstacleStopActive && (distance < 0 || distance > SEUIL_OBSTACLE)) {
          obstacleStopActive = false;
          encoders_read(&last_left, &last_right);
      }

      delay(40);
      return;  // quitte loop pour rester arrêté
  }

  // --- Déplacer robot 1 ---
  robot_move_distance(1000, 140);

  // --- Vérification urgence après déplacement ---
  distance = ultrasonic_readDistance();
  if (emergencyButton_isPressed()) emergencyStopActive = true;
  if (distance > 0 && distance <= SEUIL_OBSTACLE) obstacleStopActive = true;
  if (emergencyStopActive || obstacleStopActive) { robot_stop(); return; }

  Serial.print("ticksL="); Serial.print(ticksL);
  Serial.print(" ticksR="); Serial.println(ticksR);
  delay(2000);

  // --- Déployer bras ---
  bras_deployer();

  // --- Vérification urgence après bras ---
  distance = ultrasonic_readDistance();
  if (emergencyButton_isPressed()) emergencyStopActive = true;
  if (distance > 0 && distance <= SEUIL_OBSTACLE) obstacleStopActive = true;
  if (emergencyStopActive || obstacleStopActive) { robot_stop(); return; }

  delay(2000);

  // --- Rotation ---
  robot_rotate(-230, 140);

  // --- Vérification urgence après rotation ---
  distance = ultrasonic_readDistance();
  if (emergencyButton_isPressed()) emergencyStopActive = true;
  if (distance > 0 && distance <= SEUIL_OBSTACLE) obstacleStopActive = true;
  if (emergencyStopActive || obstacleStopActive) { robot_stop(); return; }

  delay(2000);

  // --- Déplacer robot 2 ---
  robot_move_distance(1000, 140);

  // --- Vérification urgence après déplacement ---
  distance = ultrasonic_readDistance();
  if (emergencyButton_isPressed()) emergencyStopActive = true;
  if (distance > 0 && distance <= SEUIL_OBSTACLE) obstacleStopActive = true;
  if (emergencyStopActive || obstacleStopActive) { robot_stop(); return; }

  delay(2000);

  // --- Rétracter bras ---
  bras_retracter();

  // --- Vérification urgence après bras ---
  distance = ultrasonic_readDistance();
  if (emergencyButton_isPressed()) emergencyStopActive = true;
  if (distance > 0 && distance <= SEUIL_OBSTACLE) obstacleStopActive = true;
  if (emergencyStopActive || obstacleStopActive) { robot_stop(); return; }

  // Arrêt final
  robot_stop();
  runSequence = false;
}
