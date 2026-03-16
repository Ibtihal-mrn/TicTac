#include <Arduino.h>
#include <Wire.h>

// Hardware
#include "robot.h"
#include "bras.h"
#include "Relais.h"

// Debug prints
#include "encoders.h"
#include "us.h"         // ultrasonic
#include "utils.h"
#include "Debug.h"
#include "config.h"
#include "StartSwitch.h"
#include "TeamSwitch.h"

#include "EmergencyButton.h" // ← Bouton urgence
#include "safety.h"          // ← Safety update
#include "motors.h"          // ← Test moteurs
extern Motors motors;        // ← Import depuis robot.cpp

// ------ helpers ------
void imAlive()
{
  static unsigned long millis_print = 0;
  if (millis() - millis_print >= 2000)
  {
    Serial.println("I'm alive");
    millis_print = millis();
  }
}

StartSwitch startSwitch(GPIO_NUM_2);
TeamSwitch teamSwitch((gpio_num_t)TEAM_SWITCH_PIN);

// ========= SETUP ===============
// const int RELAY_PIN = 41;
const bool RELAY_ACTIVE_LOW = true;



bool lastSwitchState = HIGH;   // INPUT_PULLUP
bool sequenceDone = false;     // Pour éviter répétition

void setup()
{
  // Serial.begin(115200);
  debugInit(115200, // does Serial.begin()
            DBG_FSM |
                DBG_MOTORS |
                DBG_SENSORS |
                DEBUG_TEAM_SWITCH |
                DBG_SERVO |
                DBG_ENCODER |
                DBG_MAGNET
            // DBG_COMMS |        // comment DBG_ to deactivate its related prints
            // DBG_ENCODER |
            // DBG_LAUNCH_TGR
  );

  pinMode(SWITCH_PIN, INPUT_PULLUP);
  lastSwitchState = digitalRead(SWITCH_PIN);
  relais_init(RELAY_PIN, RELAY_ACTIVE_LOW);

  // Init electro-aimant
  const bool RELAY_ACTIVE_LOW = true;
  bool lastSwitchState = HIGH;   // INPUT_PULLUP
  bool sequenceDone = false;     // Pour éviter répétition
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  lastSwitchState = digitalRead(SWITCH_PIN);
  relais_init(RELAY_PIN, RELAY_ACTIVE_LOW);
  relais_on();

  // // I2C Init.
  Wire.begin(6, 7); // SDA, SCL
  Wire.setClock(100000);
  delay(200);

  // Utils.h
  printEsp32Info();
  // i2c_scanner();

  // Init Hardware et Robot
  ESP32PWM::allocateTimer(0); // SERVO timer (doit rester ici)
  ESP32PWM::allocateTimer(1); // SERVO timer (doit rester ici)
  bras_init();                // must run FIRST
  robot_init();

  Serial.println("Setup Done.");

  // startSwitch.begin();
  teamSwitch.begin();

  Serial.println("Waiting for start switch...");
  // startSwitch.waitForStart();
  Serial.println("Starting sequence..."); 
}

void loop()
{

  static bool runSequence = true;  
  ultrasonic_update();


  if (!runSequence) {
    return; 
  }

  if (teamSwitch.readTeam() == TeamSwitchTeam::A)
  {
    Serial.println("Equipe A");
    
    // driveDistancePID(1000, 200);   // avance 15 cm
    // delay(1000);                   // arrêt du robot

    // bras_deployer();              // déploie le bras
    // delay(1000);                   // arrêt du robot

    // rotateAnglePID(180, 200);      // tourne à droite de 90°
    // delay(1000);                   // arrêt du robot

    // driveDistancePID(1000, 200);    // avance 100 cm
    // delay(1000);                   // arrêt du robot

    // driveDistancePID(-500, 200);    // recule 50 cm
    // delay(1000);                   // arrêt du robot

    // rotateAnglePID(-90, 200);      // tourne à gauche de 90°
    // delay(1000);                   // arrêt du robot 

    driveDistancePID(500, 254);     // avance 50 cm
    delay(1000);                   // arrêt du robot
    bras_deployer();              // déploie le bras
    delay(1000);                   // arrêt du robot
    bras_retracter();             // rétracte le bras
    delay(1000);                   // arrêt du robot
     bras_deployer();              // déploie le bras
    delay(1000);                   // arrêt du robot
    bras_retracter();             // rétracte le bras
    delay(1000);  
     bras_deployer();              // déploie le bras
    delay(1000);                   // arrêt du robot
    bras_retracter();             // rétracte le bras
    delay(1000);  
  }

  else
  {
    // Serial.println("Equipe B");
    // driveDistancePID(1000, 254);
    // delay(1000);
    // // bras_deployer();
    // // delay(2000);
    // // robot_rotate_gyro(180, 200);
    // // delay(1000);
    // // driveDistancePID(500, 254);
    // // delay(1000);
    // // bras_retracter();
    driveDistancePID(500, 200);   // avance
    delay(500);                    // arrêt

    relais_on();                   // active le relais
    delay(500);                   // attendre 5 secondes

    driveDistancePID(500, 200);   // avance
    delay(500);                   // ← ce delay ne sert pas d'arrêt, c'est trop long

    relais_off();                  // désactive le relais
    delay(500);

    driveDistancePID(500, 200);   // avance
    }


  runSequence = false;
}

  //Servo Test
  // bras_deployer();
  // delay(2000);
  // bras_retracter();
  // delay(2000);

  //driveDistancePID(-1000, 254);
  //driveDistancePID(500, 254);

  // imAlive();
  // printEncodersVal();
  // printUltrasonicVal();
  // imAlive();
  // printEncodersVal();
  // printUltrasonicVal();

  // Servo Test
  //  bras_deployer();
  //  delay(2000);
  //  bras_retracter();
  //  delay(2000);
  //  bras_deployer();
  //  delay(2000);
  //  bras_retracter();
  //  delay(2000);

  // driveDistancePID(500, 254);
  // delay(1000);
  // driveDistancePID(-500, 254);
  // delay(1000);

  // rotateAnglePID(90, 200);




// void loop() {
//     printUltrasonicVal();  // Décommente
//     Serial.print("Obstacle? ");
//     Serial.println(ultrasonic_isObstacle() ? "OUI" : "NON");
//     delay(500);
// }

// void loop() {
//     Serial.println("=== DEBUG SAFETY ===");
//     Serial.print("emergencyButton: "); Serial.println(emergencyButton_isPressed() ? "OUI" : "NON");
//     Serial.print("ultrasonic obs: "); Serial.println(ultrasonic_isObstacle() ? "OUI" : "NON");
//     Serial.print("safety_update: "); Serial.println(safety_update() ? "STOP" : "OK");
//     delay(200);
// }

// void loop() {
//     Serial.println("=== DEBUG DRIVE ===");
//     Serial.print("safety_update: "); Serial.println(safety_update() ? "STOP" : "OK");
//     if (safety_update()) Serial.println("*** WOULD STOP ***");
//     delay(200);
// }

// void loop() {
//     Serial.println("START MOTORS");
//     motors.forward(150, 150);  // ← Démarre
//     delay(3000);

//     Serial.println("STOP TEST");
//     motors.stopMotors();       // ← Test
//     delay(3000);

//     Serial.println("---");
// }
  //runSequence = false;