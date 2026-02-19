#include <Arduino.h>
#include <Wire.h>

// Hardware
#include "robot.h"
#include "bras.h"

// Debug prints
#include "encoders.h"
#include "ultrasonic.h"
#include "utils.h"
#include "Debug.h"
#include "config.h"

#include "EmergencyButton.h"  // ← Bouton urgence
#include "safety.h"           // ← Safety update
#include "motors.h"            // ← Test moteurs
extern Motors motors;  // ← Import depuis robot.cpp



// ------ helpers ------
// void imAlive()
// {
//   static unsigned long millis_print = 0;
//   if (millis() - millis_print >= 2000)
//   {
//     Serial.println("I'm alive");
//     millis_print = millis();
//   }
// }

// ========= SETUP ===============
void setup()
{
  // Serial.begin(115200);
  debugInit(115200, // does Serial.begin()
            DBG_FSM |
                DBG_MOTORS |
                DBG_SENSORS
            // DBG_COMMS |        // comment DBG_ to deactivate its related prints
            // DBG_ENCODER |
            // DBG_LAUNCH_TGR
  );

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
}

void loop()
{
  static bool runSequence = true;

  // imAlive();
  // printEncodersVal();
  // printUltrasonicVal();

  // if (true) return;  // CETTE LIGNE BLOQUAIT LE CODE
  if (!runSequence)
  {
    return;
  }

  //Servo Test
  bras_deployer();
  delay(2000);
  bras_retracter();
  delay(2000);
  // bras_deployer();
  // delay(2000);
  // bras_retracter();
  // delay(2000);

  //driveDistancePID(-1000, 254);
  driveDistancePID(4000, 254);
  // robot_rotate_gyro(90, 200);


  runSequence = false;
}

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
