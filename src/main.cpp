#include <Arduino.h>
#include <Wire.h>

// Hardware
#include "robot.h"
#include "bras.h"
#include "Relais.h"

// Debug prints
#include "encoders.h"
#include "ultrasonic.h"
#include "utils.h"
#include "Debug.h"
#include "config.h"

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
const int RELAY_PIN = 41;
const bool RELAY_ACTIVE_LOW = true;

const int SWITCH_PIN = 2;

bool lastSwitchState = HIGH;   // INPUT_PULLUP
bool sequenceDone = false;     // Pour éviter répétition

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

  pinMode(SWITCH_PIN, INPUT_PULLUP);
  lastSwitchState = digitalRead(SWITCH_PIN);
  relais_init(RELAY_PIN, RELAY_ACTIVE_LOW);

  // Relais ON au démarrage
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
}

void loop()
{
  static bool runSequence = true;

  // imAlive();
  // printEncodersVal();
  // printUltrasonicVal();

  // if (true) return;  // CETTE LIGNE BLOQUAIT LE CODE
  //if (!runSequence)
  //{
    //return;
  //}
  if (sequenceDone) return;

  bool currentSwitchState = digitalRead(SWITCH_PIN);

  // Détection appui (HIGH → LOW)
  if (lastSwitchState == HIGH && currentSwitchState == LOW)
  {
      Serial.println("Switch activé !");

      // ➜ Avancer 10 cm
      driveDistancePID(100, 200);

      //  Relais OFF
      relais_off();

      // ➜ Avancer encore 10 cm
      driveDistancePID(100, 200);

      sequenceDone = true; // éviter répétition
  }

  lastSwitchState = currentSwitchState;
}

  // Servo Test
  // bras_deployer();
  // delay(2000);
  // bras_retracter();
  // delay(2000);
  // bras_deployer();
  // delay(2000);
  // bras_retracter();
  // delay(2000);

  //driveDistancePID(-1000, 254);
  //driveDistancePID(500, 254);


  //runSequence = false;
