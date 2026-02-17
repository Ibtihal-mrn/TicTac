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

  // Servo Test
  // bras_deployer();
  // delay(2000);
  // bras_retracter();
  // delay(2000);
  // bras_deployer();
  // delay(2000);
  // bras_retracter();
  // delay(2000);

  driveDistancePID(-1000, 254);
  driveDistancePID(500, 254);


  runSequence = false;
}