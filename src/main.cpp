// main.cpp
#include <Arduino.h>
#include <Wire.h>

// Hardware
#include "robot.h"
#include "bras.h"
#include "Relais.h"
#include "encoders.h"
#include "StartSwitch.h"
#include "TeamSwitch.h"
#include "motors.h"

// Hub sensors Coprocessor
#include "i2c_comm.h"
#include "us.h"

// Config & Debug prints
#include "utils.h"
#include "Debug.h"
#include "config.h"
#include "globals.h"

// External Objects
extern Motors motors;
StartSwitch startSwitch(GPIO_NUM_38);
TeamSwitch teamSwitch((gpio_num_t)TEAM_SWITCH_PIN);

// OBSTACLE : Hardware Interrupt (Ultrasonic sensors Hub Obstacle detected).
volatile bool emergencyStop = false;  //extern in globals.h
void IRAM_ATTR stopISR() { emergencyStop = digitalRead(STOP_PIN); }


// TODO: remove/obfuscate this :
// const int RELAY_PIN = 41;
const bool RELAY_ACTIVE_LOW = true;
bool lastSwitchState = HIGH;   // INPUT_PULLUP
bool sequenceDone = false;     // Pour éviter répétition



// ================== SETUP ==================
void setup()
{
  debugInit(115200,
            DBG_FSM |
            DBG_I2C_HUB |
            // DBG_MOTORS |
            // DBG_SENSORS |
            // DEBUG_TEAM_SWITCH |
            // DBG_SERVO |
            // DBG_ENCODER |
            // DBG_MAGNET
            // DBG_COMMS |  
            DBG_ENCODER 
            // DBG_LAUNCH_TGR       
            // comment to deactivate its related prints
  );

  // pinMode(SWITCH_PIN, INPUT_PULLUP);
  // lastSwitchState = digitalRead(SWITCH_PIN);
  // relais_init(RELAY_PIN, RELAY_ACTIVE_LOW);

  // Init electro-aimant
  const bool RELAY_ACTIVE_LOW = true;  
  bool lastSwitchState = HIGH;   // INPUT_PULLUP
  bool sequenceDone = false;     // Pour éviter répétition
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  lastSwitchState = digitalRead(SWITCH_PIN);
  relais_init(RELAY_PIN, RELAY_ACTIVE_LOW);
  // relais_on();

  // I2C Setup.
  Wire.begin(SDA_PIN, SCL_PIN, 100000);
  // initUSConfig();    //TODO: change init config of hub
  delay(200);
  

  // Utils.h
  i2c_scanner();
  // printEsp32Info();


  // ======== HARDWARE INIT ==========
  pinMode(STOP_PIN, INPUT); //
  attachInterrupt(digitalPinToInterrupt(STOP_PIN), stopISR, CHANGE);

  // Init Hardware et Robot
  ESP32PWM::allocateTimer(0); // SERVO timer (doit rester ici)
  ESP32PWM::allocateTimer(1); // SERVO timer (doit rester ici)
  bras_init();                // must run FIRST
  robot_init();

  Serial.println("Setup Done.");

  startSwitch.begin();
  teamSwitch.begin();

  Serial.println("Setup Done."); 
  Serial.println("Waiting for start switch...");
  startSwitch.waitForStart();
  Serial.println("Starting sequence..."); 
}

void loop()
{
  static bool runSequence = true;  

  if (!runSequence) return;

  // driveDistancePID(1000, 200);

  relais_on();

  return;

  if (teamSwitch.readTeam() == TeamSwitchTeam::A)
  {
    Serial.println("Equipe A");
    
    driveForward(900, 150);
        // rotateAnglePID(-180, 200);      // tourne à droite de 90°
    // driveBackward(200, 50);

    delay(500);

    rotate(-80, 10);
    // rotate(-100, 10);
    // rotate(-45, 150);
    delay(200);

    driveForward(800, 150);

    delay(1000);                   // arrêt du robot
    
    bras_deployer();              // déploie le bras
    delay(500);                   // arrêt du robot
    bras_retracter();             // rétracte le bras
    delay(500);  
  }

  else
  {
    Serial.println("Equipe B");

    driveForward(100, 50);
        // rotateAnglePID(-180, 200);      // tourne à droite de 90°
    // driveBackward(200, 50);
    rotate(200, 100);

    delay(4000);
    rotate(80, 10);
    delay(3000);
    rotate(-100, 10);
    delay(3000);
    rotate(120, 10);
    delay(3000);
    rotate(-140, 10);
    // rotate(-45, 150);

    // driveForward(1000, 255);

    // driveDistancePID(500, 200);   // avance
    // rotate(180, 10);      // tourne à droite de 90°
    // delay(500);                    // arrêt

    // rotate(-180, 10);
    // relais_on();                   // active le relais
    // delay(500);                   // attendre 5 secondes

    // driveDistancePID(500, 200);   // avance
    // delay(500);                   // ← ce delay ne sert pas d'arrêt, c'est trop long

    // relais_off();                  // désactive le relais
    // delay(500);

    }


  runSequence = false;
}
