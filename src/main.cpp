// main.cpp
#include <Arduino.h>
#include <Wire.h>

// Hardware
// #include "robot.h" //! replaced by fsm.h
#include "fsm.h"
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

Context fsmCtx{};

// OBSTACLE : Hardware Interrupt (Ultrasonic sensors Hub Obstacle detected).
volatile bool emergencyStop = false;  //extern in globals.h
void IRAM_ATTR stopISR() { emergencyStop = digitalRead(STOP_PIN); }


// TODO: remove/obfuscate this :
// const int RELAY_PIN = 41;
// const bool RELAY_ACTIVE_LOW = true;
// bool lastSwitchState = HIGH;   // INPUT_PULLUP
// bool sequenceDone = false;     // Pour éviter répétition



// ================== SETUP ==================
void setup()
{
  debugInit(115200,
            DBG_FSM |
            DBG_I2C_HUB |
            DBG_PID |
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
  // bras_init();                // must run FIRST
  // robot_init();

  hardware_init(fsmCtx); //! called in fsm

  Serial.println("Setup Done.");

  // startSwitch.begin();
  // teamSwitch.begin();

  // Serial.println("Setup Done."); 
  // Serial.println("Waiting for start switch...");
  // startSwitch.waitForStart();
  // Serial.println("Starting sequence..."); 

  
}

void loop()
{

  // motors.forward(150, 150);
  // delay(2000);
  // motors.stopMotors();
  // delay(5000);
  // return;
  robot_step(fsmCtx);

}
