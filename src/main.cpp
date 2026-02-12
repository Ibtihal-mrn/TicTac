#include <Arduino.h>
#include "encoders.h"
#include "motors.h"
#include "control.h"
#include "robot.h"
#include "bras.h"
#include "imu.h"

void setup()
{
  Serial.begin(9600);
  robot_init();
  bras_init();
}

void loop()
{
  static bool runSequence = true;  

  if (!runSequence) {
    return; 
  }

  // -----------------------------------
  // --------- Carré sans gyro ---------
  // -----------------------------------
  // delay(2000);
  // robot_rotate(120, 140);
  // delay(2000);
  // robot_move_distance(1000, 140);
  // delay(2000);
  // robot_rotate(120, 140);
  // delay(2000);
  // robot_move_distance(1000, 140);
  // delay(2000);
  // robot_rotate(120, 140);
  // delay(2000);
  // robot_move_distance(1000, 140);
  // delay(2000);
  // robot_rotate(120, 140);
  // robot_stop();
  

  // -----------------------------------
  // --------- Carré avec gyro ---------
  // -----------------------------------

  // robot_move_distance(1255, 140);
  // delay(2000);
  // robot_rotate_gyro(90, 150);
  // delay(2000);

  // robot_move_distance(1255, 140);
  // delay(2000);
  // robot_rotate_gyro(90, 150);
  // delay(2000);

  // robot_move_distance(1255, 140);
  // delay(2000);
  // robot_rotate_gyro(90, 150);
  // delay(2000);

  // robot_move_distance(1255, 140);
  // delay(2000);
  // robot_rotate_gyro(90, 150);
  // delay(2000);

  // robot_stop();



  // -----------------------------------
  // --------- Séquence de test ---------
  // -----------------------------------

  // robot_move_distance(1255, 140);
  // delay(2000);

  // bras_deployer();
  // delay(2000);

  // robot_rotate_gyro(180, 160);
  // delay(2000);

  // robot_move_distance(1250, 140);
  // delay(2000);

  // bras_retracter();
  // delay(2000);

  // robot_stop();

  


 



  runSequence = false; 
} 


// Print 
// Serial.print("ticksL="); Serial.print(ticksL);
// Serial.print(" ticksR="); Serial.println(ticksR);

// Serial.print(digitalRead(ENC_R_A));
// Serial.print(" ");
// Serial.println(digitalRead(ENC_R_B));
// delay(50);


// long l, r ; 
// encoders_read(&l, &r);

// static long lastL = 0, lastR = 0;

// Serial.print("dTicksL="); Serial.print(l - lastL);
// Serial.print(" dTicksR="); Serial.println(r - lastR);


// lastL = ticksL;
// lastR = ticksR;
