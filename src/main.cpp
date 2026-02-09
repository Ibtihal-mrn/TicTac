#include <Arduino.h>
#include "encoders.h"
#include "motors.h"
#include "control.h"
#include "robot.h"
#include "bras.h"

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

  robot_move_distance(1000, 140);
  Serial.print("ticksL="); Serial.print(ticksL);
  Serial.print(" ticksR="); Serial.println(ticksR);
  delay(2000);

  bras_deployer();
  delay(2000);

  robot_rotate(-230, 140);
  delay(2000);
  

  robot_move_distance(1000, 140);
  delay(2000);

  bras_retracter();
  
  


  // bras_deployer();
  // delay(2000);

  //robot_rotate(110, 140);

  // robot_move_distance(1000, 140);

  // bras_retracter();
  // delay(2000);

  robot_stop();

  runSequence = false; 
}
