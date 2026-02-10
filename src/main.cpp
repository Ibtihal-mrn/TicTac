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
  // static bool runSequence = true;  

  // if (!runSequence) {
  //   return; 
  // }

  robot_move_distance(1000, 140);

  long l, r ; 
  encoders_read(&l, &r);

  static long lastL = 0, lastR = 0;

  Serial.print("dTicksL="); Serial.print(l - lastL);
  Serial.print(" dTicksR="); Serial.println(r - lastR);
  

  lastL = ticksL;
  lastR = ticksR;

  delay(2000);
  // Serial.print("ticksL="); Serial.print(ticksL);
  // Serial.print(" ticksR="); Serial.println(ticksR);

  // Serial.print(digitalRead(ENC_R_A));
  // Serial.print(" ");
  // Serial.println(digitalRead(ENC_R_B));
  // delay(50);

  // delay(2000);

  // bras_deployer();
  // delay(2000);

  // robot_rotate(-230, 140);
  // delay(2000);
  

  // robot_move_distance(1000, 140);
  // delay(2000);

  // bras_retracter();
  
  


  // bras_deployer();
  // delay(2000);

  //robot_rotate(110, 140);

  // robot_move_distance(1000, 140);

  // bras_retracter();
  // delay(2000);

  // robot_stop();

  // runSequence = false; 
}
