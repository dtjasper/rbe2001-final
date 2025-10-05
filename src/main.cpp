#include <Arduino.h>
#include "robot.h"
#include "BlueMotor.h"

Robot robot;

BlueMotor motorA;
BlueMotor motorB;



void setup() 
{
  Serial.begin(115200);

/**
 * If you define __SETUP_DEBUG__ in your .ini file, this line will make the program wait
 * until the Serial is set up so you can debug. WARNING: If you do that, you _must_ open 
 * the Serial Monitor for anything to happen!
 */
#ifdef __SETUP_DEBUG__
  while(!Serial){delay(5);}
#endif

  robot.InitializeRobot();
}

void loop() 
{
  // robot.RobotLoop();
  
    motorA.setEffortA(100);
    
    motorB.setEffortB(100);

}
