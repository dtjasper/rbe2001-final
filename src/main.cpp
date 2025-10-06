#include <Arduino.h>
#include "robot.h"
#include "BlueMotor.h"

Robot robot;

BlueMotor lowerLinkDriveMotor; //formerly motor A
BlueMotor upperLinkDriveMotor; //formerly motor B

//Time romi spends per robot loop. UPDATE 

//Both below values NEED to be updated with actual arm starting positions. UPDATE
double lowerLinkStartAngle = 0;//angle between the first virtual four bar and the ground. Four bar has a hard stop when leaning in towards ROMI's center, this should be around that
double upperLinkStartAngle = 0;//angle between the two "interior" links on both virtual four bars. In resting, try keeping this close to 0 as possibl

//Robot starts in rest, so both values should start at their start values
double lowerLinkAngle = lowerLinkStartAngle; 
double upperLinkAngle = upperLinkStartAngle; 




//Values for converting encoder count to degrees. UPDATE
double lowerLinkEncoderToDegreeConst = 0;
double upperLinkEncoderToDegreeConst = 0;
//KP Values for driving both linkages. Zero currently for obvious reasons
double lowerKP=0;
double upperKP=0; 

double tolerance = 5;


//servo positions designating open and closed claws. UPDATE
int clawClosePos = 0;
int clawOpenPos = 0;

//Arm movement poses. First angle is the lower VBar target, second value is the upper VBar target. 
//Third number is either 0, 1. If 0, this is a place w/o need for claw movement. 1 means this point sees open claw, 2 means this point sees close claw is needed
//List of poses is NOT comprehensive
//UPDATE
Pose armPose1 {300, 300, 0};
Pose armPose2 {300, 300, 0};
Pose armPose3 {0, 0, 0};
double lowerLinkTarget = armPose1.x;
double upperLinkTarget = armPose1.y;
bool needOscillate = armPose1.theta!=0; 
int currentTableInd = 0;


Pose armPoseTable[] = {armPose1,armPose2,armPose3};


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
  lowerLinkDriveMotor.setup();
}

void loop() // motors will only spin after "pio device monitor" goes in terminal rn
{
  // robot.RobotLoop();

  // both cannot spin at once
    lowerLinkDriveMotor.setEffortA(100);
    //motorB.setEffortB(100);


    //TODO: indicate which motors drive which virtual four bars

}
void linkageloop(){
  //Linkages always trying to move to targets
  //TODO: implement pose updating for both linkages

  //To ensure two motors arent active at once, this code exists to oscillate between which motors are active every loop
  //In the event running dual motors becomes possible, code to replace IF block is below
  // lowerLinkDriveMotor.setEffortA(PIDCalcArm(lowerLinkAngle,lowerLinkTarget,lowerKP));
  // updateLowerLinkagePos();
  // upperLinkDriveMotor.setEffortB(PIDCalcArm(lowerLinkAngle,lowerLinkTarget,lowerKP));
  // updateUpperLinkagePos();



  if(lowerLinkDriveMotor.getMotorAEffort() != 0){//if lower link trying to move
    lowerLinkDriveMotor.setEffortB(0);//kill it
    updateLowerLinkagePos();//update its pose with any movement cfhanges recorded by the encoder
    upperLinkDriveMotor.setEffortB(PIDCalcArm(lowerLinkAngle,lowerLinkTarget,lowerKP)); //and run the upper linkage in accordance with whatever angular target it has.
  }else{//if above doesnt run, that means lower link is NOT moving, as such upperlink must be moving
    upperLinkDriveMotor.setEffortB(0);
    updateUpperLinkagePos();
    lowerLinkDriveMotor.setEffortA(PIDCalcArm(upperLinkAngle,upperLinkTarget,upperKP));
  }
   
    
    
    //if both link angles are within their targets
    if(valWithinTolerance(lowerLinkAngle,lowerLinkTarget,tolerance) && valWithinTolerance(upperLinkAngle,upperLinkTarget,tolerance)){
      if(armPoseTable[currentTableInd].theta == 1){//a theta of 1 signals an open claw
        openClaw();
      }else if(armPoseTable[currentTableInd].theta == 2){// a theta of 2 signals a close claw
        closeClaw();
      }
      //move index to point to next pose
      currentTableInd++;
      lowerLinkTarget = armPoseTable[currentTableInd].x;//update lower link targ with new pose x, which represents the lower VBar target
      upperLinkTarget = armPoseTable[currentTableInd].y;//update lower link targ with new pose y, which represents the upper VBar target
    }
}

double PIDCalcArm(int currAngle,int targAngle,double Kp)
{
    //return the motor effort given by the lowerlinks PID calculation
    //current angle is subtracted first to result in
    return (targAngle-currAngle)*Kp;
}
bool valWithinTolerance(double currentVal, double targetVal, double tolerance)
{
    return (currentVal+tolerance>=targetVal) && (currentVal-tolerance<=targetVal);
}
void updateLowerLinkagePos(){
    lowerLinkAngle = lowerLinkStartAngle + getLowerLinkageDegreeChange();
}
void updateUpperLinkagePos(){
    upperLinkAngle = upperLinkStartAngle + getUpperLinkageDegreeChange();
}
//To be defined
double getLowerLinkageDegreeChange()
{
  //Gets the pose of the lower linkage. 
  //Note that this will likely end up returning the change in degrees from start position, since getPosA starts at 0 when robot starts up
  return lowerLinkDriveMotor.getPositionA()*lowerLinkEncoderToDegreeConst;
}
//To be defined
double getUpperLinkageDegreeChange()
{
  //Gets the pose of the lower linkage. 
  //Note that this will likely end up returning the change in degrees from start position, since getPosA starts at 0 when robot starts up
  return upperLinkDriveMotor.getPositionB()*upperLinkEncoderToDegreeConst;
}
void closeClaw(){
  servoToPos(clawClosePos);
}
void openClaw(){
  servoToPos(clawOpenPos);
}
void servoToPos(int servoPos){
  //TODO:
}