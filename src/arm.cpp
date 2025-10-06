#include "arm.h"


//Both below values NEED to be updated with actual arm starting positions. UPDATE
double lowerLinkStartAngle = 0;//angle between the first virtual four bar and the ground. Four bar has a hard stop when leaning in towards ROMI's center, this should be around that
double upperLinkStartAngle = 0;//angle between the two "interior" links on both virtual four bars. In resting, try keeping this close to 0 as possibl

//Robot starts in rest, so both values should initialize to their start values

double lowerLinkTarget;
double upperLinkTarget;
bool needOscillate;

double lowerLinkAngle; 
double upperLinkAngle;

//KP Values for driving both linkages. Zero currently for obvious reasons
double lowerKP;
double upperKP; 

double tolerance;


//servo positions designating open and closed claws. UPDATE
uint16_t clawClosePos = 0;
uint16_t clawOpenPos = 0;

//Arm movement poses. First angle is the lower VBar target, second value is the upper VBar target. 
//Third number is either 0, 1. If 0, this is a place w/o need for claw movement. 1 means this point sees open claw, 2 means this point sees close claw is needed
//List of poses is NOT comprehensive and is NOT accurate
//UPDATE
int currentTableInd = 0;

void arm::setup(){
    lowerLinkTarget = armPose1.x;
    upperLinkTarget = armPose1.y;
    needOscillate = armPose1.theta != 0; 
    lowerLinkAngle = lowerLinkStartAngle; 
    upperLinkAngle = upperLinkStartAngle; 
    lowerKP=0;
    upperKP=0;
    tolerance = 5;
}

void arm::moveClaw(uint16_t position){
    clawServo.setTargetPos(position);
}

double arm::PIDCalcArm(int currAngle,int targAngle,double Kp)
{
    //return the motor effort given by the lowerlinks PID calculation
    //current angle is subtracted first to result in
    return (targAngle-currAngle)*Kp;
}
bool arm::valWithinTolerance(double currentVal, double targetVal, double tolerance)
{
    return (currentVal+tolerance>=targetVal) && (currentVal-tolerance<=targetVal);
}
void arm::updateLowerLinkagePos(){
    lowerLinkAngle = lowerLinkStartAngle + getLowerLinkageDegreeChange();
}
void arm::updateUpperLinkagePos(){
    upperLinkAngle = upperLinkStartAngle + getUpperLinkageDegreeChange();
}
//To be defined
double arm::getLowerLinkageDegreeChange()
{
  //Gets the pose of the lower linkage. 
  //Note that this will likely end up returning the change in degrees from start position, since getPosA starts at 0 when robot starts up
  return lowerLinkDriveMotor.getPositionA()*lowerLinkEncoderToDegreeConst;
}
//To be defined
double arm::getUpperLinkageDegreeChange()
{
  //Gets the pose of the lower linkage. 
  //Note that this will likely end up returning the change in degrees from start position, since getPosA starts at 0 when robot starts up
  return upperLinkDriveMotor.getPositionB()*upperLinkEncoderToDegreeConst;
}



void arm::armLoop(){
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
        moveClaw(clawOpenPos);
      }else if(armPoseTable[currentTableInd].theta == 2){// a theta of 2 signals a close claw
        moveClaw(clawClosePos);
      }
      //move index to point to next pose
      currentTableInd++;
      lowerLinkTarget = armPoseTable[currentTableInd].x;//update lower link targ with new pose x, which represents the lower VBar target
      upperLinkTarget = armPoseTable[currentTableInd].y;//update lower link targ with new pose y, which represents the upper VBar target
    }
}
