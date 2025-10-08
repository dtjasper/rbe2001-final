#include "arm.h"


//Both below values NEED to be updated with actual arm starting positions. UPDATE
double lowerLinkStartAngle = 150;//angle between the first virtual four bar and the ground. Four bar has a hard stop when leaning in towards ROMI's center, this should be around that
double upperLinkStartAngle = 0;//angle between the two "interior" links on both virtual four bars. In resting, try keeping this close to 0 as possibl

//Robot starts in rest, so both values should initialize to their start values

// VARIABLES DEFINED IN SETUP
double lowerLinkTarget;
double upperLinkTarget;
bool needOscillate;

double lowerLinkAngle=0; 
double upperLinkAngle;

//KP Values for driving both linkages. Zero currently for obvious reasons
double lowerKP;
double upperKP; 

double tolerance;

double prevLowerPos = 0;
double prevUpperPos = 0;


//servo positions designating open and closed claws. UPDATE
uint16_t clawClosePos = 0;
uint16_t clawOpenPos = 0;

//Arm movement poses. First angle is the lower VBar target, second value is the upper VBar target. 
//Third number is either 0, 1. If 0, this is a place w/o need for claw movement. 1 means this point sees open claw, 2 means this point sees close claw is needed
//List of poses is NOT comprehensive and is NOT accurate
//UPDATE
int currentTableInd = 0;

void arm::setup(){
    lowerLinkTarget = 90;
    upperLinkTarget = armPose1.y;
    needOscillate = armPose1.theta != 0; 
    lowerLinkAngle = lowerLinkStartAngle; 
    upperLinkAngle = upperLinkStartAngle; 
    lowerKP=32;

    upperKP=10;
    tolerance = 2.5;
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
  // bool retVal = (currentVal + tolerance == targetVal + tolerance) || (currentVal - tolerance == targetVal - tolerance)
  //(currentVal+tolerance>=targetVal) && (currentVal-tolerance<=targetVal);
  return ((currentVal <= targetVal + tolerance) && (currentVal >= targetVal - tolerance));
}
void arm::updateLowerLinkagePos(){
  double currentMotorPos = motorDriver.getPositionA();
  double motorTranslate;
  if(motorDriver.getMotorAEffort()>0){
    motorTranslate = pow(pow(currentMotorPos-prevLowerPos,2),1.0/2);
  }else if(motorDriver.getMotorAEffort()<0){
    motorTranslate = -pow(pow(currentMotorPos-prevLowerPos,2),1.0/2);
  }else{
    motorTranslate = 0;
  }
  Serial.print(pow(pow(currentMotorPos-prevLowerPos,1),1/2));
  Serial.println();
  lowerLinkAngle = lowerLinkAngle+ motorTranslate*lowerLinkEncoderToDegreeConst;
  Serial.print(motorTranslate);
  Serial.println("TRANSLATE");
  prevLowerPos = currentMotorPos;

  // Serial.println(motorDriver.getMotorAEffort()*lowerLinkEncoderToDegreeConst);
  // Serial.println("SUPPOSED CALC");
  // double oldLower = lowerLinkAngle;
  // lowerLinkAngle = oldLower + motorDriver.getMotorAEffort()*lowerLinkEncoderToDegreeConst;
  
    
}
void arm::updateUpperLinkagePos(){
    upperLinkAngle = upperLinkAngle + getUpperLinkageDegreeChange();
}
double arm::getLowerLinkageDegreeChange()
{
  //Gets the pose of the lower linkage. 
  //Note that this will likely end up returning the change in degrees from start position, since getPosA starts at 0 when robot starts up
  // double retVal = (motorDriver.getPositionA()-prevLowerPos)*lowerLinkEncoderToDegreeConst;
  // prevLowerPos = motorDriver.getPositionA();
  // return retVal;

  //return motorDriver.getMotorAEffort()>0? motorDriver.getPositionA()*lowerLinkEncoderToDegreeConst
  return 1.0;
}
//To be defined
double arm::getUpperLinkageDegreeChange()
{
  //Gets the pose of the lower linkage. 
  //Note that this will likely end up returning the change in degrees from start position, since getPosA starts at 0 when robot starts up
  double retVal = (motorDriver.getPositionB()-prevUpperPos)*upperLinkEncoderToDegreeConst;
  prevUpperPos = motorDriver.getPositionB();
  return retVal;
}

void arm::Stop(){
  motorDriver.goodSetEffort(0, true);
  motorDriver.goodSetEffort(0, false);
}

void arm::armLoop(){
  boolean serialStop = false;
  //Linkages always trying to move to targets
  //TODO: implement pose updating for both linkages

  //To ensure two motors arent active at once, this code exists to oscillate between which motors are active every loop
  //In the event running dual motors becomes possible, code to replace IF block is below
  if(!valWithinTolerance(lowerLinkAngle, lowerLinkTarget, 5)){
    Serial.print(lowerLinkAngle);
    Serial.println("  LOWER ANGLE");
    Serial.print(PIDCalcArm(lowerLinkAngle,lowerLinkTarget,lowerKP));
    Serial.println("  PID CALC OUTPUT VALUE");
    Serial.print(motorDriver.getMotorAEffort());
    Serial.println("  EFFRT");
    motorDriver.goodSetEffort(PIDCalcArm(lowerLinkAngle,lowerLinkTarget,lowerKP), true);
    //motorDriver.goodSetEffort(PIDCalcArm(lowerLinkAngle,lowerLinkTarget,lowerKP),false);
    updateLowerLinkagePos();

  }
  else{
    if(!serialStop){
      //Serial.println("DONE");
      serialStop = true;
    }
    
    Stop();
  }
  //motorDriver.goodSetEffort(PIDCalcArm(upperLinkAngle,upperLinkTarget,upperKP), false);
  //updateUpperLinkagePos();
/*
  if(lowerLinkDriveMotor.getMotorAEffort() != 0 && !valWithinTolerance(lowerLinkAngle, lowerLinkTarget, lowerKP)){
    motorDriver.goodSetEffort(0, false);
    updateLowerLinkagePos();
    Serial.print(getLowerLinkageDegreeChange());
    motorDriver.goodSetEffort(100, true);
  }
  else if (upperLinkDriveMotor.getMotorBEffort() == 0 && lowerLinkDriveMotor.getMotorAEffort() != 0 && !valWithinTolerance(upperLinkAngle, upperLinkTarget, upperKP)){
    motorDriver.goodSetEffort(0, true);
    updateUpperLinkagePos();
    motorDriver.goodSetEffort(100, false);
  }
  else{
    Stop();
  }

  
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
    
      }*/
}
