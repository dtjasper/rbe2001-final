#pragma once

#include "servo32u4.h"
#include "BlueMotor.h"
#include "utils.h"

class arm
{
protected:

    Pose armPose1 {3, 5, 0};
    Pose armPose2 {5, 7, 0};
    Pose armPose3 {0, 0, 0};
    Pose armPoseTable[3] = {armPose1,armPose2,armPose3};

    Pose pose1 {300, -300, 10};
    Pose pose2 {300, 300, 20};
    Pose pose3 {0, 0, 0};
    

    //Values for converting encoder count to degrees. UPDATE
    double lowerLinkEncoderToDegreeConst = 2;
    double upperLinkEncoderToDegreeConst = 0.25;

    //first two nums represent lower linkage and higher linkage angles respectively, third val is to never change.
   
    double lowerLinkPID = 0;

    Servo32U4Pin12 clawServo; // update when know pins
    BlueMotor lowerLinkDriveMotor; //formerly motor A
    BlueMotor upperLinkDriveMotor; //formerly motor B

    BlueMotor motorDriver;

    

    Pose destination_array[2] = {pose2, pose3};
    
public:
    void moveClaw(uint16_t position);
    void armLoop(void);
    void setup();
    void Stop(void); 

protected:
    double PIDCalcArm(int,int,double);
    bool valWithinTolerance(double, double, double);
    double getLowerLinkageDegreeChange(void);
    double getUpperLinkageDegreeChange(void);
    void updateLowerLinkagePos(void);
    void updateUpperLinkagePos(void);   

};
