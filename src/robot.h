#pragma once

#include "chassis.h"
#include "servo32u4.h"
#include "BlueMotor.h"
#include "arm.h"

class Robot
{
protected:
    /**
     * robotState is used to track the current task of the robot. You will add new states as 
     * the term progresses.
     */
    enum ROBOT_STATE 
    {
        ROBOT_IDLE,
        ROBOT_DRIVE_TO_POINT,
    };
    ROBOT_STATE robotState = ROBOT_IDLE;

    /* Define the chassis*/
    Chassis chassis;
    arm theArm;




    // For managing key presses
    String keyString;

    /**
     * For tracking current pose and the destination.
     */
    Pose currPose;
    Pose destPose;


    Pose armPose1 {300, 300, 0};
    Pose armPose2 {300, 300, 0};
    Pose armPose3 {0, 0, 0};
    //Pose armPoseTable[] = {armPose1,armPose2,armPose3};

    Pose pose1 {300, -300, 10};
    Pose pose2 {300, 300, 20};
    Pose pose3 {0, 0, 0};

    
    BlueMotor lowerLinkDriveMotor; //formerly motor A
    BlueMotor upperLinkDriveMotor; //formerly motor B

    

    Pose destination_array[2] = {pose2, pose3};
    
public:
    Robot(void) {keyString.reserve(10);}
    void InitializeRobot(void);
    void RobotLoop(void);
    
    

protected:
    /* State changes */    
    void EnterIdleState(void);

    // /* Navigation methods.*/
    double getLowerLinkageDegreeChange(void);
    double getUpperLinkageDegreeChange(void);
    void updateUpperLinkagePos(void);
    void updateLowerLinkagePos(void);
    void UpdatePose(const Twist& u);
    void SetDestination(const Pose& destination);
    void DriveToPoint(void);
    bool CheckReachedDestination(void);
    void HandleDestination(void);
};
