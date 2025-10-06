#pragma once

#include "chassis.h"

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

    // For managing key presses
    String keyString;

    /**
     * For tracking current pose and the destination.
     */
    Pose currPose;
    Pose destPose;



    Pose pose1 {300, -300, 10};
    Pose pose2 {300, 300, 20};
    Pose pose3 {0, 0, 0};

    //first two nums represent lower linkage and higher linkage angles respectively, third val is to never change.
   
    double lowerLinkPID = 0;
    
    

    

    Pose destination_array[2] = {pose2, pose3};
    
public:
    Robot(void) {keyString.reserve(10);}
    void InitializeRobot(void);
    void RobotLoop(void);

    void armLoop();

protected:
    /* State changes */    
    void EnterIdleState(void);

    // /* Navigation methods.*/
    void UpdatePose(const Twist& u);
    void SetDestination(const Pose& destination);
    void DriveToPoint(void);
    bool CheckReachedDestination(void);
    void HandleDestination(void);
    void PIDCalc(int currAngle, int targAngle, double Kp);
    double getLowerLinkPos();
    void linkageLoop();
    void Linkageloop();
    double PIDCalcArm(int currAngle, int targAngle, double Kp);
    bool valWithinTolerance(double currentVal, double targetVal, double tolerance);
    void WholeLinkageToTarget(int lowerAngle, int upperAngle);
    void calcArmPos();
    void lowerLinkageToTarget(int targetAngle);
    void LinkageToTarget(int, int);
};
