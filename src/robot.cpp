#include "robot.h"
int count{0};
/*int way_points[9] = {-150, 75, 0,
                    -190, 120, 0,
                    -230, 150, 0
};*/



void Robot::InitializeRobot(void)
{
    chassis.InititalizeChassis();
    theArm.setup();
    lowerLinkDriveMotor.setup();


    //this works
    //int way_points[9] = {(-150,75,0), (3,4), (4,5)};
    //int way_points[9] = {-150, 75, 0};

    //Pose inputDestination1 {200, 400, 0.0};
    //500,-400,0
    //500,-500,0

    SetDestination(pose1);
    /**
     * TODO: Set pin 13 HIGH when navigating and LOW when destination is reached.
     * Need to set as OUTPUT here.
     */
    //pinMode(13, HIGH);
}

void Robot::EnterIdleState(void)
{
    count++;
    chassis.Stop();

    Serial.println("-> IDLE");
    robotState = ROBOT_IDLE;

}


/**
 * The main loop for your robot. Process both synchronous events (motor control),
 * and asynchronous events (distance readings, etc.).
*/
void Robot::RobotLoop(void) 
{
     /**
     * Run the chassis loop, which handles low-level control.
     */
 
    //this robotState thing was not there before, so it may be wrong to have
    if(robotState != ROBOT_IDLE) {
    Twist velocity;
    if(chassis.ChassisLoop(velocity))
    {
        // We do FK regardless of state
        theArm.armLoop();
        UpdatePose(velocity);
        //chassis.SetMotorEfforts(0,0); //220,220
        
        /**
         * Here, we break with tradition and only call these functions if we're in the 
         * DRIVE_TO_POINT state. CheckReachedDestination() is expensive, so we don't want
         * to do all the maths when we don't need to.
         * 
         * While we're at it, we'll toss DriveToPoint() in, as well.
         */ 
        if(robotState == ROBOT_DRIVE_TO_POINT)
        {
            DriveToPoint();
            if(CheckReachedDestination()) HandleDestination();
        }
    }
    }
}
