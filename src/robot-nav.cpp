/**
 * robot-nav.cpp is where you should put navigation routines.
 */

#include "robot.h"





void Robot::UpdatePose(const Twist& twist)
{
    //convert degress back to radians as C++ trig only takes radians
    float theta_rad = currPose.theta * PI / 180.0;
    currPose.x += ((twist.u)*cos(theta_rad));
    currPose.y += ((twist.u)*sin(theta_rad));
    
    //theta here remains in degrees to readability
    currPose.theta += twist.omega;
    /**
     * TODO: Add your FK algorithm to update currPose here.
     */


#ifdef __NAV_DEBUG__
    TeleplotPrint("x", currPose.x);
    TeleplotPrint("y", currPose.y);
    TeleplotPrint("theta", currPose.theta);
#endif
}



/**
 * Sets a destination in the lab frame.
 */
void Robot::SetDestination(const Pose& dest)
{
    
    /**
     * TODO: Turn on LED, as well.
     */
    Serial.print("Setting dest to: ");
    Serial.print(dest.x);
    Serial.print(", ");
    Serial.print(dest.y);
    Serial.print('\n');

    destPose = dest;
    robotState = ROBOT_DRIVE_TO_POINT;


}

bool Robot::CheckReachedDestination(void)
{
    bool retVal = false;

    /**
     * TODO: Add code to check if you've reached destination here.
     */

    float dx = destPose.x - currPose.x;
    float dy = destPose.y - currPose.y;
    float distErr = sqrt(dx*dx + dy*dy);

//the idle state stuff shouldn't be here, should be in handle destination, but it wasn't working
    if (distErr <= 50) {  
        Serial.println("we have reached position");
        chassis.SetMotorEfforts(0,0);
        retVal = true;
    }

    return retVal;
}

void Robot::DriveToPoint(void)
{
    float delta_x = (destPose.x - currPose.x) / 10.0;//in cm
    float delta_y = (destPose.y - currPose.y)/ 10.0;
    float epi_distance = sqrt((delta_x * delta_x) + (delta_y * delta_y));

    float target_theta = atan2(delta_y, delta_x);
    target_theta = target_theta * (180 / PI);
    float epi_theta = target_theta - currPose.theta;

    // normalize heading error
    //while (epi_theta > M_PI) epi_theta -= 2*M_PI;
    //while (epi_theta < -M_PI) epi_theta += 2*M_PI;

    float Kp_dist = 4.4;   // start small, then tune 4.4
    float Kp_theta = 9.5;   //6.2,8,(9,best)

    if (robotState == ROBOT_DRIVE_TO_POINT)
    {
        if (epi_distance < 5.0) { // stop when close enough
            chassis.SetMotorEfforts(0, 0);
            robotState = ROBOT_IDLE;
            return;
        }

        float desiredLeft = Kp_dist*epi_distance - Kp_theta*epi_theta;
        float desiredRight = Kp_dist*epi_distance + Kp_theta*epi_theta;

        int16_t leftEffort = constrain(desiredLeft, -100, 100);
        int16_t rightEffort = constrain(desiredRight, -100, 100);

        chassis.SetMotorEfforts(leftEffort, rightEffort);

        Serial.print("Left Effort: ");
        Serial.println(leftEffort);

        Serial.print("Right Effort: ");
        Serial.println(rightEffort);

        Serial.print("epi-theta: ");
        Serial.println(epi_theta);

        Serial.print("Target Theta: ");
        Serial.println(target_theta);
    }
}



int index = 0;


void Robot::HandleDestination(void)
{

    if(index < 2) {
        SetDestination(destination_array[index]);
        index++;
    } else {
        EnterIdleState();
    }

    
      //TODO: Stop and change state. Turn off LED.

}





//all of mike pannys code starts here



