/**
 * robot-nav.cpp is where you should put navigation routines.
 */

#include "robot.h"
#include "chassis.h"

void Robot::UpdatePose(const Twist& twist)
{
    /**
     * TODO: Add your FK algorithm to update currPose here.
     */
    float dt = (chassis.CONTROL_LOOP_PERIOD_MS) / 1000.0;
    
    double prevtheta = currPose.theta;
    currPose.theta += twist.omega * dt;
    double thetastar = (currPose.theta + prevtheta)/2.0;
    currPose.x += (dt * (twist.u)*(cos(thetastar)));
    currPose.y += (dt * (twist.u)*(sin(thetastar)));


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
    if (currPose.x > (destPose.x - 2) && currPose.x < (destPose.x + 2)){
        if (currPose.y > (destPose.y - 2) && currPose.y< (destPose.y + 2)){
            retVal = true;
        }
    }

    return retVal;
}

void Robot::DriveToPoint(void)
{
    if(robotState == ROBOT_DRIVE_TO_POINT)
    {
        float distError;
        float headingError;
        float dx = destPose.x - currPose.x;
        float dy = destPose.y - currPose.y;

        distError = sqrt(dx*dx + dy*dy);
        headingError = atan2(dy, dx) - currPose.theta;
        if (headingError > PI){
            headingError -= (2*PI);
        }
        else if(headingError < -PI){
            headingError += (2*PI);
        }
        TeleplotPrint("distanceError", distError);
        TeleplotPrint("headingError", headingError);

#ifdef __NAV_DEBUG__
        // Print useful stuff here.
#endif

        /**
         * TODO: Call chassis.SetMotorEfforts() to command the motion, based on your calculations above.
         */
        float kp_distance = 2.0;
        float kp_theta = 100.0;

        float v = kp_distance * distError;
        float w = kp_theta * -headingError;

        float left_effort = v-w;
        float right_effort = v + w;

        chassis.SetMotorEfforts(left_effort + 40, right_effort + 40);
        
    }
}

void Robot::HandleDestination(void)
{
    /**
     * TODO: Stop and change state. Turn off LED.
     */


    if (Index == pointSize){
        chassis.SetMotorEfforts(0,0);
        robotState = ROBOT_IDLE;
    }else{
        SetDestination(point[Index]);
        Index++;
    }
    

    return;
}