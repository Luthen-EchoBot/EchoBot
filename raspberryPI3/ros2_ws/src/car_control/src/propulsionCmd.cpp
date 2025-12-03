#include "../include/car_control/propulsionCmd.h"



/* Calculate rightRearPwmCmd and leftRearPwmCmd (PWM) in MANUAL mode (from joystick orders)
*
* The joystick sends throttle order, which is directly transformed into PWM. The PWMs are equal for both motors in Manual Mode
* 
*/
int * manualPropulsionCmd(float requestedThrottle, bool reverse, uint8_t& leftRearPwmCmd, uint8_t& rightRearPwmCmd){

    if (reverse){
        leftRearPwmCmd = 50 - 50*requestedThrottle;    //requestedThrottle : [0 ; 1] => PWM : [50 -> 0] (reverse)

    } else{
        leftRearPwmCmd = 50 + 50*requestedThrottle;    //requestedThrottle : [0 ; 1] => PWM : [50 -> 100] (forward)
    }

    rightRearPwmCmd = leftRearPwmCmd;

    return 0;

}

int * obstacleDetection1(float requestedThrottle, bool reverse, uint8_t& leftRearPwmCmd, uint8_t& rightRearPwmCmd,int us_front,int us_front_right,int us_front_left,int us_rear,int us_rear_right,int us_rear_left){
    // FRONT OBSTACLE 
    int d_stop= 40; 
    int d_slowdown= 150;
    int v_null = 50 ;
    float cmd=0;

    if ((us_front <= d_stop || us_front_left <= d_stop || us_front_right <= d_stop) && reverse == false) {
        rightRearPwmCmd = v_null;
        leftRearPwmCmd = v_null;
        }
    else if (us_front <= d_slowdown && reverse == false) {
        cmd = requestedThrottle * (float(us_front) / float(d_slowdown - d_stop));
        leftRearPwmCmd = v_null + 50*cmd;
        rightRearPwmCmd = v_null + 50*cmd;
    }

    // REAR OBSTACLE 
    if ((us_rear <= d_stop || us_rear_left <= d_stop || us_rear_right <= d_stop) && reverse == true) {
        rightRearPwmCmd = v_null;
        leftRearPwmCmd = v_null;
    }
    else if (us_rear <= d_slowdown && reverse == true) {
        cmd = requestedThrottle* (float(us_rear) / float(d_slowdown-d_stop));
        leftRearPwmCmd = v_null - 50*cmd;
        rightRearPwmCmd = v_null - 50*cmd;
    } 

    //RCLCPP_INFO(this->get_logger(), "Vitesse: %d | Front distance: %d | Rear distance: %d  | CMD: %f", rightRearPwmCmd, us_front, us_rear, cmd);
    return 0;

}

int * obstacleDetection2(float& requestedSteerAngle, float requestedThrottle, bool reverse, uint8_t& leftRearPwmCmd, uint8_t& rightRearPwmCmd,int us_front,int us_front_right,int us_front_left,int us_rear,int us_rear_right,int us_rear_left){
    // FRONT OBSTACLE 
    int d_stop=40; 
    int d_slowdown=150;
    int d_obstacle2avoid=60;
    int v_null=50;
    float cmd=0;


    if (us_front <= d_stop && reverse == false) {
        rightRearPwmCmd = v_null;
        leftRearPwmCmd = v_null;
        }
    else if (us_front <= d_slowdown && reverse == false) {
        cmd = requestedThrottle * (float(us_front) / float(d_slowdown - d_stop));
        leftRearPwmCmd = v_null + 50*cmd;
        rightRearPwmCmd = v_null + 50*cmd;
    }
    if (us_front_left <= d_obstacle2avoid && us_front_left < us_front_right && reverse == false){
        requestedSteerAngle = +1;
    }
    else if (us_front_right <= d_obstacle2avoid && us_front_left > us_front_right && reverse == false){
        requestedSteerAngle = -1;
    }

    // REAR OBSTACLE 
    if (us_rear <= d_stop && reverse == true) {
        rightRearPwmCmd = v_null;
        leftRearPwmCmd = v_null;
    }
    else if (us_rear <= d_slowdown && reverse == true) {
        cmd = requestedThrottle* (float(us_rear) / float(d_slowdown-d_stop));
        leftRearPwmCmd = v_null - 50*cmd;
        rightRearPwmCmd = v_null - 50*cmd;
    } 
    if (us_rear_left <= d_obstacle2avoid && us_rear_left < us_rear_right && reverse == true){
        requestedSteerAngle = +1;
    }
    else if (us_rear_right <= d_obstacle2avoid && us_rear_left > us_rear_right && reverse == true ){
        requestedSteerAngle = -1;
    }

    //RCLCPP_INFO(this->get_logger(), "Vitesse: %d | Front distance: %d | Rear distance: %d  | CMD: %f", rightRearPwmCmd, us_front, us_rear, cmd);
    return 0;

}