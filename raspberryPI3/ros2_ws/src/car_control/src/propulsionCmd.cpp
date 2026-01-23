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

//Retourne le steering angle pour suivre la personne
float suivreHumain (int x, int w, float past_steer_angle) {

    float output = 0.0 ;
    float x_center = 200.0;
    float alpha = 0.05;
    float output_filtre = past_steer_angle;

    if (x > x_center && x < 2*x_center) {
        output = (float(x)-x_center)/x_center;
    } else if (x <= x_center) {
        output = -(x_center-float(x))/x_center;
    } else {
        output = 1.0;
    }

    output_filtre = alpha * output + (1 - alpha) * output_filtre;

    return output_filtre ;

}







int * obstacleDetection2(float& requestedSteerAngle, float requestedThrottle, bool reverse, uint8_t& leftRearPwmCmd, uint8_t& rightRearPwmCmd,int us_front,int us_front_right,int us_front_left,int us_rear,int us_rear_right,int us_rear_left){
    // FRONT OBSTACLE 
    int d_stop=50; 
    int d_slowdown=150;
    int d_obstacle2avoid=70;
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
        //requestedSteerAngle = +1;
        float sum_lr = us_front_left + us_front_right;
        if(sum_lr < 0.01) sum_lr = 0.01; // avoid division by zero
        float error = (us_front_right - us_front_left) / sum_lr;

        float output = 0 ;
        
        if(error > 1.0) {
            output = 1.0;
        }

        else if(error < -1.0){
            output = -1.0;
        } 
        else {
            output = error ;
        }
        requestedSteerAngle = output;
    }
    else if (us_front_right <= d_obstacle2avoid && us_front_left > us_front_right && reverse == false){
        //requestedSteerAngle = -1;
        float sum_lr = us_front_left + us_front_right;
        if(sum_lr < 0.01) sum_lr = 0.01; // avoid division by zero
        float error = (us_front_right - us_front_left) / sum_lr;

        float output = 0 ;
        
        if(error > 1.0) {
            output = 1.0;
        }

        else if(error < -1.0){
            output = -1.0;
        } 
/*         else if (error < 0.1 || error > 0.1) { //A ENLEVER PEUT ETRE
            output = output;
        } */
        else {
            output = error;
        }
        requestedSteerAngle = output;
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
    /* if (us_rear_left <= d_obstacle2avoid && us_rear_left < us_rear_right && reverse == true){
        requestedSteerAngle = +1;
    }
    else if (us_rear_right <= d_obstacle2avoid && us_rear_left > us_rear_right && reverse == true ){
        requestedSteerAngle = -1;
    }
 */
    //RCLCPP_INFO(this->get_logger(), "Vitesse: %d | Front distance: %d | Rear distance: %d  | CMD: %f", rightRearPwmCmd, us_front, us_rear, cmd);
    
    
    
    
    
    return 0;

}


/* float Steercalculation (int us_front_left, int us_front_right, int us_rear_right, int us_rear_left){
    float sum_lr = us_front_left + us_front_right;
    if(sum_lr < 0.01) sum_lr = 0.01; // avoid division by zero
    float error = (float)(us_front_right - us_front_left) / sum_lr;

    float output = 0 ;
    
    if(error > 1.0) {
        output = 1.0;
    }

    else if(error < -1.0){
        output = -1.0;
    } 
    else {
        output = error ;
    }
    return output ;
} */

void autoBloque (int& is_stopped, int us_front_left, int us_front_right, int us_rear_right, int us_rear_left, int us_rear, float& requestedSteerAngle, uint8_t& leftRearPwmCmd, uint8_t& rightRearPwmCmd ) {
    int dist_min_avance = 30;
    int dist_min_recule = 40;
    int manouvre_finished_cpt;
    int turn_left = 0;

    if (us_front_left < dist_min_recule && us_front_right > us_front_left) {
        requestedSteerAngle = -0.6;
        leftRearPwmCmd = 25;
        rightRearPwmCmd = 25;
        turn_left = 1;
    }
    else if (us_front_right < dist_min_recule && us_front_left > us_front_right) {
        requestedSteerAngle = 0.6;
        leftRearPwmCmd = 25;
        rightRearPwmCmd = 25;
        turn_left = 0;
    }

/*     if (us_rear_right < dist_min_avance && (float)us_rear_left > 1.75*(float)us_rear_right) {
        requestedSteerAngle = -0.6;
 
        leftRearPwmCmd = 60;
        rightRearPwmCmd = 60;
    }
    else if (us_rear_left < dist_min_avance && (float)us_rear_right > 1.75*(float)us_rear_left) {
        requestedSteerAngle = 0.6;

        leftRearPwmCmd = 60;
        rightRearPwmCmd = 60;
    }
 */

/*     if (us_rear_right < dist_min_avance || us_rear_left < dist_min_avance){
        leftRearPwmCmd = 70;
        rightRearPwmCmd = 70;        
        if ((float)us_rear_left > 1.75*(float)us_rear_right) {
            requestedSteerAngle = -0.6;
        }
        else if (us_rear_left < dist_min_avance && (float)us_rear_right > 1.75*(float)us_rear_left) {
            requestedSteerAngle = 0.6;
        }
    } */

    else {
        is_stopped = 0;
       /*  if (manouvre_finished_cpt == 1000 && turn_left == 1) {
            manouvre_finished_cpt = 0;
            requestedSteerAngle = 0.6;
            leftRearPwmCmd = 70;
            rightRearPwmCmd = 70;        
        }
        else if (manouvre_finished_cpt == 1000 && turn_left == 0) {
            manouvre_finished_cpt = 0;
            requestedSteerAngle = -0.6;
            leftRearPwmCmd = 70;
            rightRearPwmCmd = 70; //ENELEVER TOUT C PEUT ETRE
        }
        manouvre_finished_cpt =+ 1; */
    }
    

}








