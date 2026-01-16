#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "interfaces/msg/motors_order.hpp"
#include "interfaces/msg/motors_feedback.hpp"
#include "interfaces/msg/steering_calibration.hpp"
#include "interfaces/msg/joystick_order.hpp"
#include "interfaces/msg/ultrasonic.hpp"
#include "interfaces/msg/ai.hpp"
#include "interfaces/msg/state.hpp"
#include "interfaces/msg/following_state_event.hpp"

#include "std_srvs/srv/empty.hpp"

#include "../include/car_control/steeringCmd.h"
#include "../include/car_control/propulsionCmd.h"
#include "../include/car_control/car_control_node.h"

using namespace std;
using placeholders::_1;

int us_front=900;
int us_front_right=900;
int us_front_left=900;
int us_rear=900;
int us_rear_right=900;
int us_rear_left=900;
int is_stopped=0;

int timer = 0;
float past_steering_angle = 0.0 ;
int ai_x = 0;
int ai_y = 0;
int ai_w = 0;
int ai_h = 0;
std::string ai_class_name = "person";
int ai_id = 0 ;
float ai_proba = 0.0;
float ai_estimated_distance = 0.0 ;

int car_state = 1;
bool follow_mode = false;      // false -> No follow, true -> Follow master
int following_id = -1;         // ID de la personne suivie
int follow_state = 1;          // 1 -> Looking for a master, 2 -> Master found, waiting for gestures, 3 -> Following master
std::string following_state_event = "none";

int gesture_id = -1 ;
std::string gesture_name = "aaaaaaaaaaaaaa";
float gesture_probability = 0.0 ;

class car_control : public rclcpp::Node {

public:
    car_control()
    : Node("car_control_node")
    {
        start = false;
        mode = 0;
        requestedThrottle = 0;
        requestedSteerAngle = 0;

        publisher_can_= this->create_publisher<interfaces::msg::MotorsOrder>("motors_order", 10);

        publisher_steeringCalibration_ = this->create_publisher<interfaces::msg::SteeringCalibration>("steering_calibration", 10);

        publisher_followingStateEvent_ = this->create_publisher<interfaces::msg::FollowingStateEvent>("following_state_event", 10);
        

        subscription_joystick_order_ = this->create_subscription<interfaces::msg::JoystickOrder>(
        "joystick_order", 10, std::bind(&car_control::joystickOrderCallback, this, _1));

        subscription_motors_feedback_ = this->create_subscription<interfaces::msg::MotorsFeedback>(
        "motors_feedback", 10, std::bind(&car_control::motorsFeedbackCallback, this, _1));

        subscription_steering_calibration_ = this->create_subscription<interfaces::msg::SteeringCalibration>(
        "steering_calibration", 10, std::bind(&car_control::steeringCalibrationCallback, this, _1));

        subscription_us_data_ = this->create_subscription<interfaces::msg::Ultrasonic>(
        "us_data", 10, std::bind(&car_control::ultrasonicCallback, this, _1));

        subscription_AI = this->create_subscription<interfaces::msg::AI>(
        "ai_perception_data", 10, std::bind(&car_control::AICallback, this, _1));
        
        subscription_State = this->create_subscription<interfaces::msg::State>(
        "state", 10, std::bind(&car_control::StateCallback, this, _1));
        

        server_calibration_ = this->create_service<std_srvs::srv::Empty>(
                            "steering_calibration", std::bind(&car_control::steeringCalibration, this, std::placeholders::_1, std::placeholders::_2));

	// timer_following_update_ = this->create_wall_timer(std::chrono::milliseconds(250), [&](){this->publishingFollowingState();};
	
        timer_ = this->create_wall_timer(PERIOD_UPDATE_CMD, std::bind(&car_control::updateCmd, this));

        
        RCLCPP_INFO(this->get_logger(), "car_control_node READY");

    }

    
private:


    void AICallback(const interfaces::msg::AI & AI){
        int i = 0;
        
        if (AI.boxes.empty()) {
        RCLCPP_INFO(this->get_logger(), "No human detected");
        following_id = -1 ;
        }
        else{
            ai_x = AI.boxes[i].x;
            ai_y = AI.boxes[i].y;
            ai_w = AI.boxes[i].w ;
            ai_h = AI.boxes[i].h ;
            ai_class_name = AI.boxes[i].class_name;
            ai_id = AI.boxes[i].id ;
            ai_proba = AI.boxes[i].probability;
            ai_estimated_distance = AI.boxes[i].estimated_distance ;

            gesture_id = AI.gesture.id ;
            gesture_name = AI.gesture.class_name;
            gesture_probability = AI.gesture.probability ;
        }
    }

    void StateCallback(const interfaces::msg::State & State_controller){
        int i = 0;
        car_state = State_controller.state;
    }

    void ultrasonicCallback(const interfaces::msg::Ultrasonic & ultrasonic) {
        us_front= ultrasonic.front_center;
        us_front_right= ultrasonic.front_right;
        us_front_left= ultrasonic.front_left;
        us_rear= ultrasonic.rear_center;
        us_rear_right= ultrasonic.rear_right;
        us_rear_left= ultrasonic.rear_left;
    }

    /* Update start, mode, requestedThrottle, requestedSteerAngle and reverse from joystick order [callback function]  :
    *
    * This function is called when a message is published on the "/joystick_order" topic
    */
    void joystickOrderCallback(const interfaces::msg::JoystickOrder & joyOrder) {

        if (joyOrder.start != start){
            start = joyOrder.start;

            if (start)
                RCLCPP_INFO(this->get_logger(), "START");
            else 
                RCLCPP_INFO(this->get_logger(), "STOP");
        }
        

        if (joyOrder.mode != mode && joyOrder.mode != -1){ //if mode change
            mode = joyOrder.mode;

            if (mode==0){
                RCLCPP_INFO(this->get_logger(), "Switching to MANUAL Mode");
            }else if (mode==1){
                RCLCPP_INFO(this->get_logger(), "Switching to AUTONOMOUS Mode");
            }else if (mode==2){
                RCLCPP_INFO(this->get_logger(), "Switching to STEERING CALIBRATION Mode");
                startSteeringCalibration();
            }
        }
        
        if (mode == 0 && start){  //if manual mode -> update requestedThrottle, requestedSteerAngle and reverse from joystick order
            requestedThrottle = joyOrder.throttle;
            requestedSteerAngle = joyOrder.steer;
            reverse = joyOrder.reverse;
        }
    }

    /* Update currentAngle from motors feedback [callback function]  :
    *
    * This function is called when a message is published on the "/motors_feedback" topic
    * 
    */
    void motorsFeedbackCallback(const interfaces::msg::MotorsFeedback & motorsFeedback){
        (void)motorsFeedback; // not used for now
        //currentAngle = motorsFeedback.steering_angle;
    }


    /* Update PWM commands : leftRearPwmCmd, rightRearPwmCmd, steeringPwmCmd
    *
    * This function is called periodically by the timer [see PERIOD_UPDATE_CMD in "car_control_node.h"]
    * 
    * In MANUAL mode, the commands depends on :
    * - requestedThrottle, reverse, requestedSteerAngle [from joystick orders]
    * - currentAngle [from motors feedback]
    */
    void updateCmd(){

        publishingFollowingState();

        auto motorsOrder = interfaces::msg::MotorsOrder();

        int d_stop= 60; 
        int d_slowdown= 150;
        int v_null= 50; 
        float cmd=0;
        
        if (!start){    //Car stopped
            leftRearPwmCmd = STOP;
            rightRearPwmCmd = STOP;
            //steeringPwmCmd = STOP;


        }else{ //Car started

            //Manual Mode <==> MODE DE FONCTIONNEMENT à l'ARRET
            if (mode==0){
                
                // Run mode
                if (car_state == 1) {

                    // Réaction à la perte de l'id du master car le master est sortie du champs de vision de la caméra
                    if (ai_id != following_id and following_id != -1) {
                        //RCLCPP_INFO(this->get_logger(), "Master lost!");
                        following_id = -1; // Aucun id n'est suivi
                        follow_mode = 1; // Pas de suivi
                        follow_state = 1; // Looking for a master
                        leftRearPwmCmd = 50; // Mode de fonctionnement à l'arret
                        rightRearPwmCmd = 50; // Mode de fonctionnement à l'arret
                        if (following_state_event == "none") {
                            following_state_event = "master_lost";
                        }
                    }

                    if (following_id == -1) { // Aucun humain n'est le master
                        //RCLCPP_INFO(this->get_logger(), "Looking for a master...");
                        follow_state = 1; // Looking for a master
                        follow_mode = false; // Pas de suivi
                        leftRearPwmCmd = 50; // Mode de fonctionnement à l'arret
                        rightRearPwmCmd = 50; // Mode de fonctionnement à l'arret
                        if (ai_id != -1) {
                            following_id = ai_id;
                        } 
                        if (following_state_event == "none") {
                            following_state_event = "looking_for_master";
                        }
                    } else {

                        if (follow_mode == false) {
                            follow_state = 2; // Master found, waiting for gestures from the master
                            //RCLCPP_INFO(this->get_logger(), "Waiting for gestures from the detected master...");
                            if (following_state_event == "none") {
                                following_state_event = "waiting_for_gestures";
                            }
                        }
                        
                        if (gesture_id == following_id) {
                            if (gesture_name == "Victory") {
                                follow_mode=false; // On arrête à suivre le master
                                follow_state = 1; // Master found, waiting for gestures from the master
                                following_id = -1;
                                //RCLCPP_INFO(this->get_logger(), "Changing master!");
                                if (following_state_event == "none") {
                                    following_state_event = "change_master";
                                }

                            } else if (gesture_name == "Thumb_Up") {
                                follow_mode=false; // On met la voiture en STOP
                                follow_state = 2; // Master found, waiting for gestures from the master
                                //RCLCPP_INFO(this->get_logger(), "Stop to follow master!");
                                if (following_state_event == "none") {
                                    following_state_event = "stop_following";
                                }
                            } else if (gesture_name == "Pointing_Up" and follow_mode==false) {
                                follow_mode=true; // On commence à suivre le master
                                follow_state = 3; // Follow master
                                //RCLCPP_INFO(this->get_logger(), "Starting to follow master!");
                                if (following_state_event == "none") {
                                    following_state_event = "following_master";
                                }
                            }
                        }
                    }

                    // Activation du suivi du master avec le booléen follow_mode
                    if (follow_mode) {
                        //RCLCPP_INFO(this->get_logger(), "Following master");
                        follow_state = 3; // Follow master
                        // NE PAS MODIFIER - // MODE DE FONCTIONNEMENT à l'ARRET
                        leftRearPwmCmd = 50; // Mode de fonctionnement à l'arret
                        rightRearPwmCmd = 50; // Mode de fonctionnement à l'arret
                        requestedThrottle = 0;
                        
                        requestedSteerAngle = suivreHumain(ai_x, ai_w);
                        
                        // On refait une saturation
                        if (requestedSteerAngle > 1.0) {
                            requestedSteerAngle = 1.0;
                        }
                        else if (requestedSteerAngle < -1.0) {
                            requestedSteerAngle = -1.0;
                        }

                    } else {
                        requestedSteerAngle = 0;
                        leftRearPwmCmd = 50;
                        rightRearPwmCmd = 50;
                    }
                
                // Stop mode
                } else {
                    requestedSteerAngle = 0;
                    leftRearPwmCmd = 50;
                    rightRearPwmCmd = 50;
                }

                // ## Ancienne version du mode manuel ##
                //
                // if (car_state == 1) {
                //     manualPropulsionCmd(requestedThrottle, reverse, leftRearPwmCmd, rightRearPwmCmd);
                //     if (leftRearPwmCmd == 50 && rightRearPwmCmd == 50) {
                //         is_stopped = 1;
                //     }
                //     if (is_stopped == 1) {
                //         autoBloque (is_stopped, us_front_left, us_front_right, us_rear_right, us_rear_left, us_rear, requestedSteerAngle, leftRearPwmCmd, rightRearPwmCmd );
                //     }
                //     obstacleDetection2(requestedSteerAngle,requestedThrottle, reverse, leftRearPwmCmd, rightRearPwmCmd, us_front, us_front_right, us_front_left, us_rear, us_rear_right, us_rear_left);
                //     //RCLCPP_INFO(this->get_logger(), "Gesture : %s", gesture_name.c_str());

                // } else {
                //     requestedSteerAngle = 0;
                //     leftRearPwmCmd = 50;
                //     rightRearPwmCmd = 50;
                // }

            //Autonomous Mode
            } else if (mode==1){

                // Run mode
                if (car_state == 1) {

                    // Réaction à la perte de l'id du master car le master est sortie du champs de vision de la caméra
                    if (ai_id != following_id and following_id != -1) {
                        //RCLCPP_INFO(this->get_logger(), "Master lost!");
                        following_id = -1; // Aucun id n'est suivi
                        follow_mode = 1; // Pas de suivi
                        follow_state = 1; // Looking for a master
                        leftRearPwmCmd = 50; // Mode de fonctionnement à l'arret
                        rightRearPwmCmd = 50; // Mode de fonctionnement à l'arret
                        if (following_state_event == "none") {
                            following_state_event = "master_lost";
                        }
                    }

                    if (following_id == -1) { // Aucun humain n'est le master
                        //RCLCPP_INFO(this->get_logger(), "Looking for a master...");
                        follow_state = 1; // Looking for a master
                        follow_mode = false; // Pas de suivi
                        leftRearPwmCmd = 50; // Mode de fonctionnement à l'arret
                        rightRearPwmCmd = 50; // Mode de fonctionnement à l'arret
                        if (ai_id != -1) {
                            following_id = ai_id;
                        } 
                        if (following_state_event == "none") {
                            following_state_event = "looking_for_master";
                        }
                    } else {

                        if (follow_mode == false) {
                            follow_state = 2; // Master found, waiting for gestures from the master
                            //RCLCPP_INFO(this->get_logger(), "Waiting for gestures from the detected master...");
                            if (following_state_event == "none") {
                                following_state_event = "waiting_for_gestures";
                            }
                        }
                        
                        if (gesture_id == following_id) {
                            if (gesture_name == "Victory") {
                                follow_mode=false; // On arrête à suivre le master
                                follow_state = 1; // Master found, waiting for gestures from the master
                                following_id = -1;
                                //RCLCPP_INFO(this->get_logger(), "Changing master!");
                                if (following_state_event == "none") {
                                    following_state_event = "change_master";
                                }

                            } else if (gesture_name == "Thumb_Up") {
                                follow_mode=false; // On met la voiture en STOP
                                follow_state = 2; // Master found, waiting for gestures from the master
                                //RCLCPP_INFO(this->get_logger(), "Stop to follow master!");
                                if (following_state_event == "none") {
                                    following_state_event = "stop_following";
                                }
                            } else if (gesture_name == "Pointing_Up" and follow_mode==false) {
                                follow_mode=true; // On commence à suivre le master
                                follow_state = 3; // Follow master
                                //RCLCPP_INFO(this->get_logger(), "Starting to follow master!");
                                if (following_state_event == "none") {
                                    following_state_event = "following_master";
                                }
                            }
                        }
                    }

                    // Activation du suivi du master avec le booléen follow_mode
                    if (follow_mode) {
                        //RCLCPP_INFO(this->get_logger(), "Following master");
                        follow_state = 3; // Follow master
                        leftRearPwmCmd = 80;
                        rightRearPwmCmd = 80;
                        requestedThrottle = 0.8;
                        past_steering_angle = requestedSteerAngle;
                        requestedSteerAngle = suivreHumain(ai_x, ai_w);
                        if ((requestedSteerAngle - past_steering_angle) > 0.1){
                            requestedSteerAngle = past_steering_angle + 0.1 ;
                        }
                        else if ((requestedSteerAngle - past_steering_angle) < -0.1){
                            requestedSteerAngle = past_steering_angle - 0.1 ;
                        }
                        // On refait une saturation
                        if (requestedSteerAngle > 1.0) {
                            requestedSteerAngle = 1.0;
                        }
                        else if (requestedSteerAngle < -1.0) {
                            requestedSteerAngle = -1.0;
                        }
                        //obstacleDetection2(requestedSteerAngle,requestedThrottle, reverse, leftRearPwmCmd, rightRearPwmCmd, us_front, us_front_right, us_front_left, us_rear, us_rear_right, us_rear_left);
                    } else {
                        requestedSteerAngle = 0;
                        leftRearPwmCmd = 50;
                        rightRearPwmCmd = 50;
                    }
                
                // Stop mode
                } else {
                    requestedSteerAngle = 0;
                    leftRearPwmCmd = 50;
                    rightRearPwmCmd = 50;
                }
            }
        }

        //Send order to motors
        motorsOrder.left_rear_pwm = leftRearPwmCmd;
        motorsOrder.right_rear_pwm = rightRearPwmCmd;

        motorsOrder.steering_angle = (int8_t)((int8_t)(requestedSteerAngle*127.0)); //Scale [-1,1] to [-127,+127]
        currentAngle = requestedSteerAngle;

        
        timer+=1;
        if (timer == 20){
            publisher_can_->publish(motorsOrder);
            // RCLCPP_INFO(this->get_logger(), "Steering angle : %f ", requestedSteerAngle);
            // RCLCPP_INFO(this->get_logger(), "Pos x AI : %d ", ai_x);
            timer = 0;
        }
        
    }


    /* Start the steering calibration process :
    *
    * Publish a calibration request on the "/steering_calibration" topic
    */
    void startSteeringCalibration(){

        auto calibrationMsg = interfaces::msg::SteeringCalibration();
        calibrationMsg.request = true;

        RCLCPP_INFO(this->get_logger(), "Sending calibration request .....");
        publisher_steeringCalibration_->publish(calibrationMsg);
    }

    /* Start publishing the following states :
    *
    * Publish the state of the following of the car on the topic following_state_event
    */
    void publishingFollowingState(){

        auto msg = interfaces::msg::FollowingStateEvent();

        if (following_state_event != "none") {
            if (following_state_event == "master_lost") {
                msg.state = 1;
            } else if (following_state_event == "looking_for_master") {
                msg.state = 2;
            } else if (following_state_event == "waiting_for_gestures") {
                msg.state = 3;
            } else if (following_state_event == "change_master") {
                msg.state = 4;
            } else if (following_state_event == "stop_following") {
                msg.state = 5;
            } else if (following_state_event == "following_master") {
                msg.state = 6;
            }

            following_state_event = "none";
        } else {
            msg.state = -1;
        }
        
        // Only publish when a change occurs to limit subscriber's work
        if (previous_following_state != msg.state) {
        	RCLCPP_INFO(this->get_logger(), "Following state event : %i", msg.state);
        	publisher_followingStateEvent_->publish(msg);
        	previous_following_state = msg.state;
        }
    }


    /* Function called by "steering_calibration" service
    * 1. Switch to calibration mode
    * 2. Call startSteeringCalibration function
    */
    void steeringCalibration([[maybe_unused]] std_srvs::srv::Empty::Request::SharedPtr req,
                            [[maybe_unused]] std_srvs::srv::Empty::Response::SharedPtr res)
    {

        mode = 2;    //Switch to calibration mode
        RCLCPP_WARN(this->get_logger(), "Switching to STEERING CALIBRATION Mode");
        startSteeringCalibration();
    }
    

    /* Manage steering calibration process [callback function]  :
    *
    * This function is called when a message is published on the "/steering_calibration" topic
    */
    void steeringCalibrationCallback (const interfaces::msg::SteeringCalibration & calibrationMsg){

        if (calibrationMsg.in_progress == true && calibrationMsg.user_need == false){
        RCLCPP_INFO(this->get_logger(), "Steering Calibration in progress, please wait ....");

        } else if (calibrationMsg.in_progress == true && calibrationMsg.user_need == true){
            RCLCPP_WARN(this->get_logger(), "Please use the buttons (L/R) to center the steering wheels.\nThen, press the blue button on the NucleoF103 to continue");
        
        } else if (calibrationMsg.status == 1){
            RCLCPP_INFO(this->get_logger(), "Steering calibration [SUCCESS]");
            RCLCPP_INFO(this->get_logger(), "Switching to MANUAL Mode");
            mode = 0;    //Switch to manual mode
            start = false;  //Stop car
        
        } else if (calibrationMsg.status == -1){
            RCLCPP_ERROR(this->get_logger(), "Steering calibration [FAILED]");
            RCLCPP_INFO(this->get_logger(), "Switching to MANUAL Mode");
            mode = 0;    //Switch to manual mode
            start = false;  //Stop car
        }
    
    }
    
    // ---- Private variables ----

    //General variables
    bool start;
    int mode;    //0 : Manual    1 : Auto    2 : Calibration
    int8_t previous_following_state = -10;

    
    //Motors feedback variables
    float currentAngle;

    //Manual Mode variables (with joystick control)
    bool reverse;
    float requestedThrottle;
    float requestedSteerAngle;

    //Control variables
    uint8_t leftRearPwmCmd;
    uint8_t rightRearPwmCmd;
    uint8_t steeringPwmCmd;

    //Publishers
    rclcpp::Publisher<interfaces::msg::MotorsOrder>::SharedPtr publisher_can_;
    rclcpp::Publisher<interfaces::msg::SteeringCalibration>::SharedPtr publisher_steeringCalibration_;
    rclcpp::Publisher<interfaces::msg::FollowingStateEvent>::SharedPtr publisher_followingStateEvent_;

    //Subscribers
    rclcpp::Subscription<interfaces::msg::JoystickOrder>::SharedPtr subscription_joystick_order_;
    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr subscription_motors_feedback_;
    rclcpp::Subscription<interfaces::msg::SteeringCalibration>::SharedPtr subscription_steering_calibration_;
    rclcpp::Subscription<interfaces::msg::Ultrasonic>::SharedPtr subscription_us_data_;
    rclcpp::Subscription<interfaces::msg::AI>::SharedPtr subscription_AI;
    rclcpp::Subscription<interfaces::msg::State>::SharedPtr subscription_State;

    //Timer
    rclcpp::TimerBase::SharedPtr timer_;

    //Steering calibration Service
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr server_calibration_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<car_control>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
