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

int us_front = 900;
int us_front_right = 900;
int us_front_left = 900;
int us_rear = 900;
int us_rear_right = 900;
int us_rear_left = 900;
int is_stopped = 0;

int timer = 0;

enum struct F_State
{
    NONE = 0,
    MASTER_LOST = 1, //unused
    LOOKING_FOR_MASTER = 2,
    WAITING_FOR_GESTURE = 3, // waiting for pointing up gesture
    CHANGE_MASTER = 4,  
    STOP_FOLLOWING = 5, //unused
    FOLLOWING_MASTER = 6 //"waiting" for thumbs up or victory
};

class car_control : public rclcpp::Node
{

public:
    car_control()
        : Node("car_control_node")
    {
        start = false;
        mode = 0;
        requestedThrottle = 0;
        requestedSteerAngle = 0;

        publisher_can_ = this->create_publisher<interfaces::msg::MotorsOrder>("motors_order", 10);

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

        timer_ = this->create_wall_timer(PERIOD_UPDATE_CMD, std::bind(&car_control::updateCmd, this));

        RCLCPP_INFO(this->get_logger(), "car_control_node READY");
    }

private:
    void AICallback(const interfaces::msg::AI &AI)
    {
        AI_data = AI; //copy
    }

    void StateCallback(const interfaces::msg::State &State_controller)
    {
        RFID_state = State_controller.state;
    }

    void ultrasonicCallback(const interfaces::msg::Ultrasonic &ultrasonic)
    {
        us_front = ultrasonic.front_center;
        us_front_right = ultrasonic.front_right;
        us_front_left = ultrasonic.front_left;
        us_rear = ultrasonic.rear_center;
        us_rear_right = ultrasonic.rear_right;
        us_rear_left = ultrasonic.rear_left;
    }

    /* Update start, mode, requestedThrottle, requestedSteerAngle and reverse from joystick order [callback function]  :
     *
     * This function is called when a message is published on the "/joystick_order" topic
     */
    void joystickOrderCallback(const interfaces::msg::JoystickOrder &joyOrder)
    {

        if (joyOrder.start != start)
        {
            start = joyOrder.start;

            if (start)
                RCLCPP_INFO(this->get_logger(), "START");
            else
                RCLCPP_INFO(this->get_logger(), "STOP");
        }

        if (joyOrder.mode != mode && joyOrder.mode != -1)
        { // if mode change
            mode = joyOrder.mode;

            if (mode == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Switching to MANUAL Mode");
            }
            else if (mode == 1)
            {
                RCLCPP_INFO(this->get_logger(), "Switching to AUTONOMOUS Mode");
            }
            else if (mode == 2)
            {
                RCLCPP_INFO(this->get_logger(), "Switching to STEERING CALIBRATION Mode");
                startSteeringCalibration();
            }
        }

        if (mode == 0 && start)
        { // if manual mode -> update requestedThrottle, requestedSteerAngle and reverse from joystick order
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
    void motorsFeedbackCallback(const interfaces::msg::MotorsFeedback &motorsFeedback)
    {
        (void)motorsFeedback; // not used for now
        // currentAngle = motorsFeedback.steering_angle;
    }

    /* Update PWM commands : leftRearPwmCmd, rightRearPwmCmd, steeringPwmCmd
     *
     * This function is called periodically by the timer [see PERIOD_UPDATE_CMD in "car_control_node.h"]
     *
     * In MANUAL mode, the commands depends on :
     * - requestedThrottle, reverse, requestedSteerAngle [from joystick orders]
     * - currentAngle [from motors feedback]
     */
    void updateCmd()
    {

        publishingFollowingState();

        auto motorsOrder = interfaces::msg::MotorsOrder();

        if (!start) // Car stopped
            stop_car();
        else
        { // Car started

            // Run mode
            if (RFID_state == 1)
            {
                F_State new_follow_state{0};
                switch (follow_state)
                {
                case F_State::NONE:
                {
                    RCLCPP_INFO(this->get_logger(), "Lookinig for master");
                    new_follow_state = F_State::LOOKING_FOR_MASTER;
                    [[fallthrough]];
                }
                case F_State::LOOKING_FOR_MASTER:
                {
                    stop_car();
                    if (!AI_data.boxes.empty())
                    {
                        auto box = AI_data.boxes[0];
                        master_id = box.id;
                        is_following = false;
                        RCLCPP_INFO(this->get_logger(), "Waiting gesture");
                        new_follow_state = F_State::WAITING_FOR_GESTURE;
                    }
                    else
                    {
                        RCLCPP_INFO(this->get_logger(), "Looking for master");
                        new_follow_state = F_State::LOOKING_FOR_MASTER;
                    }
                    break;
                }
                case F_State::WAITING_FOR_GESTURE:
                {
                    bool master_found = false;
                    for (auto &&box : AI_data.boxes)
                    {
                        if (box.id == master_id)
                        {
                            master_found = true;
                            if (AI_data.gesture.id == master_id && AI_data.gesture.class_name == "Pointing_Up")
                            {
                                follow(box.x, box.w);
                                RCLCPP_INFO(this->get_logger(), "Following master");
                                new_follow_state = F_State::FOLLOWING_MASTER;
                            }
                            else if (AI_data.gesture.id == master_id && AI_data.gesture.class_name == "Victory")
                            {
                                stop_car();
                                ban_id = master_id;
                                master_id = -1;
                                RCLCPP_INFO(this->get_logger(), "Changing master");
                                new_follow_state = F_State::CHANGE_MASTER;
                            }
                            else
                            {
                                stop_car();
                            }
                            break;
                        }
                    }
                    if (!master_found)
                    {
                        stop_car();
                        RCLCPP_INFO(this->get_logger(), "Looking for master");
                        new_follow_state = F_State::LOOKING_FOR_MASTER;
                    }
                    break;
                }
                case F_State::FOLLOWING_MASTER:
                {
                    bool master_found = false;
                    for (auto &&box : AI_data.boxes)
                    {
                        if (box.id == master_id)
                        {
                            master_found = true;
                            if (AI_data.gesture.id == master_id && AI_data.gesture.class_name == "Thumb_Up")
                            {
                                stop_car();
                                new_follow_state = F_State::WAITING_FOR_GESTURE;
                            }
                            else
                            {
                                follow(box.x, box.w);
                            }
                            break;
                        }
                    }
                    if (!master_found)
                    {
                        new_follow_state = F_State::LOOKING_FOR_MASTER;
                    }
                    break;
                }
                case F_State::CHANGE_MASTER:
                {
                    stop_car();
                    for (auto &&box : AI_data.boxes)
                    {
                        if (box.id != ban_id)
                        {
                            master_id = box.id;
                            ban_id = -1;
                            new_follow_state = F_State::WAITING_FOR_GESTURE;
                            break;
                        }
                    }
                    break;
                }
                case F_State::MASTER_LOST:
                case F_State::STOP_FOLLOWING:
                default:
                {
                    RCLCPP_ERROR(this->get_logger(), "Unknown/unreachable state: %i!", static_cast<int>(follow_state));
                }
                }
                if(new_follow_state != F_State::NONE)
                    follow_state = new_follow_state;
            }
        }

        // Send order to motors
        motorsOrder.left_rear_pwm = leftRearPwmCmd;
        motorsOrder.right_rear_pwm = rightRearPwmCmd;

        motorsOrder.steering_angle = (int8_t)(requestedSteerAngle * 127.0); // Scale [-1,1] to [-127,+127]
        currentAngle = requestedSteerAngle;

        timer += 1;
        if (timer == 20)
        {
            publisher_can_->publish(motorsOrder);
            // RCLCPP_INFO(this->get_logger(), "Steering angle : %f ", requestedSteerAngle);
            // RCLCPP_INFO(this->get_logger(), "Pos x AI : %d ", ai_x);
            timer = 0;
        }
    }

    bool
    ai_data_has_master(int master_id)
    {
        for (auto &&box : AI_data.boxes)
        {
            if (box.id == master_id)
                return true;
        }
        return false;
    }

    /* Start the steering calibration process :
     *
     * Publish a calibration request on the "/steering_calibration" topic
     */
    void startSteeringCalibration()
    {

        auto calibrationMsg = interfaces::msg::SteeringCalibration();
        calibrationMsg.request = true;

        RCLCPP_INFO(this->get_logger(), "Sending calibration request .....");
        publisher_steeringCalibration_->publish(calibrationMsg);
    }

    /* Start publishing the following states :
     *
     * Publish the state of the following of the car on the topic following_state_event
     */
    void publishingFollowingState()
    {

        auto msg = interfaces::msg::FollowingStateEvent();

        msg.state = static_cast<unsigned char>(follow_state);

        // Only publish when a change occurs to limit subscriber's work
        if (previous_following_state != msg.state)
        {
            RCLCPP_INFO(this->get_logger(), "Following state event: %i", msg.state);
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

        mode = 2; // Switch to calibration mode
        RCLCPP_WARN(this->get_logger(), "Switching to STEERING CALIBRATION Mode");
        startSteeringCalibration();
    }

    void stop_car()
    {
        requestedSteerAngle = 0;
        leftRearPwmCmd = 50;
        rightRearPwmCmd = 50;
    }

    void follow(int master_x, int master_w)
    {
        leftRearPwmCmd = 80;
        rightRearPwmCmd = 80;
        requestedThrottle = 0.8;
        past_steering_angle = requestedSteerAngle;
        requestedSteerAngle = suivreHumain(master_x, master_w, past_steering_angle);
        if ((requestedSteerAngle - past_steering_angle) > 0.1)
        {
            requestedSteerAngle = past_steering_angle + 0.1;
        }
        else if ((requestedSteerAngle - past_steering_angle) < -0.1)
        {
            requestedSteerAngle = past_steering_angle - 0.1;
        }
        // On refait une saturation
        if (requestedSteerAngle > 1.0)
        {
            requestedSteerAngle = 1.0;
        }
        else if (requestedSteerAngle < -1.0)
        {
            requestedSteerAngle = -1.0;
        }
        obstacleDetection1(requestedThrottle, reverse, leftRearPwmCmd, rightRearPwmCmd, us_front, us_front_right, us_front_left, us_rear, us_rear_right, us_rear_left);
    }

    /* Manage steering calibration process [callback function]  :
     *
     * This function is called when a message is published on the "/steering_calibration" topic
     */
    void steeringCalibrationCallback(const interfaces::msg::SteeringCalibration &calibrationMsg)
    {

        if (calibrationMsg.in_progress == true && calibrationMsg.user_need == false)
        {
            RCLCPP_INFO(this->get_logger(), "Steering Calibration in progress, please wait ....");
        }
        else if (calibrationMsg.in_progress == true && calibrationMsg.user_need == true)
        {
            RCLCPP_WARN(this->get_logger(), "Please use the buttons (L/R) to center the steering wheels.\nThen, press the blue button on the NucleoF103 to continue");
        }
        else if (calibrationMsg.status == 1)
        {
            RCLCPP_INFO(this->get_logger(), "Steering calibration [SUCCESS]");
            RCLCPP_INFO(this->get_logger(), "Switching to MANUAL Mode");
            mode = 0;      // Switch to manual mode
            start = false; // Stop car
        }
        else if (calibrationMsg.status == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Steering calibration [FAILED]");
            RCLCPP_INFO(this->get_logger(), "Switching to MANUAL Mode");
            mode = 0;      // Switch to manual mode
            start = false; // Stop car
        }
    }

    // ---- Private variables ----

    // General variables
    bool start;
    int mode; // 0 : Manual    1 : Auto    2 : Calibration
    int8_t previous_following_state = -10;

    // Motors feedback variables
    float currentAngle;
    float past_steering_angle = 0.0;

    // Manual Mode variables (with joystick control)
    bool reverse;
    float requestedThrottle;
    float requestedSteerAngle;

    // FSM
    int RFID_state = 0;
    bool is_following = false; // false -> No follow, true -> Follow master
    int master_id = -1;        // ID de la personne suivie
    F_State follow_state = F_State::NONE;
    int ban_id = -1;
    // int follow_state = 1;          // 1 -> Looking for a master, 2 -> Master found, waiting for gestures, 3 -> Following master
    // F_State following_state_event = F_State::None;

    // Control variables
    uint8_t leftRearPwmCmd;
    uint8_t rightRearPwmCmd;
    uint8_t steeringPwmCmd;

    // Publishers
    rclcpp::Publisher<interfaces::msg::MotorsOrder>::SharedPtr publisher_can_;
    rclcpp::Publisher<interfaces::msg::SteeringCalibration>::SharedPtr publisher_steeringCalibration_;
    rclcpp::Publisher<interfaces::msg::FollowingStateEvent>::SharedPtr publisher_followingStateEvent_;

    // Subscribers
    rclcpp::Subscription<interfaces::msg::JoystickOrder>::SharedPtr subscription_joystick_order_;
    rclcpp::Subscription<interfaces::msg::MotorsFeedback>::SharedPtr subscription_motors_feedback_;
    rclcpp::Subscription<interfaces::msg::SteeringCalibration>::SharedPtr subscription_steering_calibration_;
    rclcpp::Subscription<interfaces::msg::Ultrasonic>::SharedPtr subscription_us_data_;
    rclcpp::Subscription<interfaces::msg::AI>::SharedPtr subscription_AI;
    rclcpp::Subscription<interfaces::msg::State>::SharedPtr subscription_State;
    interfaces::msg::AI AI_data{}; // buffered last AI data

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Steering calibration Service
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr server_calibration_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<car_control>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
