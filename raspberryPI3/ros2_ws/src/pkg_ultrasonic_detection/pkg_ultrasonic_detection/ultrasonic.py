
import rclpy
from rclpy.node import Node

from interfaces.msg import Ultrasonic, JoystickOrder


class UltrasonicSubscriber(Node):

    def __init__(self):
        super().__init__('ultrasonic_subscriber')

        self.subscription = self.create_subscription(
            Ultrasonic,
            '/us_data',
            self.listener_callback,
            10
        )
        self.subscription

        # Publisher  motors
        self.publisher_ = self.create_publisher(JoystickOrder, 'joystick_order', 10)

    def listener_callback(self, msg):
        self.get_logger().info(
            f"""
            Données Ultrasonic :
            front_left   : {msg.front_left}
            front_center : {msg.front_center}
            front_right  : {msg.front_right}
            rear_left    : {msg.rear_left}
            rear_center  : {msg.rear_center}
            rear_right   : {msg.rear_right}
            """
        )

        motor_msg = JoystickOrder()
        motor_msg.start = True
        motor_msg.mode = 0
        motor_msg.reverse = False

        front_obstacle = min(msg.front_left, msg.front_center, msg.front_right)
        rear_obstacle = min(msg.rear_left, msg.rear_center, msg.rear_right)

        # Stop condition and accelerate control 
        if front_obstacle <= 30:
            self.get_logger().warn("Obstacle detected, stop the car")
            motor_msg.steer = 0.0
            motor_msg.throttle = 0.0
        else:
            if front_obstacle <= 80:
               motor_msg.throttle = (front_obstacle - 30) / 50.0
           # else:
           #    motor_msg.throttle = 1.0

        self.publisher_.publish(motor_msg)
        self.get_logger().info(f'Movement Sequence Node published : mode={motor_msg.mode}, throttle={motor_msg.throttle}, steer={motor_msg.steer}, reverse={motor_msg.reverse}')


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

