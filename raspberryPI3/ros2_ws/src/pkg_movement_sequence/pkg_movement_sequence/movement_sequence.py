import rclpy
from rclpy.node import Node
from interfaces.msg import JoystickOrder

class MovementSequence(Node):

    def __init__(self):
        super().__init__('movement_sequence')
        self.publisher_ = self.create_publisher(JoystickOrder, 'joystick_order', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = JoystickOrder()
        sequence_steer = [-1.0, -0.5, 0.0, 0.5, 1.0]
        msg.start = True
        msg.mode = 0
        msg.throttle = 0.0
        msg.steer = sequence_steer[self.i]
        msg.reverse = False
        self.publisher_.publish(msg)
        self.get_logger().info(f'Movement Sequence Node published : mode={msg.mode}, throttle={msg.throttle}, steer={msg.steer}, reverse={msg.reverse}')
        self.i += 1
        if self.i == 4:
           self.i = 0


def main(args=None):
    rclpy.init(args=args)

    movement_sequence = MovementSequence()
    rclpy.spin(movement_sequence)

    # Destroy the node explicitly
    movement_sequence.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
