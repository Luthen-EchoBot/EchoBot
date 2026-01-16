import rclpy
from rclpy.node import Node

from interfaces.msg import ArduinoData
from interfaces.msg import State 

class StateControl(Node):

    def __init__(self):
        super().__init__('state_control')
        self.publisher_ = self.create_publisher(State, 'state', 10)

        self.subscription = self.create_subscription(ArduinoData,'arduino_data',self.listener_callback,10)
        self.subscription # ??

        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.publisher_callback)
        self.i = 0

        self.state = 0
        self.state = "STOP MODE" 
        self.RUN_MODE = "RUN MODE"
        self.STOP_MODE = "STOP MODE"

    def publisher_callback(self):
        msg = State()
        if self.state == "RUN MODE":
            msg.state = 1
        else:
            msg.state = 0
        self.publisher_.publish(msg)
        # if (self.state == self.RUN_MODE):
        #     self.get_logger().info("State controller | RUN MODE")
        # else:
        #     self.get_logger().warn("State controller | STOP MODE")
        self.i += 1

    def listener_callback(self,data):
        rfid_status = data.rfid
        # print(f"État RFID reçu : {rfid_status}")
        # print(f"État actuel : {self.state}")

        if rfid_status == 0: # no badge
            pass
        elif rfid_status == 2: # wrong badge
            self.get_logger().warn(f"Wrong RFID bagde detected")
        elif rfid_status == 1: # right badge => changing state
            if self.state == self.STOP_MODE:
                self.state = self.RUN_MODE
            elif self.state == self.RUN_MODE:
                self.state = self.STOP_MODE
            self.get_logger().info(f"Changing state: {self.state}")
        else:
            self.get_logger().warn(f"Statut RFID inconnu: {rfid_status}!")

def main(args=None):
    rclpy.init(args=args)

    state_controler = StateControl()

    rclpy.spin(state_controler)

    state_controler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
