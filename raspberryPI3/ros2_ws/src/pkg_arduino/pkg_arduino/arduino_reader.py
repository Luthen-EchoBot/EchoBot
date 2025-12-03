import rclpy
from rclpy.node import Node
from interfaces.msg import ArduinoData
import serial

class ArduinoReader(Node):

    def __init__(self):
        super().__init__('arduino_reader')
        self.publisher_ = self.create_publisher(ArduinoData, 'arduino_data', 10)
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        self.get_logger().info("Arduino Reader | Running")

        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        ligne = self.ser.readline().decode().strip()

        if not ligne:
            return

        try:
            parts = ligne.split("/")
            if len(parts) != 3:
                self.get_logger().warn(f"Message ignoré : format incorrect -> '{ligne}'")
                return

            rfid_state_str, temp_str, humidity_str = parts

            if not rfid_state_str or not temp_str or not humidity_str:
                self.get_logger().warn(f"Message ignoré : champs vides -> '{ligne}'")
                return

            # Conversion en int (robuste)
            rfid_state = int(rfid_state_str)
            temp = int(temp_str)
            humidity = int(humidity_str)

        except ValueError:
            # Arrive si un champ ne peut pas être converti en int
            self.get_logger().warn(f"Message ignoré : non convertible en int -> '{ligne}'")
            return

        except Exception as e:
            self.get_logger().warn(f"Erreur lors de l'analyse du message : {e}")
            return

        # Création du message ROS
        msg = ArduinoData()
        msg.rfid = rfid_state
        msg.temp = temp
        msg.humidity = humidity

        self.publisher_.publish(msg)
        self.get_logger().info(
            f"Arduino Reader | RFID : {msg.rfid} | Temperature : {msg.temp} | Humidity : {msg.humidity}"
        )



def main(args=None):
    rclpy.init(args=args)

    arduino_reader = ArduinoReader()
    rclpy.spin(arduino_reader)

    # Destroy the node explicitly
    arduino_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
