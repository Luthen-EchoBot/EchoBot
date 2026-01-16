import rclpy
from rclpy.node import Node
from interfaces.msg import ArduinoData
import serial
import time

class ArduinoReader(Node):

    def __init__(self):
        super().__init__('arduino_reader')
        self.publisher_ = self.create_publisher(ArduinoData, 'arduino_data', 10)
        
        # On n'ouvre pas la connexion ici, on initialise juste la variable à None
        self.ser = None
        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 115200

        self.get_logger().info("Arduino Reader | Initialized (Waiting for connection...)")

        # Timer pour la lecture ET la gestion de connexion
        self.timer = self.create_timer(0.5, self.timer_callback)

    def connect_serial(self):
        """Tente d'établir la connexion série."""
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Arduino Reader | Connexion établie sur {self.serial_port}")
            return True
        except serial.SerialException as e:
            # On log en warning pour ne pas spammer 'error' si c'est juste débranché
            self.get_logger().warn(f"Arduino Reader | Impossible de se connecter à {self.serial_port}. En attente...", throttle_duration_sec=5.0)
            return False

    def close_serial(self):
        """Ferme proprement la connexion."""
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.get_logger().warn("Arduino Reader | Connexion perdue. Tentative de reconnexion...")

    def timer_callback(self):
        # 1. Gestion de la connexion
        if self.ser is None:
            if not self.connect_serial():
                return  # Si échec, on attend le prochain tick du timer

        # 2. Lecture des données
        try:
            # readline peut lever une exception si le câble est arraché pendant la lecture
            if self.ser.in_waiting > 0:
                line_bytes = self.ser.readline()
            else:
                return # Rien à lire pour l'instant
            
            ligne = line_bytes.decode('utf-8', errors='ignore').strip()

        except (serial.SerialException, OSError, Exception) as e:
            self.get_logger().error(f"Arduino Reader | Erreur de lecture (Déconnexion probable) : {e}")
            self.close_serial()
            return

        if not ligne:
            return

        # 3. Parsing (Votre logique existante)
        try:
            parts = ligne.split("/")
            if len(parts) != 3:
                # Log en debug pour éviter le spam si l'arduino démarre et envoie des données partielles
                self.get_logger().debug(f"Message ignoré : format incorrect -> '{ligne}'") 
                return

            rfid_state_str, temp_str, humidity_str = parts

            if not rfid_state_str or not temp_str or not humidity_str:
                return

            # Conversion en int (robuste)
            rfid_state = int(rfid_state_str)
            temp = int(temp_str)
            humidity = int(humidity_str)

            # Création du message ROS
            msg = ArduinoData()
            msg.rfid = rfid_state
            msg.temp = temp
            msg.humidity = humidity

            self.publisher_.publish(msg)
            # self.get_logger().info(f"Arduino Reader | RFID : {msg.rfid} | Temp : {msg.temp} | Hum : {msg.humidity}")

        except ValueError:
            self.get_logger().warn(f"Message ignoré : non convertible en int -> '{ligne}'")
        except Exception as e:
            self.get_logger().warn(f"Erreur logique lors de l'analyse : {e}")

def main(args=None):
    rclpy.init(args=args)

    arduino_reader = ArduinoReader()
    
    try:
        rclpy.spin(arduino_reader)
    except KeyboardInterrupt:
        pass
    finally:
        # Fermeture propre à l'arrêt du noeud
        arduino_reader.close_serial()
        arduino_reader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
