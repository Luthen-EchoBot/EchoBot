import rclpy
from rclpy.node import Node
from interfaces.msg import AI
import serial
import time

class ArduinoSender(Node):

    def __init__(self):
        super().__init__('camera_rotation_controller')

        # --- Configuration ---
        self.declare_parameter('serial_port', '/dev/ttyACM1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('ai_topic_name', 'ai_perception_data')

        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        topic_name = self.get_parameter('ai_topic_name').get_parameter_value().string_value

        # --- Subscriber ---
        # On écoute les données de l'IA
        self.subscription = self.create_subscription(
            AI,
            topic_name,
            self.ai_callback,
            10
        )
        self.get_logger().info(f"Arduino Sender | Abonné au topic : {topic_name}")

        # --- Gestion Série ---
        self.ser = None
        self.connect_serial() # Tentative de connexion au démarrage

    def connect_serial(self):
        """Tente d'établir ou de rétablir la connexion série."""
        if self.ser and self.ser.is_open:
            return True # Déjà connecté

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Arduino Sender | Connexion établie sur {self.serial_port}")
            return True
        except serial.SerialException:
            self.get_logger().warn(f"Failed to connect")
            # On ne spamme pas les logs ici, on gérera l'erreur lors de l'envoi
            return False

    def close_serial(self):
        """Ferme proprement la connexion."""
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.get_logger().warn("Arduino Sender | Port série fermé.")

    def send_to_arduino(self, value):
        """Envoie la donnée formatée à l'Arduino : %valeur%"""
        # 1. Vérification connexion
        if not self.ser or not self.ser.is_open:
            if not self.connect_serial():
                self.get_logger().warn(f"Arduino non connecté. Impossible d'envoyer : {value}", throttle_duration_sec=5.0)
                return

        # 2. Formatage et Envoi
        try:
            # Format demandé : %valeur%
            message = f"%{value}%\n"
            
            # Envoi (encodage en bytes nécessaire)
            self.ser.write(message.encode('utf-8'))
            
            # Optionnel : log pour debug
            self.get_logger().warn(f"Envoyé à l'Arduino : {message}")

        except (serial.SerialException, OSError):
            self.get_logger().error("Arduino Sender | Erreur lors de l'envoi (Câble débranché ?)")
            self.close_serial()

    def processing_logic(self, ai_x, ai_w):
        screen_width = 640.0 
        max_angle = 360.0
        obj_center_x = ai_x + (ai_w / 2.0)
        angle = (obj_center_x / screen_width) * max_angle

        if angle < 0:
            angle = 0
        elif angle > max_angle:
            angle = max_angle
            
        return int(angle)

    def ai_callback(self, msg):
        """
        Callback déclenché à chaque réception de message IA.
        Traduction de ta logique C++ en Python.
        """
        
        # 1. Vérification si vide (équivalent à AI.boxes.empty())
        if not msg.boxes:
            self.get_logger().info("Aucun humain détecter", throttle_duration_sec=2.0)
            return

        # 2. Extraction des données (On prend l'index 0 comme dans ton exemple 'int i = 0')
        i = 0
        current_box = msg.boxes[i]

        # Variables extraites
        ai_x = current_box.x
        ai_w = current_box.w

        valeur_finale = self.processing_logic(ai_x,ai_w)
        # self.get_logger().warn(f"Angle servomoteur : {valeur_finale}", throttle_duration_sec=5.0)
        
        ## TEMPORARY!!
        commande = [90,180]
        time_scale = 5.0
        ti = int(time.time()/time_scale) % len(commande)
        valeur_finale = commande[ti]
        # 4. Envoi à l'Arduino
        self.send_to_arduino(valeur_finale)


def main(args=None):
    rclpy.init(args=args)

    node = ArduinoSender()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_serial()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
