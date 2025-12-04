import rclpy
from rclpy.node import Node
import socket
import pickle
import threading
import time

from interfaces.msg import AI, BBox, Gesture

from .ai_data_class import AI_Data, SOCKET_PORT, MAX_MSG_LEN

class TcpReceiverNode(Node):
    def __init__(self):
        super().__init__('tcp_receiver_node')
        
        # Publisher
        self.publisher_ = self.create_publisher(IA, 'ai_perception_data', 10)
        
        # TCP Configuration
        self.host = 'localhost'
        self.port = SOCKET_PORT
        self.running = True

        self.get_logger().info(f'Starting TCP Receiver on {self.host}:{self.port}')

        # Start TCP server in a separate thread to avoid blocking ROS
        self.server_thread = threading.Thread(target=self.tcp_server_loop)
        self.server_thread.daemon = True # Kills thread if main program exits
        self.server_thread.start()

    def tcp_server_loop(self):
        """
        Handles the socket connection and data reception.
        """
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                s.bind((self.host, self.port))
                s.listen(1)
            except Exception as e:
                self.get_logger().error(f'Failed to bind socket: {e}')
                return

            self.get_logger().info("Socket listening...")
            
            while self.running and rclpy.ok():
                try:
                    # accept() is blocking, so we put a timeout or just wait
                    # If we need clean shutdown, we might use settimeout on s
                    conn, addr = s.accept()
                    with conn:
                        self.get_logger().info(f'Connected by {addr}')
                        self.handle_client(conn)
                        self.get_logger().info('Client disconnected, listening again...')
                except OSError:
                    break

    def handle_client(self, conn):
        """
        Loops receiving data from a connected client.
        """
        while self.running and rclpy.ok():
            try:
                data = conn.recv(MAX_MSG_LEN)
                if not data:
                    break # Connection closed by client
                
                # Deserialization
                try:
                    received_obj = pickle.loads(data)
                    self.process_and_publish(received_obj)
                except pickle.UnpicklingError:
                    self.get_logger().warn("Failed to unpickle data. Stream might be corrupted.")
                except Exception as e:
                    self.get_logger().error(f"Error processing data: {e}")

            except ConnectionResetError:
                break

    def process_and_publish(self, python_obj):
        """
        Maps the Python object to the ROS 2 message and publishes it.
        """
        msg = IA()
        
        # --- MAPPING LOGIC START ---
        if hasattr(python_obj, 'gesture_data'):
            msg.gesture.class_name = python_obj.gesture_data.get('name', 'none')
            msg.gesture.id = int(python_obj.gesture_data.get('id', -1))
            msg.gesture.probability = float(python_obj.gesture_data.get('probability', 0.0))

        # Mapping for Boxes:
        temp_bbox_list = []
        if hasattr(python_obj, 'detections'):
            for i, det in enumerate(python_obj.detections):
                # if i >= 16: break # Safety cap                
                bbox = BBox()
                bbox.x = int(det.get('x', 0))
                bbox.y = int(det.get('y', 0))
                bbox.w = int(det.get('w', 0))
                bbox.h = int(det.get('h', 0))
                bbox.id = int(det.get('id', -1))
                bbox.probability = float(det.get('probability', 0.0))
                bbox.estimated_distance = float(det.get('estimated_distance', 0.0))
                bbox.class_name = str(det.get('class_name', 'unknown'))
                
                temp_bbox_list.append(bbox)
        msg.boxes = temp_bbox_list
        # --- MAPPING LOGIC END ---

        self.publisher_.publish(msg)
        self.get_logger().debug(f'Published IA message with {len(temp_bbox_list)} boxes')

def main(args=None):
    rclpy.init(args=args)
    node = TcpReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
