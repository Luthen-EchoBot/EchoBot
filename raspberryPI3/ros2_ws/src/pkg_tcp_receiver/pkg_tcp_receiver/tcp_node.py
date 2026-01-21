import rclpy
from rclpy.node import Node
import socket
import pickle
import threading
import time
import websockets
import asyncio

from interfaces.msg import AI, BBox, Gesture

import sys

################# EXPRIMENTAL ##############
# 1. Import the class locally from your package
from . import ai_data_class 

# 2. Force Python to recognize 'ai_data_class' as a top-level module
# This tricks pickle into finding the class definition where the sender expects it.
sys.modules['ai_data_class'] = ai_data_class
############### END OF EXPERIMENTAL ########

from .ai_data_class import AI_Data, SOCKET_PORT, MAX_MSG_LEN

class TcpReceiverNode(Node):
    def __init__(self):
        super().__init__('tcp_receiver_node')
        
        # Publisher
        self.publisher_ = self.create_publisher(AI, 'ai_perception_data', 10)

        self.last_msg_sent = AI()
        
        # TCP Configuration
        self.host = '192.168.1.1'
        self.port = SOCKET_PORT
        self.running = True

        self.get_logger().info(f'Starting TCP Receiver on {self.host}:{self.port}')

        # Start TCP server in a separate thread to avoid blocking ROS
        self.server_thread = threading.Thread(target=self.tcp_server_loop)
        self.server_thread.daemon = True # Kills thread if main program exits
        self.server_thread.start()

        self.websocket_thread = threading.Thread(target=lambda:asyncio.run(self.websocket_loop()))
        self.websocket_thread.daemon = True
        self.websocket_thread.start()

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

    async def websocket_loop(self):
        num_errors = 0
        while num_errors<10:
            try:
                async with websockets.connect("wss://magictintin.fr/ws") as websocket:
                    await websocket.send("luthen/main:ping")
                    await websocket.send("micasend:ping")
                    while True:
                        response = await websocket.recv()
                        self.get_logger().debug(f"Response: {response}")
                        if response == "👍":
                            self.last_msg_sent.gesture.class_name = "Thumb_Up"
                            self.last_msg_sent.gesture.probability = 100.0
                            self.publisher_.publish(self.last_msg_sent)
                            self.get_logger().warn(f"Thumb up sent: {self.last_msg_sent}")
            except Exception as e:
                num_errors+=1
                self.get_logger().debug(f"Error({num_errors}/10):",e)

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

    def process_and_publish(self, ai_data_obj):
        """
        Maps the Python object to the ROS 2 message and publishes it.
        """
        msg = AI()
        
        # --- MAPPING LOGIC START ---
        if hasattr(ai_data_obj, 'gesture_data') and ai_data_obj.gesture_data is not None:
            msg.gesture.class_name = str(ai_data_obj.gesture_data.class_name)
            msg.gesture.id = int(ai_data_obj.gesture_data.id)
            msg.gesture.probability = float(ai_data_obj.gesture_data.probability)
        else:
            # Default values if no gesture data
            msg.gesture.class_name = "none"
            msg.gesture.id = -1
            msg.gesture.probability = 0.0

        # Mapping for Boxes: 
        temp_bbox_list = []
        if hasattr(ai_data_obj, 'detections'):
            for i, det in enumerate(ai_data_obj.detections):
                # if i >= 16: break # Safety cap                
                bbox = BBox()
                bbox.x = int(det.x)
                bbox.y = int(det.y)
                bbox.w = int(det.w)
                bbox.h = int(det.h)
                bbox.id = int(det.id)
                bbox.probability = float(det.probability)
                bbox.estimated_distance = float(det.estimated_distance)
                bbox.class_name = str(det.class_name)
                
                temp_bbox_list.append(bbox)
        msg.boxes = temp_bbox_list
        # --- MAPPING LOGIC END ---

        self.last_msg_sent = msg
        self.publisher_.publish(msg)
        self.get_logger().debug(f'Published AI message with {len(temp_bbox_list)} boxes')

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
