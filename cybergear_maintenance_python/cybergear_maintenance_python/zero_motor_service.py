from cybergear_driver_msgs.srv import ZeroMotors

from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node
import socket
import threading
import struct
import time

class MinimalService(Node):

    def __init__(self):
        super().__init__('zero_motor_service')
        self.srv = self.create_service(ZeroMotors, 'zero_motors', self.service_callback)

        self.can_interfaces = []

        self.read_config()

        #self.receive_can_messages(interface_name='can0', callback=self.zero_motor_callback)
        #self.receive_can_messages(interface_name='can1', callback=self.zero_motor_callback)
        #self.receive_can_messages(interface_name='can2', callback=self.zero_motor_callback)
        #self.receive_can_messages(interface_name='can3', callback=self.zero_motor_callback)

    def read_config(self):
        """
        Read configuration parameters from ROS2 parameter server
        """
        # Declare parameters with default values
        self.declare_parameter('can_interfaces', ['can0', 'can1', 'can2', 'can3'])
        self.declare_parameter('zero_command_id', 0x7FF)
        self.declare_parameter('timeout_seconds', 5.0)

        # Get the CAN interfaces list
        can_interfaces = self.get_parameter('can_interfaces').value
        self.zero_command_id = self.get_parameter('zero_command_id').value
        self.timeout_seconds = self.get_parameter('timeout_seconds').value

        # For each interface, declare a parameter for motor IDs
        for interface in can_interfaces:
            # Declare parameter for motor IDs on this interface
            self.declare_parameter(f'{interface}.motor_ids', [1, 2, 3, 4])
            
            # Get the motor IDs for this interface
            motor_ids = self.get_parameter(f'{interface}.motor_ids').value
            self.can_interfaces.append((interface, motor_ids))
            
            self.get_logger().info(f'  {interface}: {motor_ids}')

        # Log the configuration
        self.get_logger().info(f'Loaded configuration:')
        self.get_logger().info(f'  CAN interfaces: {self.can_interfaces}')
        self.get_logger().info(f'  Zero command ID: {hex(self.zero_command_id)}')
        self.get_logger().info(f'  Timeout: {self.timeout_seconds} seconds')
        
        # Update CAN listeners based on config
        for interface_name, motor_ids in self.can_interfaces:
            if not hasattr(self, f'{interface_name}_thread'):
                self.receive_can_messages(interface_name=interface_name, callback=self.zero_motor_callback)

    def service_callback(self, request, response):
        self.get_logger().info(f"Received request: {request}")
        
        # Process the request and prepare the response
        response.success = True
        return response

    def zero_motor_callback(self, can_response : bytes):
        
        #print out full response
        self.get_logger().info(f"Received response: {can_response.hex()}")
        can_id, can_dlc = struct.unpack("=IB3x", can_response[:8])
        data = list(struct.unpack("8B", can_response[8:16]))[:can_dlc]

        self.get_logger().info(f"CAN ID: {hex(can_id)}, DLC: {can_dlc}, Data: {data}")
        


    def send_frame_direct(self, interface_name, can_id):
        """
        Send a CAN frame directly to a CAN interface without using ROS bridges

        Args:
            packet: CybergearPacket instance
            interface_name (str): CAN interface name (e.g., 'can0')
            can_id (int): CAN ID for the message
            data (list): Data bytes to send (up to 8 bytes)
        """

        data = [0x00] * 8  # Default data, can be modified as needed

        try:
            # Create and bind socket
            s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            s.bind((interface_name,))

            # Prepare data - ensure it's exactly 8 bytes
            can_dlc = min(len(data), 8)
            data_padded = data[:8] + [0] * (8 - len(data[:8]))

            # CAN frame format: <CAN_ID><DLC><DATA[8]>
            # Use proper struct packing for CAN frame
            can_frame_format = "=IB3x8B"  # ID(4) + DLC(1) + padding(3) + data(8)

            can_frame = struct.pack(can_frame_format, 
                                   can_id,           # CAN ID
                                   can_dlc,          # Data length
                                   *data_padded)     # 8 bytes of data

            # Send the frame
            #bytes_sent = s.send(can_frame)

        except socket.error as e:
            print(f"Socket error: {e}")
            # Check if CAN interface exists
            import subprocess
            try:
                result = subprocess.run(['ip', 'link', 'show', interface_name], 
                                      capture_output=True, text=True)
                if result.returncode != 0:
                    print(f"CAN interface '{interface_name}' may not exist or be up")
                    print("Try: sudo ip link set can0 up type can bitrate 1000000")
            except:
                pass
        except Exception as e:
            print(f"Unexpected error: {e}")
        finally:
            if 's' in locals():
                s.close()
    

    def receive_can_messages(self,interface_name, callback):
        """
        Listen for CAN messages on the given interface and call the callback with (can_id, data) when a frame is received.
        """
        def listener():
            try:
                s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                s.bind((interface_name,))
                while True:
                    frame = s.recv(16)
                    self.zero_motor_callback(frame)
            except Exception as e:
                print(f"Error in CAN listener: {e}")
            finally:
                if 's' in locals():
                    s.close()

        thread = threading.Thread(target=listener, daemon=True)
        thread.start()

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()