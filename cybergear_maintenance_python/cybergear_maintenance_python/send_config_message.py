#!/usr/bin/env python3
# filepath: cybergear_config/cybergear_config/cybergear_config_node.py
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from can_msgs.msg import Frame
from cybergear_maintenance_python.cybergear_packet import CybergearPacket

class CybergearConfigNode(Node):
    def __init__(self):
        super().__init__('cybergear_config')
        
        # Declare parameters
        self.declare_parameter('primary_id', 0)
        self.declare_parameter('device_id', 127)
        self.declare_parameter('target_id', 126)
        self.declare_parameter('send_frequency', 10.0)
        self.declare_parameter('wait_receive_can_frame', 2.0)
        self.declare_parameter('operation', 'change_id')
        
        # Get parameters
        self.primary_id = self.get_parameter('primary_id').value
        self.device_id = self.get_parameter('device_id').value
        self.target_id = self.get_parameter('target_id').value
        self.send_frequency = self.get_parameter('send_frequency').value
        self.wait_time = self.get_parameter('wait_receive_can_frame').value
        self.operation = self.get_parameter('operation').value
        
        # Log parameters
        self.get_logger().info(f"Primary ID: {self.primary_id}")
        self.get_logger().info(f"Device ID: {self.device_id}")
        self.get_logger().info(f"Target ID: {self.target_id}")
        self.get_logger().info(f"Operation: {self.operation}")
        
        # Initialize packet helper
        self.packet = CybergearPacket(self.primary_id, self.device_id)
        
        # Initialize state
        self.counter = 0
        self.start_time = time.time()
        self.initialization_delay = 1.0  # 1 second delay for CAN bus init
        self.found_devices = set()
        
        # Create publisher and subscriber with reliable QoS
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(Frame, 'to_can_bus', qos)
        self.subscription = self.create_subscription(
            Frame, 
            'from_can_bus', 
            self.can_frame_callback, 
            qos
        )
        
        # Create timer
        period = 1.0 / self.send_frequency
        self.timer = self.create_timer(period, self.timer_callback)
        
        self.get_logger().info("Node initialized, waiting for CAN bus to be ready...")

    def can_frame_callback(self, msg):
        """Process incoming CAN frames"""
        try:
            device_id = self.packet.get_frame_id(msg.id)
            
            # Skip our own frames
            if device_id == self.primary_id:
                return
                
            # Check if it's an info frame
            if self.packet.is_info_frame(msg.id):
                if device_id not in self.found_devices:
                    self.found_devices.add(device_id)
                    self.get_logger().info(f"Found CyberGear device: {device_id}")
        except Exception as e:
            self.get_logger().error(f"Error processing CAN frame: {e}")

    def timer_callback(self):
        """Send CAN frames based on current statemain"""
        elapsed = time.time() - self.start_time
        
        # Log progress
        self.get_logger().info(f"Timer callback: {self.counter}, elapsed: {elapsed:.2f}s")
        
        # Wait for CAN bus initialization
        if elapsed < self.initialization_delay:
            self.get_logger().info("Waiting for CAN bus initialization...")
            self.counter += 1
            return
            
        # Send commands for a limited number of attempts
        if self.counter < 10:
            if self.operation == 'change_id':
                self.send_change_id_command()
            elif self.operation == 'search':
                self.send_search_command()
            else:
                self.get_logger().error(f"Unknown operation: {self.operation}")
        else:
            # Check if we should shut down
            if elapsed > (self.initialization_delay + self.wait_time):
                self.get_logger().info("Operation complete, shutting down...")
                rclpy.shutdown()
                
        self.counter += 1
        
    def send_change_id_command(self):
        """Send command to change motor ID"""
        self.get_logger().info(f"Sending change ID command: {self.device_id} -> {self.target_id}")
        
        msg = Frame()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.is_rtr = False
        msg.is_extended = True
        msg.is_error = False
        msg.dlc = 8
        msg.data = [0, 0, 0, 0, 0, 0, 0, 1]  # Command data for ID change
        msg.id = self.packet.get_change_device_id_frame(self.target_id)
        
        self.publisher.publish(msg)
        
    def send_search_command(self):
        """Send command to search for motors"""
        search_id = self.counter % 256  # Cycle through all possible IDs
        self.get_logger().info(f"Searching for device with ID: {search_id}")
        
        msg = Frame()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.is_rtr = False
        msg.is_extended = True
        msg.is_error = False
        msg.dlc = 8
        msg.data = [0, 0, 0, 0, 0, 0, 0, 0]  # Command data for read info
        msg.id = self.packet.get_read_info_frame(search_id)
        
        self.publisher.publish(msg)

def change_id(args=None):
    rclpy.init(args=args)
    node = CybergearConfigNode()
    node.send_change_id_command()
    rclpy.spin(node)
    rclpy.shutdown()
