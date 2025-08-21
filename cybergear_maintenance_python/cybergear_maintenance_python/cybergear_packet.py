#!/usr/bin/env python3

import enum
from dataclasses import dataclass

class CommandId(enum.IntEnum):
    """Command IDs for CyberGear motors"""
    READ_PID_DATA = 0x30
    WRITE_PID_DATA = 0x31
    READ_ACCELERATION = 0x33
    WRITE_ACCELERATION = 0x34
    READ_ENCODER = 0x90
    WRITE_ENCODER_OFFSET = 0x91
    READ_MULTI_TURNS_ANGLE = 0x92
    READ_SINGLE_CIRCLE_ANGLE = 0x94
    CLEAR_ERROR_FLAG = 0x9B
    READ_MOTOR_STATUS_1 = 0x9A
    READ_MOTOR_STATUS_2 = 0x9C
    READ_MOTOR_STATUS_3 = 0x9D
    MOTOR_OFF = 0x80
    MOTOR_STOP = 0x81
    MOTOR_RUNNING = 0x88
    SET_TORQUE_CURRENT = 0xA1
    SET_SPEED = 0xA2
    SET_POSITION_3 = 0xA3
    SET_POSITION_4 = 0xA4
    SET_POSITION_5 = 0xA5
    SET_POSITION_6 = 0xA6
    SET_POSITION_7 = 0xA7
    CHANGE_ID = 0x07
    READ_INFO = 0x01
    SET_PARAMETER = 0x18
    READ_PARAMTER = 0x17


class CybergearPacket:
    """Helper class for CyberGear packet formatting and parsing"""
    
    def __init__(self, primary_id, device_id):
        """
        Initialize the CyberGear packet helper
        
        Args:
            primary_id: Controller/host ID
            device_id: Default device ID to communicate with
        """
        self.primary_id = primary_id
        self.device_id = device_id
    
    def construct_can_id(self, command_id, target_id, host_id, motor_id):
        """
        Construct a CAN ID according to the CyberGear protocol
        
        Format: [command_id][target_id][host_id][motor_id]
        
        Args:
            command_id: Command ID (see CommandId enum)
            target_id: Target/parameter ID (for change_id, this is the new ID)
            host_id: Controller/host ID
            motor_id: Current motor ID
            
        Returns:
            int: Constructed CAN ID
        """
        return (command_id << 24) | (target_id << 16) | (host_id << 8) | motor_id
    
    def parse_can_id(self, can_id):
        """
        Parse a CAN ID into its components
        
        Args:
            can_id: CAN ID to parse
            
        Returns:
            tuple: (command_id, target_id, host_id, device_id)
        """
        command_id = (can_id >> 24) & 0xFF
        target_id = (can_id >> 16) & 0xFF
        host_id = (can_id >> 8) & 0xFF
        device_id = can_id & 0xFF
        
        return (command_id, target_id, host_id, device_id)
    
    def get_frame_id(self, can_id):
        """
        Get the device ID from a CAN frame ID
        
        Args:
            can_id: CAN ID
            
        Returns:
            int: Device ID
        """
        return can_id & 0xFF
    
    def is_info_frame(self, can_id):
        """
        Check if the CAN frame is an info frame
        
        Args:
            can_id: CAN ID
            
        Returns:
            bool: True if this is an info frame
        """
        command_id = (can_id >> 24) & 0xFF
        return command_id == CommandId.READ_INFO
    
    def get_change_ki_frame(self, target_value):
        """
        Create a CAN frame ID for changing speed KI
        
        Structure:
        - SET_SPD_KI command (0x7020)
        - New KI value as target
        - Host ID (primary_id)
        - Current device ID
        
        Args:
            target_id: New KI value for the device
            
        Returns:
            int: CAN frame ID
        """
        return self.construct_can_id(
            CommandId.SET_PARAMETER,
            0,
            self.primary_id,
            self.device_id,
            
        )
    
    def get_read_param_frame(self):
        """
        Create a CAN frame ID for reading parameters
        
        Structure:
        - READ_PARAMTER command (0x17)
        - Target ID (0 for all)
        - Host ID (primary_id)
        -
        """

        return self.construct_can_id(
            CommandId.READ_PARAMTER,
            0,  # Target ID not used for read
            self.primary_id,
            111
        )


    
    def get_change_device_id_frame(self, target_id):
        """
        Create a CAN frame ID for changing device ID
        
        Structure:
        - CHANGE_ID command (0x07)
        - New ID as target
        - Host ID (primary_id)
        - Current device ID
        
        Args:
            target_id: New ID for the device
            
        Returns:
            int: CAN frame ID
        """
        return self.construct_can_id(
            CommandId.CHANGE_ID,
            target_id,
            self.primary_id,
            self.device_id
        )
    
    def get_read_info_frame(self, query_id):
        """
        Create a CAN frame ID for reading device info
        
        Args:
            query_id: ID of the device to query
            
        Returns:
            int: CAN frame ID
        """
        return self.construct_can_id(
            CommandId.READ_INFO,
            0,  # Target ID not used for read
            self.primary_id,
            query_id
        )
    
    def get_motor_off_frame(self, motor_id=None):
        """
        Create a CAN frame ID for turning motor off
        
        Args:
            motor_id: ID of the motor (defaults to self.device_id)
            
        Returns:
            int: CAN frame ID
        """
        if motor_id is None:
            motor_id = self.device_id
            
        return self.construct_can_id(
            CommandId.MOTOR_OFF,
            0,  # Target ID not used
            self.primary_id,
            motor_id
        )
    
    def get_motor_on_frame(self, motor_id=None):
        """
        Create a CAN frame ID for turning motor on
        
        Args:
            motor_id: ID of the motor (defaults to self.device_id)
            
        Returns:
            int: CAN frame ID
        """
        if motor_id is None:
            motor_id = self.device_id
            
        return self.construct_can_id(
            CommandId.MOTOR_RUNNING,
            0,  # Target ID not used
            self.primary_id,
            motor_id
        )
    
    def get_set_position_frame(self, motor_id=None, mode=3):
        """
        Create a CAN frame ID for setting position
        
        Args:
            motor_id: ID of the motor (defaults to self.device_id)
            mode: Position control mode (3-7)
            
        Returns:
            int: CAN frame ID
        """
        if motor_id is None:
            motor_id = self.device_id
            
        if mode < 3 or mode > 7:
            raise ValueError("Position mode must be between 3 and 7")
            
        command_id = getattr(CommandId, f"SET_POSITION_{mode}")
            
        return self.construct_can_id(
            command_id,
            0,  # Target ID not used
            self.primary_id,
            motor_id
        )
    
    def create_position_data(self, position, max_speed=0.0):
        """
        Create data field for position command
        
        Args:
            position: Target position in degrees
            max_speed: Maximum speed for position control
            
        Returns:
            list: Data field bytes
        """
        # Convert position to int (position * 100)
        pos_int = int(position * 100)
        
        # Convert max_speed to int (speed * 100)
        speed_int = int(max_speed * 100)
        
        return [
            (pos_int >> 0) & 0xFF,
            (pos_int >> 8) & 0xFF,
            (pos_int >> 16) & 0xFF,
            (pos_int >> 24) & 0xFF,
            (speed_int >> 0) & 0xFF,
            (speed_int >> 8) & 0xFF,
            0,
            0
        ]
    
    @staticmethod
    def format_can_id_for_debug(can_id):
        """
        Format CAN ID for debug output in the standard format
        
        Args:
            can_id: CAN ID
            
        Returns:
            str: Formatted CAN ID string
        """
        cmd = (can_id >> 24) & 0xFF
        target = (can_id >> 16) & 0xFF
        host = (can_id >> 8) & 0xFF
        device = can_id & 0xFF
        
        return f"{cmd:02X}{target:02X}{host:02X}{device:02X}"

    