from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="cybergear_maintenance_python",
            executable="zero_motor_service",
            name="zero_motor_service",
            parameters=["config/motor_id_config.yaml"]
        )
    ])