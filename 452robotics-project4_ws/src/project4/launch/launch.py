from launch import LaunchDescription
from launch_ros.actions import *

def generate_launch_description():
    node = Node(package='project4',
                executable='simulator')
    robot_state_publisher = Node(package='robot_state_publisher',
                executable='robot_state_publisher')
    ld = LaunchDescription([ node, robot_state_publisher ])
    return ld