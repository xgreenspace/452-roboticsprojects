from launch import LaunchDescription
from launch_ros.actions import *
from project4.disc_robot import load_disc_robot
from launch.actions import *
from launch.substitutions import *
from launch.event_handlers import *
from launch.events import *

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # load file stuff
    robotfile = "normal.robot"
    urdf = os.path.join(
        get_package_share_directory('project4'),
        robotfile)
    robot = load_disc_robot(urdf)
    
    
    node = Node(package='project4',
                executable='simulator')
    robot_state_publisher = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot['urdf']}],
                arguments=[urdf])
    

    
    # # read in the robot and world file
    # # declares new argument
    # robotIn = DeclareLaunchArgument('robot', default_value = 'normal.robot') 
    # # read in that new argument (not string)
    # robotInArg = LaunchConfiguration('robot')

    # # declares new argument
    # mapIn = DeclareLaunchArgument('map', default_value = 'brick.world') #command line is: ros2 launch project4 launch.py map:=brick.world
    # # read in that new argument (not string)
    # mapInArg = LaunchConfiguration('map')
    
    ld = LaunchDescription([ node, robot_state_publisher ])#
    return ld