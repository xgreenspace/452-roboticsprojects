from launch import LaunchDescription
from launch_ros.actions import *

def generate_launch_description():
    playBag = Node(package='project3',
                executable='playBag')
    peopleCounter = Node(package='project3',
                executable='peopleCounter')
    ld = LaunchDescription([peopleCounter, playBag])
    return ld