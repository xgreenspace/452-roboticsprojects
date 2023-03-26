from launch import LaunchDescription
from launch_ros.actions import *

def generate_launch_description():
    peopleCounter = Node(package='project3',
                executable='peopleCounter')
    playBag = Node(package='project3',
                executable='playBag')
    ld = LaunchDescription([ peopleCounter, playBag])
    return ld