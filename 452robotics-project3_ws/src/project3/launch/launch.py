from launch import LaunchDescription
from launch.actions import *
from launch.substitutions import *
from launch_ros.actions import *
from launch.event_handlers import *
from launch.events import *


def generate_launch_description():
    # playBag = Node(package='project3',
    #             executable='playBag')
    peopleCounter = Node(package='project3',
                executable='peopleCounter')
    
    # declares new argument
    bagIn = DeclareLaunchArgument('bagIn', default_value = 'example1') #command line is: ros2 launch project3 launch.py bagIn:=example1
    # read in that new argument (not string)
    bagInName = LaunchConfiguration('bagIn')
    # concatenated path for example# 
    bagPath = DeclareLaunchArgument('bagPath', default_value=['./src/project3/project3/bags/', bagInName])
    
    bagPathName = LaunchConfiguration('bagPath')
    
    ep = ExecuteProcess(cmd=['ros2', 'bag', 'play', bagPathName])
    
    #automatically terminate everything once the bag play finishes
    event_handler = OnProcessExit(target_action=ep,
                              on_exit=[EmitEvent(event=Shutdown())])
    
    terminate_at_end = RegisterEventHandler(event_handler)
    
    ld = LaunchDescription([bagIn, bagPath, terminate_at_end, ep, peopleCounter])
    return ld