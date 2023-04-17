
import rclpy.node
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from tf2_ros import TransformBroadcaster, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose, Quaternion, Point
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from nav_msgs.msg import OccupancyGrid
from tf2_geometry_msgs import do_transform_pose
from std_msgs.msg import String
import math


from tf2_msgs.msg import TFMessage

from project4.disc_robot import load_disc_robot

import yaml
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory





class DifferentialDriveSimulator(Node):
    def __init__(self):
        super().__init__('DifferentialDrive')
        
    
        


class LidarSimulator(Node):
    def __init__(self):
        super().__init__('Lidar')
    
        self.subscription = self.create_subscription(
                Pose, 
                '/robot_pos', 
                self.listener_callback, 
                10
            )
        self.subscription  # prevent unused variable warning
        
        self.publisher = self.create_publisher(String, 'testlaser', 10)  # Publish
        
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


    
    def listener_callback(self, pose:Pose):
        
        # Create a PoseStamped message with the input Pose2D and input frame id
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = pose 
        
        
        try:
            self.tf_buffer.can_transform(

                'baselink',
                'laser',
                pose_stamped.header.stamp,
                timeout=rclpy.time.Duration(seconds=5.0),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().error("Transform not available yet.")
            return
        
        
        # Get the transform
        try:
            #identity pose
            identity_pose = Pose()
            identity_pose.position = Point(x=0.0, y=0.0, z=0.0)
            identity_pose.orientation.w = 1.0
            
            #transform pose
            transform = self.tf_buffer.lookup_transform(
                            'base_link',
                            'laser',
                            rclpy.time.Time())

            transformed_pose = do_transform_pose(pose, transform)
            
            test = String()
            test.data = str(transformed_pose.position.x) + " " + str(transformed_pose.position.y) + " " + str(euler_from_quaternion(transformed_pose.orientation)[2])
            self.publisher.publish(test)
        except LookupException as e:
            self.get_logger().error(f"Error: {str(e)}")
        

    
def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q   

# for testing purposes
def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

      
                
def ReadWorldFile(filename):
    with open(filename) as f:
        world = yaml.safe_load(f)
    return world

def ParseMap(StringMap):
    mapp = []
    StringMap = StringMap.split('\n')
    for line in StringMap:
        if line != '':  # gets rid of extra newline at the end
            mapp.append([0 if c == '.' else 100 for c in list(line)])
    return mapp


class VelocitySimulator(Node):
    def __init__(self):
        super().__init__('Velocity')
        

class MainSimulator(Node):
    def __init__(self, ):
        super().__init__('Main')
        
        # TF Stuff
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_world_to_base_link_transform)
        
        # self.URDF_publisher = self.create_publisher(?, 'RobotData', 10)
        self.World_publisher = self.create_publisher(
                                    OccupancyGrid, 
                                    '/map', 
                                    qos_profile=QoSProfile(
                                        depth=1,
                                        durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                        history=HistoryPolicy.KEEP_LAST,
                                        )
                                    )
        # self.Frame_publisher = self.create(?, 'FrameData', 10)

        worldfile = "brick.world"
        robotfile = "normal.robot"
        
        
        robotfile = os.path.join(get_package_share_directory('project4'), robotfile)
        worldfile = os.path.join(get_package_share_directory('project4'), worldfile)
        
        self.robotYaml = load_disc_robot(robotfile)
        self.worldYaml = ReadWorldFile(worldfile)
        self.world_map = ParseMap(self.worldYaml["map"])
        self.world_height = len(self.world_map)
        self.world_width = len(self.world_map[0])
        self.world_map.reverse()  # row major order starting with (0, 0) at the top
        self.world_map = list(np.array(self.world_map).flatten())
        self.world_map = [int(a) for a in self.world_map]
        

        self.publish_occupancy_grid()
        self.timer = self.create_timer(1, self.publish_occupancy_grid)
        
        
        
        self.RobotState_publisher = self.create_publisher(Pose, 'robot_pos', 10)  # Publish
    
    
    def publish_occupancy_grid(self):
        
        grid = OccupancyGrid()
        grid.data = self.world_map
        grid.header.frame_id = 'world'
        grid.header.stamp = self.get_clock().now().to_msg()
        
        grid.info.resolution = self.worldYaml['resolution']  # Resolution of each grid cell (in meters)
        grid.info.width = self.world_width       # Width of the grid (in cells)
        grid.info.height = self.world_height      # Height of the grid (in cells)
        
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0
        
        self.World_publisher.publish(grid)


    def broadcast_world_to_base_link_transform(self):
        
        pose = self.worldYaml['initial_pose']
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = pose[0]
        t.transform.translation.y = pose[1]
        t.transform.translation.z = 0.0
        # t.transform.rotation.x = 0.0
        # t.transform.rotation.y = 0.0
        # t.transform.rotation.z = pose[2]
        
        quaternion = quaternion_from_euler(0, 0, pose[2])
        t.transform.rotation = quaternion


        self.br.sendTransform(t)
        
        
        # Define the input Pose2D
        input_pose = Pose()
        input_pose.position.x = pose[0]
        input_pose.position.y = pose[1]
        input_pose.position.z = 0.0
        input_pose.orientation = quaternion
        self.RobotState_publisher.publish(input_pose)
        
    # def collision_detection(self, msg):
    #     if msg.x < self.min_x or msg.x > self.max_x or msg.y < self.min_y or msg.y > self.max_y:
    #         self.stop_turtle()



        
def main(args=None):
    rclpy.init(args=args)
    # n1 = DifferentialDriveSimulator()
    n2 = LidarSimulator()
    # n3 = VelocitySimulator()  #! Gotta remove this one when we turn in
    n4 = MainSimulator()

    
    executor = SingleThreadedExecutor()
    # executor.add_node(n1)
    executor.add_node(n2)
    # executor.add_node(n3)
    executor.add_node(n4)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    
    #end_time = time.time()
    executor.shutdown()


if __name__ == "__main__":
    main()

