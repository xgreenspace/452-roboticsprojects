
import rclpy.node
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from tf2_ros import TransformBroadcaster, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose, Quaternion, Point, Twist
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from nav_msgs.msg import OccupancyGrid
from tf2_geometry_msgs import do_transform_pose
from std_msgs.msg import String, Float64
from sensor_msgs.msg import LaserScan, PointCloud
import math
from math import atan2


from tf2_msgs.msg import TFMessage

from project4.disc_robot import load_disc_robot

import yaml
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import time



def getSecs():
    return time.time_ns() * 1e-9

class DifferentialDriveSimulator(Node):
    
    def __init__(self, dataYaml):
        super().__init__('DifferentialDrive')
        self.robotYaml = dataYaml[0]
        self.worldYaml = dataYaml[1]
        self.length = self.robotYaml['wheels']['distance']
        self.error_variance_left = self.robotYaml['wheels']['error_variance_left']
        self.error_variance_right = self.robotYaml['wheels']['error_variance_right']
        self.error_update_rate = self.robotYaml['wheels']['error_update_rate']
        self.timer = None  # time.time_ns() is in nanoseconds. I will convert to seconds everytime I call it
        self.prevUpdateState = None  # measures time passed since robot's state was updated. Diff from self.timer b/c self.timer is for checking if 1-sec has passed since last vel msg to stop the robot like turtlesim. prevUpdateState just measures the time passed since last called used for calculation delta_t. We measure b/c want more precision and timer might trigger late so precision lost

        self.timer_rate = 0.1

        self.vl = 0
        self.vr = 0

        self.subscription = self.create_subscription(
            Float64, 
            '/vl', 
            self.vl_listener, 
            10
        )
        self.subscription = self.create_subscription(
            Float64, 
            '/vr', 
            self.vr_listener, 
            10
        )

        self.subscription  # prevent unused variable warning

        self.create_timer(self.timer_rate, self.UpdateState)

        # For getting the current state:
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


        self.robot_pos_publisher = self.create_publisher(Twist, 'robot_pose', 10) 

        self.my_state = np.array(self.worldYaml['initial_pose'])



    def vl_listener(self, msg):
        self.vl = msg.data
        self.timer = getSecs()
    

    def vr_listener(self, msg):
        self.vr = msg.data
        self.timer = getSecs()
     

    def UpdateState(self):

        # Initialize self.prevUpdateState. If this is the first time it's called, then return immediately b/c this is technically the starting time of the driving
        if self.prevUpdateState is None:
            
            new_state_twist = Twist()
            new_state_twist.linear.x = self.my_state[0]
            new_state_twist.linear.y = self.my_state[1]
            new_state_twist.linear.z = 0.0
            new_state_twist.angular.z  = self.my_state[2]
            self.robot_pos_publisher(new_state_twist)
            
            self.prevUpdateState = getSecs()
            return

        if getSecs() - self.timer < 1:  # else don't do anything
            # Get the current state from the transform
            # try:
            #     self.tf_buffer.can_transform(

            #         'world',
            #         'base_link',
            #         self.get_clock().now().to_msg(),
            #         timeout=rclpy.time.Duration(seconds=5.0),
            #     )
            # except (LookupException, ConnectivityException, ExtrapolationException):
            #     self.get_logger().error("Transform not available yet.")
            #     return
            
            # # Get the transform
            # try:
            #     transform_base_to_laser = self.tf_buffer.lookup_transform(
            #                     'world',
            #                     'base_link',
            #                     rclpy.time.Time())
            # except LookupException as e:
            #     self.get_logger().error(f"Error: {str(e)}")
            
            # # Calculate the current state
            # q = transform_base_to_laser.transform.rotation  # quaternion
            # robot_center = transform_base_to_laser.transform.translation

            # # From the ICC slides
            # x, y, theta = robot_center.x, robot_center.y, atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

            state = self.my_state # np.array([x, y, theta])
            dt = getSecs() - self.prevUpdateState  # measures time duraction since last callback, should be around ~ self.timer_rate = 0.1
            try:
                R = (self.length / 2.0) * (self.vr + self.vl) / (self.vr - self.vl)

                omegalol = (self.vr - self.vl) / self.length

                icc = np.array([x - R * np.sin(theta), y + R * np.cos(theta), 0.0])
            
                dTheta = omegalol*dt

                matrix = np.array([[np.cos(dTheta), -np.sin(dTheta), 0.0],
                          [np.sin(dTheta), np.cos(dTheta), 0.0],
                          [0.0, 0.0, 1.0]
                ])
                x_vec = state - icc
                base_vec = icc + np.array([0.0, 0.0, dTheta])
                new_state = matrix @ x_vec + base_vec  # [x, y, theta]
                
            except: # drive in straight line
                distance = self.vr * dt
                x += distance * np.cos(theta)
                y += distance * np.sin(theta)
                new_state = np.array([x, y, theta])
                

            self.my_state = new_state
            new_state_twist = Twist()

            new_state_twist.linear.x = new_state[0]
            new_state_twist.linear.y = new_state[1]
            new_state_twist.linear.z = 0.0
            new_state_twist.angular.z  = new_state[2]
            self.robot_pos_publisher(new_state_twist)
            
        self.prevUpdateState = getSecs()  # update prev timer



class LidarSimulator(Node):
    def __init__(self, dataYaml):
        super().__init__('Lidar')
        
        self.robotYaml = dataYaml[0]
        self.worldYaml = dataYaml[1]
        self.angle_min = self.robotYaml['laser']['angle_min']
        self.angle_max = self.robotYaml['laser']['angle_max']
        self.range_min = self.robotYaml['laser']['range_min']
        self.range_max = self.robotYaml['laser']['range_max']
        self.rate = float(self.robotYaml['laser']['rate'])
        self.count = self.robotYaml['laser']['count']
        self.resolution = self.worldYaml['resolution']
        self.error_variance = self.robotYaml['laser']['error_variance']
        self.fail_probability = self.robotYaml['laser']['fail_probability']
        
        self.world_map = ParseMap(self.worldYaml["map"])
        self.world_map.reverse()

    
        self.create_timer(self.rate, self.listener_callback)
        
        self.publisher = self.create_publisher(LaserScan, 'scan', 10)  # Publish
        
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


    
    def listener_callback(self):
        
        # Create a PoseStamped message with the input Pose2D and input frame id
        # pose_stamped = PoseStamped()
        # pose_stamped.header.frame_id = "world"
        # pose_stamped.header.stamp = self.get_clock().now().to_msg()
        # pose_stamped.pose = pose 
        
        
        try:
            self.tf_buffer.can_transform(

                'world',
                'laser',
                self.get_clock().now().to_msg(),
                timeout=rclpy.time.Duration(seconds=5.0),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().error("Transform not available yet.")
            return
        
        
        # Get the transform
        try:
            #transform pose
            transform_base_to_laser = self.tf_buffer.lookup_transform(
                            'world',
                            'laser',
                            rclpy.time.Time())
            
            
            q = transform_base_to_laser.transform.rotation  # quaternion
            laser_rotation = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 *
                                   (q.y * q.y + q.z * q.z))
            laser_origin = transform_base_to_laser.transform.translation
            laser_origin.x
            laser_origin.y

            
            
            laser_range_angle = self.angle_min + laser_rotation
            laser_angles = np.linspace(laser_range_angle, self.angle_max + laser_rotation, self.count)
            
            
            def is_point_occupied(x, y):
                grid_x = int(x / self.resolution)
                grid_y = int(y / self.resolution)
                
                
                if grid_x < 0:
                    grid_x = 0
                if grid_y < 0:
                    grid_y = 0

                if grid_x >= len(self.world_map[0]):
                    grid_x = len(self.world_map[0]) - 1

                if grid_y >= len(self.world_map):
                    grid_y = len(self.world_map) - 1


                return self.world_map[grid_y][grid_x] == 100 
            
            
            ranges = []
            # points = []
            for angle in laser_angles:
                
                current_range = 0.0
                step_size = self.resolution / 10  # Step size can be adjusted for accuracy and performance

                while current_range <= self.range_max:
                    x = laser_origin.x + current_range * math.cos(angle)
                    y = laser_origin.y + current_range * math.sin(angle)

                    if is_point_occupied(x, y):
                        if np.random.random() > self.fail_probability:
                            ranges.append(current_range + np.random.normal(0, np.sqrt(self.error_variance)))
                        else:
                            ranges.append(np.nan)
                        # points.append((x,y))
                        break

                    current_range += step_size
                if current_range > self.range_max:
                    ranges.append(np.inf)
            
            # Lasers
            msg = LaserScan()
            msg.ranges = ranges
            msg.angle_min = self.angle_min
            msg.angle_max = self.angle_max
            msg.angle_increment = laser_angles[1] - laser_angles[0]
            msg.scan_time = self.rate
            msg.time_increment = 0.0
            msg.range_min = self.range_min
            msg.range_max = self.range_max
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'laser'
            
            self.publisher.publish(msg)
                    
                    
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
    def __init__(self, dataYaml):
        super().__init__('Main')
        
        # TF Stuff
        self.br = TransformBroadcaster(self)
        # self.timer = self.create_timer(0.1, self.broadcast_world_to_base_link_transform)
        
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

        self.Position_subsription = self.create_subscription(
                                    Twist,
                                    '/inital_pos',
                                    self.pos_listener,
                                    10 
        )

        self.Position_subsription
        # self.Frame_publisher = self.create(?, 'FrameData', 10)

        # worldfile = "brick.world"
        # robotfile = "normal.robot"
        
        
        # robotfile = os.path.join(get_package_share_directory('project4'), robotfile)
        # worldfile = os.path.join(get_package_share_directory('project4'), worldfile)
        
        self.robotYaml = dataYaml[0]
        self.worldYaml = dataYaml[1]
    

        self.world_map = ParseMap(self.worldYaml["map"])
        self.world_height = len(self.world_map)
        self.world_width = len(self.world_map[0])
        self.world_map.reverse()  # row major order starting with (0, 0) at the top
        self.world_map = list(np.array(self.world_map).flatten())
        self.world_map = [int(a) for a in self.world_map]
        

        self.publish_occupancy_grid()
        self.timer = self.create_timer(1, self.publish_occupancy_grid)
        
        
        
        # self.RobotState_publisher = self.create_publisher(Pose, 'robot_pose', 10)  # Publish
    
    def position_listener(self, msg):
        pose = [msg.linear.x, msg.linear.y, msg.angular.z]


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

        
    # def collision_detection(self, msg):
    #     if msg.x < self.min_x or msg.x > self.max_x or msg.y < self.min_y or msg.y > self.max_y:
    #         self.stop_turtle()



        
def main(args=None):
    
    
    # Read Files
    
    worldfile = "brick.world"
    robotfile = "normal.robot"
    
    robotfile = os.path.join(get_package_share_directory('project4'), robotfile)
    worldfile = os.path.join(get_package_share_directory('project4'), worldfile)
    
    robotYaml = load_disc_robot(robotfile)
    worldYaml = ReadWorldFile(worldfile)

    dataYaml = (robotYaml, worldYaml)
    
    
    rclpy.init(args=args)
    # n1 = DifferentialDriveSimulator()
    n2 = LidarSimulator(dataYaml)
    # n3 = VelocitySimulator()  #! Gotta remove this one when we turn in
    n4 = MainSimulator(dataYaml)

    
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

