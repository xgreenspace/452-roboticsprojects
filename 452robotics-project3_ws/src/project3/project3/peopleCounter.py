# ros2 service type /turtle1/set_pen -> turtlesim/srv/SetPen
# ros2 interface show turtlesim/srv/SetPen

import math
import rclpy.node
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosbag2_py import Player, StorageOptions, PlayOptions
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray


from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from sensor_msgs.msg import LaserScan
from sklearn.neighbors import NearestNeighbors
from filterpy.kalman import KalmanFilter

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.spatial.distance import pdist, squareform
from sklearn.cluster import DBSCAN

import os
import sys


#convert distance vectors (polar coordinates) to points in 2D (cartesian coordinates)
def polar_to_cartesian(polar_data):
    # polar_data: A 2D NumPy array with columns [distance, angle (in radians)]
    x = polar_data[:, 0] * np.cos(polar_data[:, 1])
    y = polar_data[:, 0] * np.sin(polar_data[:, 1])
    return x, y  # np.column_stack((x, y))

def find_nearest_neighbors(pts1, pts2):
    nn = NearestNeighbors(n_neighbors=1)
    nn.fit(pts2)
    distances, indices = nn.kneighbors(pts1)
    return distances.ravel(), indices.ravel()

# Detect humans and estimate and match clusters will be called in the callback function of the scan subscriber
def detect_humans(matched_clusters, cluster_params, human_candidates):
    detected_humans = []
    
    for cluster_idx in matched_clusters:
        cluster = cluster_params[cluster_idx]
        size, circumference, straightness = cluster

        if size > 0.2 and circumference > 0.6 and straightness > 0.8: # PLACEHOLDER CONDITION
            detected_humans.append(human_candidates[cluster_idx])
            
    return detected_humans

def estimate_and_match_clusters(clusters, cluster_params, human_candidates):
    matched_clusters = []
    
    # update human candidates' Kalman filter 
    for hc in human_candidates:
        hc.predict()
    
    # estimate the positions of human candidates
    estimated_positions = [hc.kf.x[:2] for hc in human_candidates]
    
    # match the clusters to the human candidates based on the distance
    cluster_distances, matched_indices = find_nearest_neighbors(estimated_positions, clusters)
    for i, (dist, cluster_idx) in enumerate(zip(cluster_distances, matched_indices)):
        if dist < 5.2: #SET THE DISTANCE THREADSHOLD
            human_candidates[i].update(clusters[cluster_idx])
            matched_clusters.append(cluster_idx)
            
    return matched_clusters

class HumanCandidate():
    def __init__(self, id, initial_position):  # ADD PARAMETER FOR TIMESTEP dt WHEN CONSTRUCTING
        self.id = id
        self.kf = KalmanFilter(dim_x=4, dim_z=2)

        # Initialize the Kalman filter state and covariance matrices
        self.kf.x = np.array([initial_position[0], initial_position[1], 0, 0])  # state=[x, y, dx/dt, dy/dt] : [<centroid>, <velocities>]
        self.kf.P *= 1e3
        self.kf.R *= 10

        # Define the Kalman filter state transition and observation matrices
        self.dt = 1  #! CHANGE LATER
        self.kf.F = np.array([[1, 0, self.dt, 0],  # x' = 1*x + dt*vx
                              [0, 1, 0, self.dt],  # y' = 1*y + dt*vy
                              [0, 0, 1, 0],   # Doesn't matter because self.update will overwrite it
                              [0, 0, 0, 1]])  # Doesn't matter because self.update will overwrite it

        self.kf.H = np.array([[1, 0, 0, 0],
                              [0, 1, 0, 0]])
        

    def predict(self):
        self.kf.predict()

    def update(self, measurement):
        # Derive velocities from observations
        vel = (measurement - self.kf.H @ self.kf.x) / self.dt
        self.kf.x[2:4] = vel  # self.kf.x = [x, y, vx, vy] -> [x, y, vx', vy']  dynamically changes the current state's velocities to estimate the next state's velocities
        self.kf.update(measurement)  # self.kf.x' = add_estimated_velocities(self.)f.x
        
#node that subscribes to scan
class DataScanListener(Node):
    def __init__(self):
        super().__init__('data_listener')
        self.subscription = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.listener_callback, 
            10
        )
        self.subscription  # prevent unused variable warning
        
        self.publisher = self.create_publisher(String, 'CartesianData', 10)  # Publish

        
    def listener_callback(self, msg):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))  
        
        print(ranges)
        print(angles)

        polar_data = np.column_stack((ranges, angles))
        xlist, ylist = polar_to_cartesian(polar_data)
        CartesianCoords:
        float32[] - x list
        float32[] - y list
        
        msg = CartesianCoords()
        msg.x = xlist
        msg.y = ylist
        self.publisher.publish(msg)

        

#node that publishes messages person_locations, people_count_current, people_count_total
#message type sensor_msgs/PointCloud on topic /person_locations: one point for each visible person on the center of that person
        #The header field of this message type includes a field called frame_id, which you should set to the same value (the string “laser”) as the frame_id on the laser scan messages stored in the bag files.
        #The channels field of this message should be left blank.
#message type example_msgs/Int64 on topic /people_count_current: indicate number of people in the scene
#message type example_msgs/Int64 on topic /people_count_total: indicate total number of people since the beginning
class PeopleCounter(Node):
    def __init__(self):
        super().__init__('people_counter')
        self.publisher = self.create_publisher(String, 'person_locations', 10)
        self.publisher = self.create_publisher(Int64, 'people_count_current', 10)
        self.publisher = self.create_publisher(Int64, 'people_count_total', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.count
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.count += 1

    def cluster_points(data, min_distance, min_points):
        distance_matrix = squareform(pdist(data))

        adjacency_matrix = (distance_matrix < min_distance).astype(int)

        clustering = DBSCAN(eps=min_distance, min_samples=min_points, metric='precomputed')
        labels = clustering.fit_predict(adjacency_matrix)

    def compute_cluster_parameters(data, labels):
        unique_labels = np.unique(labels)
        cluster_parameters = []

        for label in unique_labels:
            if label == -1:
                continue

        cluster_data = data[labels == label]
        min_coords = cluster_data.min(axis=0)
        max_coords = cluster_data.max(axis=0)
        size = max_coords - min_coords
        circumference = 2 * (size[0] + size[1])
        
        # Compute the straightness
        start, end = cluster_data[[0, -1]]
        line = end - start
        line_norm = np.linalg.norm(line)
        line_unit = line / line_norm
        straightness = np.mean(np.abs(np.cross(cluster_data - start, end - start))) / line_norm

        cluster_parameters.append((size, circumference, straightness))


def main(args=None):
    rclpy.init(args=args)
    data_listener = DataScanListener()
    
    print("Hello world")
    rclpy.spin(data_listener)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# class TAMUBot(Node):

#     def __init__(self):
#         super().__init__('tamuturtle')
        
#         self.req = SetParameters.Request()
#         self.req.parameters = [
#             Parameter(name='background_r', value=ParameterValue(integer_value=80, type=ParameterType.PARAMETER_INTEGER)),
#             Parameter(name='background_g', value=ParameterValue(integer_value=0, type=ParameterType.PARAMETER_INTEGER)),
#             Parameter(name='background_b', value=ParameterValue(integer_value=0, type=ParameterType.PARAMETER_INTEGER))
#         ]
# # im testing now
#         self.SetBackground = self.create_client(SetParameters, 'turtlesim/set_parameters')
#         self.SetBackground.call_async(self.req)
        
#         self.publisher_cmdvel = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

#         self.subscriber_pose = self.create_subscription(Pose, '/turtle1/pose', self.callback, 10)

#         timer_period = 0.5
#         self.timer = self.create_timer(timer_period, self.move2goal)

#         self.pose = Pose()
#         self.flag = False

#         self.points = read_points("points.txt")
#         self.steps = 0 # Current point
        
#         self.goal_pose= Pose()
#         goal = rescale(*self.points[0])
        
#         self.goal_pose.x, self.goal_pose.y = goal # first point on letter
#         self.goal_pose.theta = self.steering_angle(self.goal_pose) # angle to letter

#         self.pen = self.create_client(SetPen, 'turtle1/set_pen')

#         self.set_pen(False)

#     def set_pen(self, state):
#         msg = SetPen.Request()
#         msg.r = 255
#         msg.g = 255
#         msg.b = 255
#         msg.width = 1
#         msg.off = not state
#         self.pen.call_async(msg)
    

#     def callback(self, data):
#         self.pose.x = data.x
#         self.pose.y = data.y
#         self.pose.theta = data.theta
#         msg = 'X: {:.3f}, Y: {:.3f}, Theta: {:.3f}'.format(data.x, data.y, data.theta)
#         self.get_logger().info(msg)

        
#     # returns distance between points, and angle to the horizontal of the source
#     def euclidean(self, goal_pose):
#         return math.hypot(*trans((self.pose.x, self.pose.y), (goal_pose.x, goal_pose.y)))
    

#     def steering_angle(self, goal_pose):
#         vector = trans((self.pose.x, self.pose.y), (goal_pose.x, goal_pose.y))
#         return angle(vector)

#     def linear_vel(self, goal_pose):
#         return 2 * self.euclidean(goal_pose)

#     def angular_vel(self, goal_pose):
#         return 2 * (self.steering_angle(goal_pose)-self.pose.theta)


#     def move2goal(self):

#         distance_tolerance = 0.01
#         angular_tolerance = 0.01

#         goal_pose = self.goal_pose

#         vel_msg = Twist()

#         if abs(self.steering_angle(goal_pose)-self.pose.theta) > angular_tolerance:
#             vel_msg.linear.x = 0.0
#             vel_msg.angular.z = self.angular_vel(goal_pose)
#         else:
#             vel_msg.angular.z = 0.0
#             if self.euclidean(goal_pose)>=distance_tolerance:
#                 vel_msg.linear.x = self.linear_vel(goal_pose)
#             else:
#                 vel_msg.linear.x = 0.0
#                 self.flag = True
        
#         if self.flag:
#              # put that in here
#             self.steps += 1
            
#             if self.steps >= len(self.points):
#                 self.steps = len(self.points) - 1
#                 quit()
            
#             goal = self.points[self.steps]
            
#             if goal == "STOP DRAWING":
#                 #turn off pen
#                 self.set_pen(False)
#                 self.steps += 1
#                 goal = self.points[self.steps]
                
#             if goal == "START DRAWING": # this string is 13 characters long
#                 #turn on pen
#                 self.set_pen(True)
#                 self.steps += 1
#                 goal = self.points[self.steps]
            
#             goal = rescale(*goal)  # a string is being passed into it


#             self.goal_pose= Pose()
#             self.goal_pose.x, self.goal_pose.y = goal # next point
#             self.goal_pose.theta = self.steering_angle(self.goal_pose)  # should be angle from current point to next point
#             self.flag = False
           
        
#         self.publisher_cmdvel.publish(vel_msg)
        

