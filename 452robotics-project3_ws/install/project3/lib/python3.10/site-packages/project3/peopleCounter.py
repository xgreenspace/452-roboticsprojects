# ros2 service type /turtle1/set_pen -> turtlesim/srv/SetPen
# ros2 interface show turtlesim/srv/SetPen


import time
import rclpy.node
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rosbag2_py import Player, StorageOptions, PlayOptions
from example_interfaces.msg import Int64
from std_msgs.msg import MultiArrayDimension, MultiArrayLayout, Float32MultiArray

from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan, PointCloud
from sklearn.neighbors import NearestNeighbors
from scipy.spatial.distance import cdist

import numpy as np
from scipy.spatial.distance import pdist, squareform
from sklearn.cluster import DBSCAN
from scipy.spatial import ConvexHull


start_time = time.time()
curr_frame = 0

def euclidean_distance(a, b):
    # Compute the Euclidean distance between two arrays a and b
    return np.sqrt(np.sum(np.square(a - b)))

def distance_between_arrays(A, B):
    # Compute the pairwise distances between arrays A and B
    distances = np.zeros((len(A), len(B)))
    for i, a in enumerate(A):
        for j, b in enumerate(B):
            distances[i,j] = euclidean_distance(a, b)
    return distances
    
    

class PersonParticleFilter:
    def __init__(self, max_distance=0.5):
        self.tracks = {}
        self.current_id = 0
        self.max_distance = max_distance
    
    def update(self, centroids):

        if curr_frame < 20:
            self.current_id = 0
            self.tracks = {}
            return self.tracks
        
        if len(self.tracks) == 0:
            for new_centroid in centroids:
                self.tracks[self.current_id] = new_centroid
                self.current_id += 1
        else: # for the rest of the scans
            for new_centroid in centroids:
                # 1) Find the best matching centroid from the previous canidates to this current, new one to detect centroid motion (in any direction)
                distances = []  # list of possible matching canidates
                for track_id, prev_centroid in self.tracks.items():  # for each previous centroid compute the distance from it to current centroid
                    distance = np.linalg.norm(prev_centroid - new_centroid)
                    distances.append((track_id, distance))  # store as tuple so I can extract the id too 

                closest_track_id, closest_distance = min(distances, key=lambda x: x[1])  # to compare tuple, we use a special key to compare by tuple's index 1 - the distance
                
                # 2) Decide whether its a new centroid or already seen
                if closest_distance < self.max_distance:  # if the closest_distance is within some threshold, then the curr_centroid is probably the same centroid as the one we matched with
                    self.tracks[closest_track_id] = new_centroid
                else:
                    self.tracks[self.current_id] = new_centroid # Otherwise, its a new centroid with a new id
                    self.current_id += 1

        return self.tracks
        
        

class StaticObjectFilter:
    def __init__(self):
        self.history = []
        self.history_length = 5
        self.threshold = 4  # old = 0.5

    def filter_static_objects(self, current_scan):
        self.history.append(current_scan)
        if len(self.history) > self.history_length:
            self.history.pop(0)

        if len(self.history) < self.history_length:
            return current_scan  # Not enough history yet, return the current scan

        # Calculate the median position of each point over the history
        median_positions = np.median(self.history, axis=0)

        # Remove points that are close to their median position
        filtered_scan = []
        for i, point in enumerate(current_scan):
            distance = np.linalg.norm(point - median_positions[i])
            if distance > self.threshold:
                filtered_scan.append(point)

        filtered_scan = np.array(filtered_scan)

        if np.all(filtered_scan == 0):
            filtered_scan = np.array([[0,0]])

        return np.array(filtered_scan)


class HumanTracker:
    def __init__(self):
        self.tracks = []
        self.match_threshold = 2.0
        self.max_missed_frames = 20

    def _match_tracks(self, new_clusters):
        updated_tracks = []

        for new_cluster in new_clusters:
            min_distance = float('inf')
            matched_track = None

            for track in self.tracks:
                cluster, missed_frames = track
                distance = np.linalg.norm(cluster - new_cluster)

                if distance < min_distance:
                    min_distance = distance
                    matched_track = track

            if matched_track and min_distance < self.match_threshold:
                updated_tracks.append((new_cluster, 0))
            else:
                updated_tracks.append((new_cluster, 0))

        for track in self.tracks:
            if not any(np.array_equal(track, updated_track) for updated_track in updated_tracks):
                cluster, missed_frames = track
                if missed_frames < self.max_missed_frames:
                    updated_tracks.append((cluster, missed_frames + 1))

        return updated_tracks


    def detect_humans(self, points):
        # Apply the DBSCAN clustering algorithm
        clustering = DBSCAN(eps=0.2, min_samples=5).fit(points)

        # Filter the clusters to find those that represent humans
        new_human_clusters = []
        for label in set(clustering.labels_):
            if label == -1:
                continue  # Ignore noise

            cluster_points = points[clustering.labels_ == label]
            cluster_size = cluster_points.ptp(axis=0)  # Get the size of the bounding box for the cluster

            # Set constraints for the size of a human-like cluster
            min_width, max_width = 0.01, 1.5
            min_height, max_height = 0.01, 2.2

            if min_width < cluster_size[0] < max_width and min_height < cluster_size[1] < max_height:
                centroid = np.mean(cluster_points, axis=0)
                new_human_clusters.append(centroid)

        # Update tracks with new_human_clusters
        self.tracks = self._match_tracks(new_human_clusters)

        # Count humans with non-missed detections
        human_count = len([track for track in self.tracks if track[1] == 0])

        with open('human_count_better.txt', 'a') as f:
            f.write(str(human_count) + "\n")

        return human_count, new_human_clusters




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
        self.publisher = self.create_publisher(Float32MultiArray, 'CartesianData', 10)  # Publish
        self.lol = self.create_publisher(LaserScan, 'test', 10)
        self.longest = {}

        
    def listener_callback(self, msg):
        self.get_logger().info('LaserScan message received') # Check if message have been received

        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))  
        
        polar_data = np.column_stack((ranges, angles)) 

        np.savetxt("polar_data.txt", polar_data, delimiter=',')

        filtered_polar_data = polar_data
        
        np.savetxt("filtered_polar_data.txt", filtered_polar_data[:,0] , delimiter=',')
        
        
        msg.ranges = list(filtered_polar_data[:,0].astype(float))  # publish this thing
        
        self.lol.publish(msg)

        coords = polar_to_cartesian(filtered_polar_data).flatten()  # two columns - first is x column, second is y column, gets flattened
        
        coords[np.isinf(coords)] = 20 # replace inf values with 20
        coords = np.nan_to_num(coords) # replace nan values with 0
        # np.savetxt("coords_post.txt", coords, delimiter=",")
        coords = coords.tolist()

        
        # Create the Float32MultiArray message
        msg = Float32MultiArray()

        # Create the layout message
        layout = MultiArrayLayout()
        layout.dim.append(MultiArrayDimension())
        layout.dim[0].label = "length"
        layout.dim[0].size = len(coords)
        layout.dim[0].stride = 2

        # Set the layout of the message
        msg.layout = layout

        # Set the data of the message
        msg.data = coords

        self.publisher.publish(msg)


        
class PeopleCounter(Node):
    def __init__(self):
        super().__init__('people_counter')
        self.pub_locs = self.create_publisher(PointCloud, 'person_locations', 10)
        self.pub_total = self.create_publisher(Int64, 'people_count_total', 10)
        self.pub_curr = self.create_publisher(Int64, 'people_count_current', 10)  # Publish
        
        # self.blah = self.create_publisher(Float32MultiArray, 'testing', 10) # for testing
        # self.filtered = self.create_publisher(PointCloud, 'filtered_points', 10) # for testing


        self.subscription = self.create_subscription(
            Float32MultiArray, 
            '/CartesianData', 
            self.listener_callback, 
            10
        )
        
        self.counter = HumanTracker()
        self.filter = StaticObjectFilter()
        
        self.tracker = PersonParticleFilter(1)

    
    def listener_callback(self, msg):
        global curr_frame
        curr_frame += 1
        # Reformat back into n x 2 array
        n = msg.layout.dim[0].size  # length of total
        width = msg.layout.dim[0].stride  # == 2
        coords = np.array(msg.data)

        # Calculate the number of rows
        rows = int(n / width)

        # Create a new 2-D array of shape n x 2
        coords = coords.reshape(rows, width)

        coords_filtered = self.filter.filter_static_objects(coords)
        
        curr_count = Int64()
        curr_count.data, centroids = self.counter.detect_humans(coords_filtered)
        centroids = np.array(centroids)
        
        loc_msg = PointCloud()
        filtered_msg = PointCloud()
        
        loc_msg.header.stamp = self.get_clock().now().to_msg()
        loc_msg.header.frame_id = 'laser'
        
        points = [Point32(x=float(coord[0]), y=float(coord[1]), z=0.0) for coord in centroids]
        
        # Partical filter
        tracks = self.tracker.update(centroids)
        #tracked_persons = [Point32(x=float(tracks[key][0]), y=float(tracks[key][1]), z=0.0) for key in tracks]
        
        loc_msg.points = points
        
        self.pub_locs.publish(loc_msg)
        self.pub_curr.publish(curr_count)
        
        filtered_msg.header.stamp = self.get_clock().now().to_msg()
        filtered_msg.header.frame_id = 'laser'
        
        # debugging filtered points
        # filtered_points = [Point32(x=float(coord[0]), y=float(coord[1]), z=0.0) for coord in coords_filtered]
        # filtered_msg.points = filtered_points
        # self.filtered.publish(filtered_msg)
        
        total_count = Int64()
        total_count.data = len(tracks)
        self.pub_total.publish(total_count)
        

#convert distance vectors (polar coordinates) to points in 2D (cartesian coordinates)
def analyze_cluster_shape(cluster_points):
    hull = ConvexHull(cluster_points)
    hull_area = hull.volume
    hull_length = hull.area

    aspect_ratio = hull_length / hull_area
    circularity = (4 * np.pi * hull_area) / (hull_length ** 2)

    return aspect_ratio, circularity  

def polar_to_cartesian(polar_data):
    # polar_data: A 2D NumPy array with columns [distance, angle (in radians)]
    x = polar_data[:, 0] * np.cos(polar_data[:, 1])
    y = polar_data[:, 0] * np.sin(polar_data[:, 1])
    return np.column_stack((x, y))

def find_nearest_neighbors(pts1, pts2):
    nn = NearestNeighbors(n_neighbors=1)
    nn.fit(pts2)
    distances, indices = nn.kneighbors(pts1)
    return distances.ravel(), indices.ravel()

def estimate_and_match_clusters(clusters, human_candidates):
    matched_clusters = []
    
    # update human candidates' Kalman filter 
    for hc in human_candidates:
        hc.predict()
    
    # estimate the positions of human candidates
    estimated_positions = [hc.kf.x[:2] for hc in human_candidates]  # hc.kf.x[:2] = (x, y) = centroid of human
    
    # match the clusters to the human candidates based on the distance
    cluster_distances, matched_indices = find_nearest_neighbors(estimated_positions, clusters)
    for i, (dist, cluster_idx) in enumerate(zip(cluster_distances, matched_indices)):
        if dist < 5.2: #SET THE DISTANCE THREADSHOLD
            human_candidates[i].update(clusters[cluster_idx])
            matched_clusters.append(cluster_idx)
            
    return matched_clusters


def main(args=None):
    rclpy.init(args=args)
    data_listener = DataScanListener()
    ppl_counter = PeopleCounter()
    

    executor = SingleThreadedExecutor()
    executor.add_node(data_listener)
    executor.add_node(ppl_counter)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    
    end_time = time.time()
    executor.shutdown()



if __name__ == '__main__':
    main()

        


        

