
import rclpy.node
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from disc_robot import *



class FileReader(Node):  # should be a service
    def __init__(self):
        super().__init__('FileReader')
        self.URDF_publisher = self.create_publisher(?, 'RobotData', 10)
        self.World_publisher = self.create_publisher(?, 'GridData', 10)
        self.Frame_publisher = self.create(?, 'FrameData', 10)

        
        # filenames
        world_filename = "brick.world"
        robot_filename = "normal.robot"
        
        
class DifferentialDriveSimulator(Node):
    def __init__(self):
        super().__init__('FileReader')
        



class LidarSimulator(Node):
    def __init__(self):
        super().__init__('FileReader')


class VelocitySimulator(Node):
    def __init__(self):
        super().__init__('FileReader')
        

class MainSimulator(Node):
    def __init__(self):
        super().__init__('FileReader')

        
        
def main(args=None):
    rclpy.init(args=args)
    freader = FileReader()
    diffdrive = DifferentialDriveSimulator()
    lidar = LidarSimulator()
    
    

    executor = SingleThreadedExecutor()
    executor.add_node(data_listener)
    executor.add_node(ppl_counter)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    
    end_time = time.time()
    executor.shutdown()

if __name__ == "__main__":
    main()

