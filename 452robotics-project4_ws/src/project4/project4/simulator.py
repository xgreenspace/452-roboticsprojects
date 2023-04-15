
import rclpy.node
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from disc_robot import *

        


class DifferentialDriveSimulator(Node):
    def __init__(self):
        super().__init__('DifferentialDrive')
        


class LidarSimulator(Node):
    def __init__(self):
        super().__init__('Lidar')


class VelocitySimulator(Node):
    def __init__(self):
        super().__init__('Velocity')
        

class MainSimulator(Node):
    def __init__(self):
        super().__init__('Main')
        self.URDF_publisher = self.create_publisher(?, 'RobotData', 10)
        self.World_publisher = self.create_publisher(?, 'GridData', 10)
        self.Frame_publisher = self.create(?, 'FrameData', 10)

        worldfile = "brick.world"
        robotfile = "normal.robot"

        
        
def main(args=None):
    rclpy.init(args=args)
    n1 = DifferentialDriveSimulator()
    n2 = LidarSimulator()
    n3 = VelocitySimulator()  #! Gotta remove this one when we turn in
    n4 = MainSimulator()

    
    executor = SingleThreadedExecutor()
    executor.add_node(n1)
    executor.add_node(n2)
    executor.add_node(n3)
    executor.add_node(n4)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    
    end_time = time.time()
    executor.shutdown()


if __name__ == "__main__":
    main()

