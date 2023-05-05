import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand 
from geometry_msgs.msg import PoseStamped 

class referenceTrajectoryNode(Node):

    def __init__(self):
        super().__init__('reference_trajectory_generator')
        self.get_logger().info('Reference trajectory generator start!')
        # 
        # Reference trajcetory publisher
        # 
        # self.control_publisher = self.create_publisher(AckermannControlCommand, '/control/command/control_cmd', 10) #TODO

    
def main(args=None):
    rclpy.init(args=args)
    referenceTrajectoryGenerator = referenceTrajectoryNode()
    rclpy.spin(referenceTrajectoryGenerator)
    referenceTrajectoryGenerator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()