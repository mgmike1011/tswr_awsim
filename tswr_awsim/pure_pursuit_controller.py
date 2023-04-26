import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand 
from geometry_msgs.msg import PoseStamped 

class PurePursuitNode(Node):

    def __init__(self):
        super().__init__('pure_pursuit_controller')
        self.get_logger().info('Pure Pursuit Controller start!')

    
def main(args=None):
    rclpy.init(args=args)
    PurePursuitController = PurePursuitNode()
    rclpy.spin(PurePursuitController)
    PurePursuitController.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()