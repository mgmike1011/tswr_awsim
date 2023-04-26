import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand 
from geometry_msgs.msg import PoseStamped 

class StanleyControllerNode(Node):

    def __init__(self):
        super().__init__('stanley_controller')
        self.get_logger().info('Stanley Controller start!')

    
def main(args=None):
    rclpy.init(args=args)
    StanleyController = StanleyControllerNode()
    rclpy.spin(StanleyController)
    StanleyController.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()