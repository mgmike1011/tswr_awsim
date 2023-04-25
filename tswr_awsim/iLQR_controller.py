import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand 
from geometry_msgs.msg import PoseStamped 

class iLQRNode(Node):

    def __init__(self):
        super().__init__('iLQR_controller')
        self.get_logger().info('iLQR Controller start!')

    
def main(args=None):
    rclpy.init(args=args)
    iLQRController = iLQRNode()
    rclpy.spin(iLQRController)
    iLQRController.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()