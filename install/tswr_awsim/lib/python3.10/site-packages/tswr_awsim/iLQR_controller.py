import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand 
from geometry_msgs.msg import PoseStamped 

class iLQRNode(Node):

    def __init__(self):
        super().__init__('iLQR_controller')
        self.get_logger().info('iLQR Controller start!')
        # 
        # Current pose
        # 
        self.current_pose_subscription = self.create_subscription(PoseStamped,'/ground_truth/pose', self.current_pose_listener_callback, 10)
        # Position
        self.curr_x = None
        self.curr_y = None
        self.curr_z = None
        # Orientation
        self.curr_qw = None
        self.curr_qx = None
        self.curr_qy = None
        self.curr_qz = None

    def current_pose_listener_callback(self, msg:PoseStamped):
        # Position
        self.curr_x = msg.pose.position.x
        self.curr_y = msg.pose.position.y
        self.curr_z = msg.pose.position.z
        # Orientation
        self.curr_qw = msg.pose.orientation.w
        self.curr_qx = msg.pose.orientation.x
        self.curr_qy = msg.pose.orientation.y
        self.curr_qz = msg.pose.orientation.z

    
def main(args=None):
    rclpy.init(args=args)
    iLQRController = iLQRNode()
    rclpy.spin(iLQRController)
    iLQRController.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()