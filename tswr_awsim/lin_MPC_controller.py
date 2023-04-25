import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand 
from geometry_msgs.msg import PoseStamped 

class linMPCNode(Node):

    def __init__(self):
        super().__init__('lin_MPC_controller')
        self.get_logger().info('Linearized MPC Controller start!')
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
        # 
        # Reference trajectory
        # 
        self.reference_trajectory_subscription = self.create_subscription(PoseStamped,'/control/reference_trajectory', 
                                                                          self.reference_trajectory_listener_callback, 10) #TODO
        # Position
        self.ref_x = None #TODO
        self.ref_y = None #TODO
        # 
        # Control publisher
        # 
        self.control_publisher = self.create_publisher(AckermannControlCommand, '/control/command/control_cmd', 10)
        control_publisher_timer_period = 1/50  # seconds
        self.control_publisher_timer = self.create_timer(control_publisher_timer_period, self.control_publisher_timer_callback)
        control_timer = 0.1 # seconds
        self.control_timer = self.create_timer(control_timer, self.control_timer_callback)
        # Steering angle
        self.theta = None #TODO
        # Acceleration
        self.acceleration = None #TODO
        # KOM: control_publisher publikuje sterowanie do biektu, ale w celu zapewnienia odpowiednio dużej liczby wysyłanych danych
        # wysyłanie realizowane jest częściej niż działa faktyczny kontroler. Dane wysyłane są w funkcji
        # callbacka od control_publisher_timer, natomiast działanie algorytmu regulatora implementowane jest
        # w callbacku control_timer. Wymagała tego specyfika działania symulatora :|

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

    def reference_trajectory_listener_callback(self, msg):
        pass #TODO

    def publish_control(self, theta, accel):
        acc = AckermannControlCommand()
        acc.lateral.steering_tire_angle = theta
        acc.longitudinal.acceleration = accel
        self.control_publisher.publish(acc)

    def control_publisher_timer_callback(self):
        if (self.theta is not None) and (self.acceleration is not None):
            self.publish_control(self.theta, self.acceleration)
            self.get_logger().info(f'Controller output: theta: {self.theta}, acceleration: {self.acceleration}')
        else:
            self.get_logger().info(f'Linearized MPC Controller wrong control')

    def control_timer_callback(self):
        self.theta = 1.0
        self.acceleration = 1.0
        # TODO


def main(args=None):
    rclpy.init(args=args)
    linMPCController = linMPCNode()
    rclpy.spin(linMPCController)
    linMPCController.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()