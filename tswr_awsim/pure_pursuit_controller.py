import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand 
from geometry_msgs.msg import PoseStamped 
from nav_msgs.msg import Path
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, QoSDurabilityPolicy
import numpy as np

def quat2eulers(q0:float, q1:float, q2:float, q3:float) -> tuple:
    """
    Compute yaw-pitch-roll Euler angles from a quaternion.
    @author: michaelwro
    Args
    ----
        q0: Scalar component of quaternion. [qw]
        q1, q2, q3: Vector components of quaternion. [qx, qy, qz]
    
    Returns
    -------
        (roll, pitch, yaw) (tuple): 321 Euler angles in radians
    """
    roll = math.atan2(
        2 * ((q2 * q3) + (q0 * q1)),
        q0**2 - q1**2 - q2**2 + q3**2
    )  # radians
    pitch = math.asin(2 * ((q1 * q3) - (q0 * q2)))
    yaw = math.atan2(
        2 * ((q1 * q2) + (q0 * q3)),
        q0**2 + q1**2 - q2**2 - q3**2
    )
    return (roll, pitch, yaw)

class PurePursuitNode(Node):

    def __init__(self):
        super().__init__('pure_pursuit_controller')
        self.get_logger().info('Pure Pursuit Controller start!')
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
        self.reference_trajectory_subscription = self.create_subscription(Path, '/path_points', self.reference_trajectory_listener_callback, 10)
        # Position
        self.ref_path = None
        # 
        # Control publisher
        # 
        qos_policy = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, depth=10)
        self.control_publisher = self.create_publisher(AckermannControlCommand, '/control/command/control_cmd', qos_policy)
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
        # Dimensions from https://people.ciirc.cvut.cz/~klapajar/studenti/F3-DP-2020-Zahradka-David-thesis.pdf
        self.wheelbase = 0.33  # meters, distance from rear wheels to front wheels
        self.track = 0.23  # meters, distance between left and right wheels
        self.lookahead_distance = 0.5  # meters

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

    def reference_trajectory_listener_callback(self, msg:Path):
        self.ref_path = []
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            qx = pose.pose.orientation.x
            qy = pose.pose.orientation.y
            qz = pose.pose.orientation.z
            qw = pose.pose.orientation.w
            self.ref_path.append([x, y, qx, qy, qz, qw])

    def publish_control(self, theta, accel):
        acc = AckermannControlCommand()
        acc.longitudinal.speed = 1.0
        acc.lateral.steering_tire_angle = theta
        acc.longitudinal.acceleration = accel
        self.control_publisher.publish(acc)

    def control_publisher_timer_callback(self):
        if (self.theta is not None) and (self.acceleration is not None):
            self.publish_control(self.theta, self.acceleration)
            self.get_logger().info(f'Controller output: theta: {self.theta}, acceleration: {self.acceleration}')
        else:
            self.get_logger().info(f'Pure Pursuit Controller wrong control!')

    def control_timer_callback(self):
        # Check if we have the current pose and reference trajectory
        if None in [self.curr_x, self.curr_y, self.curr_qw, self.curr_qx, self.curr_qy, self.curr_qz, self.ref_path]:
            return

        # Get the current position and orientation of the vehicle
        curr_pose = np.array([self.curr_x, self.curr_y])
        curr_yaw = quat2eulers(self.curr_qw, self.curr_qx, self.curr_qy, self.curr_qz)[2]

        # Find the nearest point on the reference trajectory
        # ref_path = [x, y, qx, qy, qz, qw]
        ref_points = np.array([[p[0], p[1]] for p in self.ref_path])
        # Calculate distance between reference points and actual pose
        distances = np.linalg.norm(ref_points - curr_pose, axis=1)
        # Find index of nearest reference point to current position
        nearest_idx = np.argmin(distances)
        nearest_point = ref_points[nearest_idx]

        # Find the lookahead point on the reference trajectory
        for i in range(nearest_idx, len(self.ref_path)):
            ref_point = np.array([self.ref_path[i][0], self.ref_path[i][1]])
            if np.linalg.norm(ref_point - curr_pose) > self.lookahead_distance:
                lookahead_point = ref_point
                break
        else:
            lookahead_point = ref_points[-1]

        # Calculate the steering angle using the pure pursuit algorithm
        # Equations from https://dingyan89.medium.com/three-methods-of-vehicle-lateral-control-pure-pursuit-stanley-and-mpc-db8cc1d32081
        dx = lookahead_point[0] - curr_pose[0]
        dy = lookahead_point[1] - curr_pose[1]
        alpha = np.arctan2(dy, dx) - curr_yaw
        Lf = self.lookahead_distance
        steering_angle = np.arctan2(2.0 * self.wheelbase * np.sin(alpha), Lf)
        self.theta = steering_angle

        # Calculate the acceleration as a constant value
        self.acceleration = 0.1



def main(args=None):
    rclpy.init(args=args)
    PurePursuitController = PurePursuitNode()
    rclpy.spin(PurePursuitController)
    PurePursuitController.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()