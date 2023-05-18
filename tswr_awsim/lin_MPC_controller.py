import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math
import numpy as np
from scipy.linalg import solve_continuous_are, inv
from scipy.linalg import solve_discrete_are
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, QoSDurabilityPolicy
from scipy.linalg import eigvals
from numpy import cos, sin, tan, clip
from rclpy.clock import Clock


def quat2eulers(q0: float, q1: float, q2: float, q3: float) -> tuple:
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
        q0 ** 2 - q1 ** 2 - q2 ** 2 + q3 ** 2
    )  # radians
    pitch = math.asin(2 * ((q1 * q3) - (q0 * q2)))
    yaw = math.atan2(
        2 * ((q1 * q2) + (q0 * q3)),
        q0 ** 2 + q1 ** 2 - q2 ** 2 - q3 ** 2
    )
    return (roll, pitch, yaw)


class linMPCNode(Node):

    def __init__(self):
        super().__init__('lin_MPC_controller')
        self.get_logger().info('Linearized MPC Controller start!')
        #
        # Current pose
        #
        self.current_pose_subscription = self.create_subscription(PoseStamped, '/ground_truth/pose',
                                                                  self.current_pose_listener_callback, 10)
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
        self.reference_trajectory_subscription = self.create_subscription(Path, '/path_points',
                                                                          self.reference_trajectory_listener_callback,
                                                                          10)
        # Position
        self.ref_path = None
        #
        # Control publisher
        #
        qos_policy = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                depth=10)
        self.control_publisher = self.create_publisher(AckermannControlCommand, '/control/command/control_cmd',
                                                       qos_policy)
        control_publisher_timer_period = 1 / 30  # seconds
        self.control_publisher_timer = self.create_timer(control_publisher_timer_period,
                                                         self.control_publisher_timer_callback)
        control_timer = 0.1  # seconds
        self.control_timer = self.create_timer(control_timer, self.control_timer_callback)
        # Steering angle
        self.theta = None  # TODO
        # Acceleration
        self.acceleration = None  # TODO
        # KOM: control_publisher publikuje sterowanie do biektu, ale w celu zapewnienia odpowiednio dużej liczby wysyłanych danych
        # wysyłanie realizowane jest częściej niż działa faktyczny kontroler. Dane wysyłane są w funkcji
        # callbacka od control_publisher_timer, natomiast działanie algorytmu regulatora implementowane jest
        # w callbacku control_timer. Wymagała tego specyfika działania symulatora :|

        self.wheelbase = 0.33
        self.last_control_update_time = self.get_clock().now()

    def current_pose_listener_callback(self, msg: PoseStamped):
        # Position
        self.curr_x = msg.pose.position.x
        self.curr_y = msg.pose.position.y
        self.curr_z = msg.pose.position.z
        # Orientation
        self.curr_qw = msg.pose.orientation.w
        self.curr_qx = msg.pose.orientation.x
        self.curr_qy = msg.pose.orientation.y
        self.curr_qz = msg.pose.orientation.z

    def reference_trajectory_listener_callback(self, msg: Path):
        self.ref_path = []
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            qx = pose.pose.orientation.x
            qy = pose.pose.orientation.y
            qz = pose.pose.orientation.z
            qw = pose.pose.orientation.w
            # self.ref_path.append([x, y, qx, qy, qz, qw])
            self.ref_path.append([x, y, quat2eulers(qw, qx, qy, qz)[2]])

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
            self.get_logger().info(f'Linearized MPC Controller wrong control!')

    def control_timer_callback(self):
        #
        # Calculate control
        #
        if (self.curr_x is None or self.curr_y is None or self.curr_z is None or
                self.curr_qw is None or self.curr_qx is None or self.curr_qy is None or self.curr_qz is None or
                self.ref_path is None):
            return
        # Calculate current yaw angle
        _, _, self.curr_yaw = quat2eulers(self.curr_qw, self.curr_qx, self.curr_qy, self.curr_qz)
        dt = 0.1

        # Calculate current speed and angular velocity
        if len(self.ref_path) > 1:
            dx = self.ref_path[1][0] - self.ref_path[0][0]
            dy = self.ref_path[1][1] - self.ref_path[0][1]
            self.ref_yaw = math.atan2(dy, dx)
            current_time = self.get_clock().now()
            duration = current_time - self.last_control_update_time
            time_elapsed = duration.nanoseconds / 1e9
            self.current_speed = math.sqrt(dx ** 2 + dy ** 2) / time_elapsed
            self.angular_velocity = (self.ref_yaw - self.curr_yaw) / time_elapsed
        else:
            self.current_speed = 0
            self.angular_velocity = 0
        self.last_control_update_time = self.get_clock().now()

        if self.acceleration is not None and self.theta is not None:
            A = np.array([[1, 0, dt * math.cos(self.curr_yaw), -dt * self.current_speed * math.sin(self.curr_yaw)],
                          [0, 1, dt * math.sin(self.curr_yaw), dt * self.current_speed * math.cos(self.curr_yaw)],
                          [0, 0, 1, 0],
                          [0, 0, (dt * math.tan(self.theta)) / self.wheelbase, 1]])
        else:
            A = np.zeros((4, 4))

        if self.theta is not None:
            B = np.array([[0, 0],
                          [0, 0],
                          [dt, 0],
                          [0, dt * self.current_speed / (self.wheelbase * math.cos(self.theta) ** 2)]])
        else:
            B = np.zeros((4, 2))

        # Q - weights on the state
        # R - weights on control input. Large R if there is a limit on control output signal
        Q = np.diag([1, 1, 0.5, 0.5])
        R = np.diag([5000, 0.01])

        # Calculate P cost-to-go matrix for DARE
        P = Q
        maxiter = 150
        eps = 0.01

        for i in range(maxiter):
            Pn = Q + A.T @ P @ A - A.T @ P @ B @ inv(R + B.T @ B) @ B.T @ P @ A
            if (abs(Pn - P)).max() < eps:
                break
            P = Pn

        # Calculate feedback gain for DARE
        K = -inv(R + B.T @ P @ B) @ B.T @ P @ A
        x_error = np.array([self.curr_x - self.ref_path[0][0],
                            self.curr_y - self.ref_path[0][1],
                            self.current_speed,
                            self.curr_yaw - self.ref_path[0][2]])

        u_optimal = K @ x_error
        self.get_logger().info(f'K : {K}')
        self.get_logger().info(f'u_optimal: : {u_optimal}')

        self.theta = u_optimal[1]
        self.theta = clip(self.theta, -1.0, 1.0)
        self.acceleration = -u_optimal[0]
        self.acceleration = clip(self.acceleration, -1.0, 0.2)


def main(args=None):
    rclpy.init(args=args)
    linMPCController = linMPCNode()
    rclpy.spin(linMPCController)
    linMPCController.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
