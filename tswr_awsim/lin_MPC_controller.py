import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, QoSDurabilityPolicy
import numpy as np
from scipy.optimize import minimize


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

        self.u_min = 0.0
        self.u_max = 1.0

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
            self.ref_path.append([x, y, qx, qy, qz, qw])
            # self.ref_path.append(pose)


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
        if (self.ref_path is None) or (self.curr_x is None) or (self.curr_y is None) or (self.curr_qw is None) or (
                self.curr_qx is None) or (self.curr_qy is None) or (self.curr_qz is None):
            return
        # Define the reference states for the MPC algorithm
        ref_states = []
        for point in (self.ref_path):
            x, y, qx, qy, qz, qw = point
            # x_ref = pose.pose.position.x
            # y_ref = pose.pose.position.y
            yaw_ref = quat2eulers(qw, qx, qy, qz)[2]
            v_ref = 1.0
            ref_states.append([x, y, yaw_ref, v_ref])

        # Define the cost function for the MPC algorithm
        Q = np.diag([1.0, 1.0, 1.0, 1.0])
        # R = np.diag([1.0, 1.0])
        R = np.diag([.1, .01])
        N = 3
        cost = lambda x, u, i, ref_states, C: np.dot(np.dot(x.T, Q), x) + np.dot(np.dot(u.T, R), u)

        # Obtain the current states of the vehicle
        roll, pitch, yaw = quat2eulers(self.curr_qw, self.curr_qx, self.curr_qy, self.curr_qz)
        x = np.array([self.curr_x, self.curr_y, yaw, 1.0])

        # Calculate the distance and heading error from the reference path
        distance_error = []
        heading_error = []
        for ref_state in ref_states:
            dx = x[0] - ref_state[0]
            dy = x[1] - ref_state[1]
            d = np.sqrt(dx ** 2 + dy ** 2)
            distance_error.append(d)
            h = np.arctan2(dy, dx) - ref_state[2]
            heading_error.append(h)

        # Define the A, B, and C matrices for the linearized kinematics bicycle model of the vehicle
        k = 0.33  # distance between the front and rear axles
        v = 5.0  # velocity of the vehicle
        A = np.array([[0, 1, 0, 0],
                      [0, 0, v, 0],
                      [0, 0, 0, 1],
                      [0, 0, 0, 0]])
        B = np.array([[0, 0],
                      [0, v],
                      [0, 0],
                      [v / k, 0]])
        C = np.array([[1, 0, 0, 0],
                      [0, 0, 1, 0]])

        # Modify the A, B, and C matrices based on the distance and heading error
        d = distance_error[0]
        h = heading_error[0]
        A[0, 1] = v * np.cos(h)
        A[2, 3] = v * np.cos(h)
        B[1, 1] = v ** 2 * np.sin(h) / d
        B[3, 0] = v ** 2 * np.sin(h) / (k * d)
        C[0, 0] = np.cos(h)
        C[1, 0] = -np.sin(h)
        C[0, 2] = d * np.sin(h)
        C[1, 2] = d * np.cos(h)

        # Solve the MPC optimization problem
        u_opt = self.linear_mpc(A, B, C, x, ref_states, N, cost)

        # Compute the steering
        delta = u_opt[0][1]
        self.acceleration = u_opt[1][0]
        # self.acceleration = 1.0
        # Calculate the steering angle delta
        # L = 2.5  # distance between the left and right wheels
        L = 0.23  # distance between the left and right wheels
        delta = np.arctan(L * delta)
        self.theta = delta

        self.get_logger().info(f'u_opt: {u_opt}')


    def linear_mpc(self, A, B, C, x0, ref_states, N, cost):
        """Run linear MPC.

            Args:
                A (np.ndarray): System matrix.
                B (np.ndarray): Input matrix.
                C (np.ndarray): Output matrix.
                x0 (np.ndarray): Initial state.
                ref_states (np.ndarray): Reference state trajectory.
                N (int): Prediction horizon length.
                cost (callable): Cost function.

            Returns:
            u_opt (numpy.ndarray): Optimal input sequence.
            x_opt (numpy.ndarray): Optimal state trajectory.
            cost_opt (float): Optimal cost.

            """
        n = A.shape[0]  # Number of states
        m = B.shape[1]  # Number of inputs

        def objective_function(u):
            x = np.zeros((n, N + 1))
            x[:, 0] = x0
            cost_sum = 0.0

            ref_states_array = np.concatenate(ref_states, axis=0)
            for i in range(N):
                x[:, i + 1] = A @ x[:, i] + B @ u[m * i:m * (i + 1)]
                cost_sum += cost(x[:, i], u[m * i:m * (i + 1)], i, ref_states_array[i], C)

            return cost_sum

        u0 = np.zeros((m * N,))  # Initial guess for u
        bounds = [(self.u_min, self.u_max) for _ in range(m * N)]  # Input bounds

        # Minimize the objective function
        result = minimize(objective_function, u0, bounds=bounds)

        # Extract the optimal control input
        u_opt = np.reshape(result.x, (m, N))

        # Return the optimal control input
        return u_opt


def main(args=None):
    rclpy.init(args=args)
    linMPCController = linMPCNode()
    rclpy.spin(linMPCController)
    linMPCController.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()