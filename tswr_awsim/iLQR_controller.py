import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand 
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, QoSDurabilityPolicy
import numpy as np
from pydrake.all import (
    Variable,
    SymbolicVectorSystem,
    DiagramBuilder,
    LogVectorOutput,
    Simulator,
    ConstantVectorSource,
    MathematicalProgram,
    Solve,
    SnoptSolver,
    PiecewisePolynomial,
)
import pydrake.symbolic as sym


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
    roll = np.arctan2(
        2 * ((q2 * q3) + (q0 * q1)),
        q0 ** 2 - q1 ** 2 - q2 ** 2 + q3 ** 2
    )  # radians
    pitch = np.arcsin(2 * ((q1 * q3) - (q0 * q2)))
    yaw = np.arctan2(
        2 * ((q1 * q2) + (q0 * q3)),
        q0 ** 2 + q1 ** 2 - q2 ** 2 - q3 ** 2
    )
    return (roll, pitch, yaw)


class derivatives:
    def __init__(self, discrete_dynamics, cost_stage, cost_final, n_x, n_u):
        self.x_sym = np.array(
            [sym.Variable("x_{}".format(i)) for i in range(n_x)]
        )
        self.u_sym = np.array(
            [sym.Variable("u_{}".format(i)) for i in range(n_u)]
        )
        x = self.x_sym
        u = self.u_sym

        l = cost_stage(x, u)
        self.l_x = sym.Jacobian([l], x).ravel()
        self.l_u = sym.Jacobian([l], u).ravel()
        self.l_xx = sym.Jacobian(self.l_x, x)
        self.l_ux = sym.Jacobian(self.l_u, x)
        self.l_uu = sym.Jacobian(self.l_u, u)

        l_final = cost_final(x)
        self.l_final_x = sym.Jacobian([l_final], x).ravel()
        self.l_final_xx = sym.Jacobian(self.l_final_x, x)

        f = discrete_dynamics(x, u)
        self.f_x = sym.Jacobian(f, x)
        self.f_u = sym.Jacobian(f, u)

    def stage(self, x, u):
        env = {self.x_sym[i]: x[i] for i in range(x.shape[0])}
        env.update({self.u_sym[i]: u[i] for i in range(u.shape[0])})
        l_x = sym.Evaluate(self.l_x, env).ravel()
        l_u = sym.Evaluate(self.l_u, env).ravel()
        l_xx = sym.Evaluate(self.l_xx, env)
        l_ux = sym.Evaluate(self.l_ux, env)
        l_uu = sym.Evaluate(self.l_uu, env)

        f_x = sym.Evaluate(self.f_x, env)
        f_u = sym.Evaluate(self.f_u, env)

        return l_x, l_u, l_xx, l_ux, l_uu, f_x, f_u

    def final(self, x):
        env = {self.x_sym[i]: x[i] for i in range(x.shape[0])}

        l_final_x = sym.Evaluate(self.l_final_x, env).ravel()
        l_final_xx = sym.Evaluate(self.l_final_xx, env)

        return l_final_x, l_final_xx


class iLQRNode(Node):

    def __init__(self):
        super().__init__('iLQR_controller')
        self.get_logger().info('iLQR Controller start!')
        # 
        # Current pose
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

        # Reference trajectory
        self.reference_trajectory_subscription = self.create_subscription(Path, '/path_points', self.reference_trajectory_listener_callback, 10)

        # Position
        self.ref_path = None

        # Control variables
        self.theta = None
        self.acceleration = None
        self.time_elapsed = 0.0

        # Control publisher
        qos_policy = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                depth=10)
        self.control_publisher = self.create_publisher(AckermannControlCommand, '/control/command/control_cmd',
                                                       qos_policy)
        control_publisher_timer_period = 1 / 30
        self.control_publisher_timer = self.create_timer(control_publisher_timer_period,
                                                         self.control_publisher_timer_callback)
        control_timer = 0.1
        self.control_timer = self.create_timer(control_timer, self.control_timer_callback)

        # iLQR Variables
        self.n_x = 4
        self.n_u = 2

        self.N = 50
        self.x0 = np.array([0, 0, 0, 0])
        self.u_trj = np.zeros((self.N - 1, self.n_u))
        self.x_trj = self.rollout(self.x0, self.u_trj)

        self.eps = 1e-6

        self.derivs = derivatives(self.discrete_dynamics, self.cost_stage, self.cost_final, self.n_x, self.n_u)

        self.wheelbase = 0.33
        self.last_control_update_time = self.get_clock().now()

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
            self.get_logger().info(f'Controller output: theta: {self.theta}, acceleration: {self.acceleration}, curr_yaw: {self.curr_yaw}')

    def control_timer_callback(self):
        # Calculate control
        if (self.curr_x is None or self.curr_y is None or self.curr_z is None or
                self.curr_qw is None or self.curr_qx is None or self.curr_qy is None or self.curr_qz is None or
                self.ref_path is None):
            return
        # Calculate current yaw angle
        _, _, self.curr_yaw = quat2eulers(self.curr_qw, self.curr_qx, self.curr_qy, self.curr_qz)

        # Calculate current speed and angular velocity
        if len(self.ref_path) > 1:
            dx = self.ref_path[1][0] - self.ref_path[0][0]
            dy = self.ref_path[1][1] - self.ref_path[0][1]
            self.ref_yaw = np.arctan2(dy, dx)
            current_time = self.get_clock().now()
            duration = current_time - self.last_control_update_time
            self.time_elapsed = duration.nanoseconds / 1e9
            self.current_speed = np.sqrt(dx ** 2 + dy ** 2) / self.time_elapsed
            self.angular_velocity = (self.ref_yaw - self.curr_yaw) / self.time_elapsed
        else:
            self.current_speed = 0
            self.angular_velocity = 0
        self.last_control_update_time = self.get_clock().now()

        # Setup problem and call iLQR
        x0 = np.array([self.curr_x, self.curr_y, self.current_speed, self.curr_yaw])
        # x0 = np.array([self.curr_x, self.curr_y, 0.0, 0.0])
        N = 150
        max_iter = 50
        regu_init = 1000
        x_trj, u_trj, cost_trace, regu_trace, redu_ratio_trace, redu_trace = self.run_ilqr(
            x0, N, max_iter, regu_init
        )

        self.get_logger().info(f'u_optimal: : {u_trj[0]}')
        self.get_logger().info(f'Curr: pos: {[self.curr_x, self.curr_y]}, pred: {x_trj[0]}')
        self.get_logger().info(f'Next: pos: {[self.ref_path[0][0], self.ref_path[0][1]]}, pred: {x_trj[-1]}')

        self.theta = np.clip(u_trj[0][1], -1.0, 1.0)

        self.acceleration = np.clip(u_trj[0][0], -1.0, 0.1)

    def discrete_dynamics(self, x, u):
        dt = 0.01

        m = sym if x.dtype == object else np  # Check type for autodiff

        if m == np:
            # Select max values
            u[0] = m.clip(u[0], -0.1, 0.1)
            u[1] = m.clip(u[1], -1.0, 1.0)

        current_speed = x[2]
        curr_yaw = x[3]

        if self.acceleration is not None and self.theta is not None:
            A = np.array([[1, 0, dt * m.cos(curr_yaw), -dt * current_speed * m.sin(curr_yaw)],
                          [0, 1, dt * m.sin(curr_yaw), dt * current_speed * m.cos(curr_yaw)],
                          [0, 0, 1, 0],
                          [0, 0, (dt * m.tan(self.theta)) / self.wheelbase, 1]])
        else:
            A = np.zeros((4, 4))

        if self.theta is not None:
            B = np.array([[0, 0],
                          [0, 0],
                          [dt, 0],
                          [0, dt * current_speed / (self.wheelbase * m.cos(self.theta) ** 2)]])
        else:
            B = np.zeros((4, 2))

        x_dot = A @ x + B @ u
        x_next = x + (dt * x_dot)

        return x_next

    def rollout(self, x0, u_trj):
        x_trj = np.zeros((u_trj.shape[0] + 1, x0.shape[0]))
        # DONE TODO: Define the rollout here and return the state trajectory x_trj: [N, number of states]
        x_trj[0] = x0
        for n in range(u_trj.shape[0]):
            x = x_trj[n]
            u = u_trj[n]

            x_trj[n + 1] = self.discrete_dynamics(x, u)
        # print(x_trj)
        # print()
        # print(u_trj)
        return x_trj
    
    def cost_stage(self, x, u):
        m = sym if x.dtype == object else np  # Check type for autodiff

        if self.ref_path != None:
            c_traj_x = (x[0] - self.ref_path[0][1]) ** 2
            c_traj_y = (x[1] - self.ref_path[0][0]) ** 2
            c_vel = (x[2] - 1) ** 2
            c_traj_yaw = (x[3] - self.ref_path[0][2]) ** 2
            c_control = (u[0] ** 2 + u[1] ** 2) * 0.1
            return 1.5 * c_traj_x + 1.5 * c_traj_y + c_vel + c_traj_yaw + c_control
        else:
            return 0.0

    def cost_final(self, x):
        m = sym if x.dtype == object else np  # Check type for autodiff

        if self.ref_path != None:
            c_traj_x = (x[0] - self.ref_path[0][0]) ** 2
            c_traj_y = (x[1] - self.ref_path[0][1]) ** 2
            c_vel = (x[2] - 1) ** 2
            c_traj_yaw = (x[3] - self.ref_path[0][2]) ** 2
            return 1.5 * c_traj_x + 1.5 * c_traj_y + c_traj_yaw + c_vel
        else:
            return 0.0

    def cost_trj(self, x_trj, u_trj):
        total = 0.0
        # DONE TODO: Sum up all costs
        for n in range(len(u_trj)):
            total += self.cost_stage(x_trj[n], u_trj[n])
        total += self.cost_final(x_trj[-1])
        # print(u_trj.shape[0])
        # print(x_trj.shape[0])
        # print('total', total)
        return total
    
    def Q_terms(self, l_x, l_u, l_xx, l_ux, l_uu, f_x, f_u, V_x, V_xx):
        # DONE TODO: Define the Q-terms here
        Q_x = np.zeros(l_x.shape)
        Q_u = np.zeros(l_u.shape)
        Q_xx = np.zeros(l_xx.shape)
        Q_ux = np.zeros(l_ux.shape)
        Q_uu = np.zeros(l_uu.shape)

        Q_x = l_x + f_x.T.dot(V_x)
        Q_u = l_u + f_u.T.dot(V_x)
        Q_xx = l_xx + f_x.T.dot(V_xx).dot(f_x)
        Q_ux = l_ux + f_u.T.dot(V_xx).dot(f_x)
        Q_uu = l_uu + f_u.T.dot(V_xx).dot(f_u)

        # MC
        # Q_x = l_x + V_x.T @ f_x
        # Q_u = l_u + V_x.T @ f_u
        # Q_xx = l_xx + f_x.T @ V_xx @ f_x
        # Q_ux = l_ux + f_u.T @ V_xx @ f_x
        # Q_uu = l_uu + f_u.T @ V_xx @ f_u

        return Q_x, Q_u, Q_xx, Q_ux, Q_uu

    def gains(self, Q_uu, Q_u, Q_ux):
        Q_uu_inv = np.linalg.inv(Q_uu)
        # TOD: Implement the feedforward gain k and feedback gain K.
        k = np.zeros(Q_u.shape)
        K = np.zeros(Q_ux.shape)
        k = -np.dot(Q_uu_inv, Q_u)
        K = -np.dot(Q_uu_inv, Q_ux)
        return k, K

    def V_terms(self, Q_x, Q_u, Q_xx, Q_ux, Q_uu, K, k):
        # TODO: Implement V_x and V_xx, hint: use the A.dot(B) function for matrix multiplcation.
        V_x = np.zeros(Q_x.shape)
        V_x = Q_x + Q_ux.T @ k + K.T @ Q_u + K.T @ Q_uu @ k
        V_xx = np.zeros(Q_xx.shape)
        V_xx = Q_xx + Q_ux.T @ K + K.T @ Q_ux + K.T @ Q_uu @ K
        return V_x, V_xx

    def expected_cost_reduction(self, Q_u, Q_uu, k):
        return -Q_u.T.dot(k) - 0.5 * k.T.dot(Q_uu.dot(k))
    
    def forward_pass(self, x_trj, u_trj, k_trj, K_trj):
        x_trj_new = np.zeros(x_trj.shape)
        x_trj_new[0, :] = x_trj[0, :]
        u_trj_new = np.zeros(u_trj.shape)
        # TODO: Implement the forward pass here
        #     for n in range(u_trj.shape[0]):
        #         u_trj_new[n,:] = # Apply feedback law
        #         x_trj_new[n+1,:] = # Apply dynamics
        for n in range(u_trj.shape[0]):
            u_trj_new[n, :] = u_trj[n, :] + k_trj[n, :] + K_trj[n, :] @ (x_trj_new[n, :] - x_trj[n, :])
            x_trj_new[n+1, :] = self.discrete_dynamics(x_trj_new[n, :], u_trj_new[n, :])
        return x_trj_new, u_trj_new

    def backward_pass(self, x_trj, u_trj, regu):
        k_trj = np.zeros([u_trj.shape[0], u_trj.shape[1]])
        K_trj = np.zeros([u_trj.shape[0], u_trj.shape[1], x_trj.shape[1]])
        expected_cost_redu = 0
        # TODO: Set terminal boundary condition here (V_x, V_xx)
        derivs = derivatives(self.discrete_dynamics, self.cost_stage, self.cost_final, x_trj.shape[1], u_trj.shape[1])
        V_x = np.zeros((x_trj.shape[1],))
        V_xx = np.zeros((x_trj.shape[1], x_trj.shape[1]))
        V_x, V_xx = derivs.final(x_trj[-1, :])
        for n in range(u_trj.shape[0] - 1, -1, -1):
            # TODO: First compute derivatives, then the Q-terms
            l_x, l_u, l_xx, l_ux, l_uu, f_x, f_u = derivs.stage(x_trj[n, :], u_trj[n, :])
            Q_x, Q_u, Q_xx, Q_ux, Q_uu = self.Q_terms(l_x, l_u, l_xx, l_ux, l_uu, f_x, f_u, V_x, V_xx)
            # Q_x = np.zeros((x_trj.shape[1],))
            # Q_u = np.zeros((u_trj.shape[1],))
            # Q_xx = np.zeros((x_trj.shape[1], x_trj.shape[1]))
            # Q_ux = np.zeros((u_trj.shape[1], x_trj.shape[1]))
            # Q_uu = np.zeros((u_trj.shape[1], u_trj.shape[1]))
            # We add regularization to ensure that Q_uu is invertible and nicely conditioned
            Q_uu_regu = Q_uu + np.eye(Q_uu.shape[0]) * regu
            k, K = self.gains(Q_uu_regu, Q_u, Q_ux)
            k_trj[n, :] = k
            K_trj[n, :, :] = K
            V_x, V_xx = self.V_terms(Q_x, Q_u, Q_xx, Q_ux, Q_uu, K, k)
            expected_cost_redu += self.expected_cost_reduction(Q_u, Q_uu, k)
        return k_trj, K_trj, expected_cost_redu

    def run_ilqr(self, x0, N, max_iter=50, regu_init=100):
        # First forward rollout
        u_trj = np.random.randn(N - 1, self.n_u) * 0.0001
        x_trj = self.rollout(x0, u_trj)
        total_cost = self.cost_trj(x_trj, u_trj)
        regu = regu_init
        max_regu = 10000
        min_regu = 0.01

        # Setup traces
        cost_trace = [total_cost]
        expected_cost_redu_trace = []
        redu_ratio_trace = [1]
        redu_trace = []
        regu_trace = [regu]

        # Run main loop
        for it in range(max_iter):
            # Backward and forward pass
            k_trj, K_trj, expected_cost_redu = self.backward_pass(x_trj, u_trj, regu)
            x_trj_new, u_trj_new = self.forward_pass(x_trj, u_trj, k_trj, K_trj)
            # Evaluate new trajectory
            total_cost = self.cost_trj(x_trj_new, u_trj_new)
            cost_redu = cost_trace[-1] - total_cost
            redu_ratio = cost_redu / abs(expected_cost_redu)
            # Accept or reject iteration
            if cost_redu > 0:
                # Improvement! Accept new trajectories and lower regularization
                redu_ratio_trace.append(redu_ratio)
                cost_trace.append(total_cost)
                x_trj = x_trj_new
                u_trj = u_trj_new
                regu *= 0.7
            else:
                # Reject new trajectories and increase regularization
                regu *= 2.0
                cost_trace.append(cost_trace[-1])
                redu_ratio_trace.append(0)
            regu = min(max(regu, min_regu), max_regu)
            regu_trace.append(regu)
            redu_trace.append(cost_redu)

            # Early termination if expected improvement is small
            if expected_cost_redu <= 1e-6:
                break

        return x_trj, u_trj, cost_trace, regu_trace, redu_ratio_trace, redu_trace

            
def main(args=None):
    rclpy.init(args=args)
    iLQRController = iLQRNode()
    rclpy.spin(iLQRController)
    iLQRController.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()