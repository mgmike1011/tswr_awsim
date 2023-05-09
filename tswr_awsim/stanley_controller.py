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
def quaternion_rotation_matrix(Q):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix

def rotation_angles(matrix, order):
    """
    input
        matrix = 3x3 rotation matrix (numpy array)
        oreder(str) = rotation order of x, y, z : e.g, rotation XZY -- 'xzy'
    output
        theta1, theta2, theta3 = rotation angles in rotation order
    """
    r11, r12, r13 = matrix[0]
    r21, r22, r23 = matrix[1]
    r31, r32, r33 = matrix[2]

    if order == 'xzx':
        theta1 = np.arctan(r31 / r21)
        theta2 = np.arctan(r21 / (r11 * np.cos(theta1)))
        theta3 = np.arctan(-r13 / r12)

    elif order == 'xyx':
        theta1 = np.arctan(-r21 / r31)
        theta2 = np.arctan(-r31 / (r11 *np.cos(theta1)))
        theta3 = np.arctan(r12 / r13)

    elif order == 'yxy':
        theta1 = np.arctan(r12 / r32)
        theta2 = np.arctan(r32 / (r22 *np.cos(theta1)))
        theta3 = np.arctan(-r21 / r23)

    elif order == 'yzy':
        theta1 = np.arctan(-r32 / r12)
        theta2 = np.arctan(-r12 / (r22 *np.cos(theta1)))
        theta3 = np.arctan(r23 / r21)

    elif order == 'zyz':
        theta1 = np.arctan(r23 / r13)
        theta2 = np.arctan(r13 / (r33 *np.cos(theta1)))
        theta3 = np.arctan(-r32 / r31)

    elif order == 'zxz':
        theta1 = np.arctan(-r13 / r23)
        theta2 = np.arctan(-r23 / (r33 *np.cos(theta1)))
        theta3 = np.arctan(r31 / r32)

    elif order == 'xzy':
        theta1 = np.arctan(r32 / r22)
        theta2 = np.arctan(-r12 * np.cos(theta1) / r22)
        theta3 = np.arctan(r13 / r11)

    elif order == 'xyz':
        theta1 = np.arctan(-r23 / r33)
        theta2 = np.arctan(r13 * np.cos(theta1) / r33)
        theta3 = np.arctan(-r12 / r11)

    elif order == 'yxz':
        theta1 = np.arctan(r13 / r33)
        theta2 = np.arctan(-r23 * np.cos(theta1) / r33)
        theta3 = np.arctan(r21 / r22)

    elif order == 'yzx':
        theta1 = np.arctan(-r31 / r11)
        theta2 = np.arctan(r21 * np.cos(theta1) / r11)
        theta3 = np.arctan(-r23 / r22)

    elif order == 'zyx':
        theta1 = np.arctan(r21 / r11)
        theta2 = np.arctan(-r31 * np.cos(theta1) / r11)
        theta3 = np.arctan(r32 / r33)

    elif order == 'zxy':
        theta1 = np.arctan(-r12 / r22)
        theta2 = np.arctan(r32 * np.cos(theta1) / r22)
        theta3 = np.arctan(-r31 / r33)

    theta1 = theta1 * 180 / np.pi
    theta2 = theta2 * 180 / np.pi
    theta3 = theta3 * 180 / np.pi

    return (theta1, theta2, theta3)
class StanleyControllerNode(Node):

    def __init__(self):
        super().__init__('stanley_controller')
        self.get_logger().info('Stanley Controller start!')
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
        control_publisher_timer_period = 1/60  # seconds
        self.control_publisher_timer = self.create_timer(control_publisher_timer_period, self.control_publisher_timer_callback)
        control_timer = 0.1 # seconds
        self.control_timer = self.create_timer(control_timer, self.control_timer_callback)
        # Steering angle
        self.theta = None 
        # Acceleration
        self.acceleration = None 
        # Controller parameter
        self.K = 3.0
        self.last_u = 0
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
        acc.lateral.steering_tire_angle = theta
        acc.longitudinal.acceleration = accel
        self.control_publisher.publish(acc)

    def control_publisher_timer_callback(self):
        if (self.theta is not None) and (self.acceleration is not None):
            self.publish_control(self.theta, self.acceleration)
            self.get_logger().info(f'Controller output: theta: {self.theta}, acceleration: {self.acceleration}')
        # else:
            # self.get_logger().info(f'Stanley Controller wrong control!')

    def control_timer_callback(self):
        # 
        # Calculate control
        # 
        if (self.ref_path is not None) and (self.curr_x is not None):
            # _, _, yaw_curr = quat2eulers(self.curr_qw, self.curr_qx, self.curr_qy, self.curr_qz)
            # _, _, yaw_traj = quat2eulers(self.ref_path[0][5], self.ref_path[0][2], self.ref_path[0][3], self.ref_path[0][4])
            # if hasattr(yaw_curr, "__len__"):
            #     yaw_curr = 0
            # if hasattr(yaw_traj, "__len__"):
            #     yaw_traj = 0
            Ra = quaternion_rotation_matrix([self.ref_path[0][5], self.ref_path[0][2], self.ref_path[0][3], self.ref_path[0][4]])# trajektoria referencyjna
            Rb = quaternion_rotation_matrix([self.curr_qw, self.curr_qx, self.curr_qy, self.curr_qz])
            R = np.transpose(Ra)@Rb
            # roll, pitch, yaw = rotation_angles(R, 'xyz')
            yaw = -np.arctan2(R[1,0],R[0,0])
            print(f'yaw: {yaw}')
            # yaw_traj = 2*np.arccos((self.ref_path[0][5]/np.sqrt(self.ref_path[0][5]**2 + self.ref_path[0][4]**2 + self.ref_path[0][3]**2 + self.ref_path[0][2]**2)))
            # # if yaw_curr > 2*np:
            # #     yaw_curr = yaw_curr/(2*np)
            # yaw_curr = 2*np.arccos((self.curr_qw/np.sqrt(self.curr_qw**2 + self.curr_qx**2 + self.curr_qy**2 + self.curr_qz**2)))
            # if math.isnan(yaw_curr):
            #     yaw_curr = 0
            # psi = yaw_traj - yaw_curr
            # psi = yaw
            # # e = 2*np.arccos(abs(self.curr_qw*self.ref_path[0][5] + self.curr_qx*self.ref_path[0][2] + self.curr_qy*self.ref_path[0][3]+ self.curr_qz*self.ref_path[0][4]))
            # # if e < -6:
            # #     e = e/(2*np.pi)
            # e = yaw - self.last_u
            delta2 = np.arctan(self.K*yaw)
            # # self.get_logger().info(f'yaw_curr: {yaw_curr}, yaw_traj: {yaw_traj}, e: {e}') 
            # # if (e + delta2) < -1:
            # #     self.theta = -1.0
            # # elif(e + delta2) > 1:
            # #     self.theta = 1.0
            # # else:
            
            # # if ( psi + delta2 ) < -1.5:
            # #     self.theta = -1.5
            # # elif( psi + delta2) > 1.5:
            # #     self.theta = 1.5
            # # else:
            
            self.theta = yaw + delta2
            # self.last_u = self.theta
            self.acceleration = 0.3

    
def main(args=None):
    rclpy.init(args=args)
    StanleyController = StanleyControllerNode()
    rclpy.spin(StanleyController)
    StanleyController.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()