import rclpy
import csv
import os
from rclpy.node import Node
from launch_ros.substitutions import FindPackageShare

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/ground_truth/pose',
            self.listener_callback,
            10)
        self.subscription

        self.publisher_ = self.create_publisher(Path, '/path_points', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def listener_callback(self, msg):
        self.get_logger().info(msg.pose.position.x, msg.pose.position.y)

    def timer_callback(self):
        msg = Path()
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing!')


def main(args=None):

    # Load path from csv file
    pkg_share = FindPackageShare('tswr_awsim').find('tswr_awsim')
    csv_file = os.path.join(pkg_share, 'path.csv')
    print(csv_file)
    filename = csv_file
    
    # initializing the titles and rows list
    fields = []
    rows = []
    
    # reading csv file
    with open(filename, 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        fields = next(csvreader)
    
        for row in csvreader:
            rows.append(row)
    
        print("Loaded csv path with a total of %d path points"%(csvreader.line_num))

    rclpy.init(args=args)

    minimal_subscriber = PathPublisher()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()