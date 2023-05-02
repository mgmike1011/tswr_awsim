import rclpy
import csv
import os
import numpy as np
from rclpy.node import Node
from launch_ros.substitutions import FindPackageShare

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class PathPublisher(Node):

    def __init__(self):
        super().__init__('path_publisher')

        # Load path from csv file
        pkg_share = FindPackageShare('tswr_awsim').find('tswr_awsim')
        csv_file = os.path.join(pkg_share, 'path.csv')
        print(csv_file)
        filename = csv_file
        
        # Initializing the titles and rows list
        self.path = []
        
        # Reading csv file
        self.path_points = 0
        with open(filename, 'r') as csvfile:
            csvreader = csv.reader(csvfile)
            fields = next(csvreader)
        
            for row in csvreader:
                self.path.append(row)

            self.path_points = csvreader.line_num
            self.get_logger().info("Loaded csv path with a total of %d path points"%(self.path_points))

        # Create ground truth pose subscriber
        self.subscription = self.create_subscription(
            PoseStamped,
            '/ground_truth/pose',
            self.listener_callback,
            10)
        self.subscription

        # Create path publisher
        self.publisher_ = self.create_publisher(Path, '/path_points', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.i = 0

    def listener_callback(self, msg):

        delta_x = float(self.path[self.i % self.path_points][0]) - msg.pose.position.x
        delta_y = float(self.path[self.i % self.path_points][1]) - msg.pose.position.y
        distance = np.sqrt(delta_x ** 2 + delta_y ** 2)

        if distance <= 0.2:
            self.i += 1

        self.get_logger().info("Point nr: %f, distance: %f"%(self.i, distance))

    def timer_callback(self):
        path = Path()
        points = [PoseStamped(), PoseStamped(), PoseStamped()]

        for i in range(len(points)):
            row = self.path[(self.i + i) % self.path_points]
            points[i].pose.position.x = float(row[0])
            points[i].pose.position.y = float(row[1])
            points[i].pose.orientation.x = float(row[2])
            points[i].pose.orientation.y = float(row[3])
            points[i].pose.orientation.z = float(row[4])
            points[i].pose.orientation.w = float(row[5])

        path.poses = points

        self.publisher_.publish(path)

        # self.get_logger().info('Publishing point no %f: %f'%(self.i % self.path_points, float(self.path[self.i % self.path_points][0])))
        # self.i += 10


def main(args=None):

    rclpy.init(args=args)

    minimal_subscriber = PathPublisher()

    rclpy.spin(minimal_subscriber)

    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()