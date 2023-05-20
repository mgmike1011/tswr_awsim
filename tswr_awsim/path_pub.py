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

        # Initialize variables
        self.radius = 0.1
        self.correct = 0
        self.incorrect = 0
        self.error = 0
        self.mean_error = 0

        # Load path from csv file
        pkg_share = FindPackageShare('tswr_awsim').find('tswr_awsim')
        csv_file = os.path.join(pkg_share, 'path.csv')
        print(csv_file)
        filename = csv_file
        
        # Initializing the titles and rows list
        self.path = []
        self.direction = 0
        
        # Reading csv file
        self.path_points = 0
        with open(filename, 'r') as csvfile:
            csvreader = csv.reader(csvfile)
            fields = next(csvreader)
        
            for row in csvreader:
                self.path.append(row)

            self.path_points = csvreader.line_num - 1
            self.get_logger().info("Loaded csv path with a total of %d path points"%(csvreader.line_num))

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

        # Calcualte distances and vactors
        delta_x = float(self.path[self.i % self.path_points][0]) - msg.pose.position.x
        delta_y = float(self.path[self.i % self.path_points][1]) - msg.pose.position.y
        distance_1 = np.linalg.norm([delta_x, delta_y])
        vect_1 = np.array([delta_x, delta_y]) / np.linalg.norm([delta_x, delta_y])

        delta_x = float(self.path[self.i % self.path_points][0]) - float(self.path[(self.i + 1) % self.path_points][0])
        delta_y = float(self.path[self.i % self.path_points][1]) - float(self.path[(self.i + 1) % self.path_points][1])
        vect_2 = np.array([delta_x, delta_y]) / np.linalg.norm([delta_x, delta_y])

        angle = np.rad2deg(np.arccos(np.clip(np.dot(vect_1, vect_2), -1.0, 1.0)))
        angle2 = np.rad2deg(np.arccos(np.clip(np.dot([vect_1[1], -vect_1[0]], vect_2), -1.0, 1.0)))

        if angle2 >= 90:
            self.direction = 1
        else:
            self.direction = 0

        if distance_1 <= self.radius:
            self.i += 1
            self.correct += 1

        elif angle <= 90.0:
            self.i += 1
            self.incorrect += 1
            self.error += distance_1
            self.mean_error = self.error / self.incorrect

        self.get_logger().info("Loop nr: %f, Total points: %f, Correct: %f, Incorrect: %f, Mean error: %f"%(self.i // self.path_points, self.i, self.correct, self.incorrect, self.mean_error))


    def timer_callback(self):
        path = Path()
        points = [PoseStamped(), PoseStamped(), PoseStamped()]

        for i in range(len(points)):
            row = self.path[(self.i + i) % self.path_points]
            points[i].pose.position.x = float(row[0])
            points[i].pose.position.y = float(row[1])
            points[i].pose.position.z = float(self.direction)
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