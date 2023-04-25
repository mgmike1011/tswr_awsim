import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from autoware_auto_control_msgs.msg import AckermannControlCommand #wysyłanie
from geometry_msgs.msg import PoseStamped #odbieranie

class TestNode(Node):

    def __init__(self):
        super().__init__('test_node')
        self.subscription = self.create_subscription(PoseStamped,'/ground_truth/pose', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(AckermannControlCommand, 'topic', 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.pose)
        x = AckermannControlCommand()
        x.lateral.steering_tire_angle = 1.0
        self.publisher_.publish(x)


def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = TestNode()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()