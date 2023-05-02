from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    path_publisher = Node(
        package="tswr_awsim",
        executable="path_publisher"
    )

    controller = Node(
        package="tswr_awsim",
        executable="stanley_controller"
    )

    ld.add_action(path_publisher)
    ld.add_action(controller)
    return ld