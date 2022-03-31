from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='btcpp_ros2_bridge',
            executable='zmq_to_ros_republisher',
            name='zmq_to_ros_republisher_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'pub_port': '1666', 'server_port': '1667', 'server_ip': 'localhost'}
            ],
        ),
        Node(
            package='btcpp_ros2_bridge',
            executable='get_tree_nodes_service',
            name='get_tree_nodes_service',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'pub_port': '1666', 'server_port': '1667', 'server_ip': 'localhost'}
            ],
        ),
    ])