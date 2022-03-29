import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory

def generate_launch_description():

    bridge_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('btcpp_ros2_bridge'),
                'launch',
                'zmq_ros_republisher.launch.py')
        )
    )

    # coverage_node = Node(
    #     package='btcpp_ros2_bridge',
    #     executable='zmq_to_ros_republisher',
    #     name='zmq_to_ros_republisher_node',
    #     output='screen',
    #     emulate_tty=True,
    #     parameters=[
    #         {'pub_port': '1666', 'server_port': '1667', 'server_ip': 'localhost'}
    #     ],
    # )
    
    return LaunchDescription([
        bridge_node
        # , 
        # coverage_node
    ])