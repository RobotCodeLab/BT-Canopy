import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python import get_package_share_directory

def generate_launch_description():

    republisher_node = Node(
        package='py_trees_ros_publisher',
        executable='topic_publisher',
        name='py_trees_ros_topic_publisher_node',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        republisher_node

    ])