from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

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
