from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    coverage_node = Node(
        package='bt_canopy_bringup',
        executable='coverage_monitor',
        name='coverage_monitor_node',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        coverage_node
    ])
