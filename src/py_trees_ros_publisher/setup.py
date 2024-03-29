from setuptools import setup

package_name = 'py_trees_ros_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/publisher.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ethan N Tran, Samuel Hodges',
    maintainer_email='entran@ncsu.edu, samuelhodges3000@gmail.com',
    description='A py_trees_ros logger for publishing behavior tree info to the ROS2 network.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'topic_publisher = py_trees_ros_publisher.topic_publisher:main'
        ],
    },
)
