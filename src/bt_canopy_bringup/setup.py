from setuptools import setup

package_name = 'bt_canopy_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/monitor.launch.py']),
        ('share/' + package_name, ['launch/monitor_with_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samuel',
    maintainer_email='samuelhodges3000@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coverage_monitor = bt_canopy_bringup.coverage_monitor:main',
        ],
    },
)
