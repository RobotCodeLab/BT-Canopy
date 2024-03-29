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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ethan N Tran, Samuel Hodges',
    maintainer_email='entran@ncsu.edu, samuelhodges3000@gmail.com',
    description='The Canopy logger that watches and save changes to behavior tree statuses.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coverage_monitor = bt_canopy_bringup.coverage_monitor:main',
        ],
    },
)
