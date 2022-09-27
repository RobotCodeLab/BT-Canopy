# BT-Canopy

Canopy is a tool for running code coverage on Behavior Trees (BTs) in ROS2.

For a BT library to work with Canopy, that library must be able to publish its BT state information in the format provided in the [tree_msgs package](src/tree_msgs). Out of the box, Canopy supports [BehaviorTree.cpp](https://github.com/BehaviorTree/BehaviorTree.CPP) and [py_trees_ros](https://github.com/splintered-reality/py_trees_ros) as detailed below. 


# To enable py_trees_ros logging

- The user need only launch the included republisher node alongside the main monitoring node
- 
  `ros2 launch py_trees_ros_publisher publisher.launch.py`

# To enable BehaviorTree.CPP logging

- The user needs to import the included BTRosPublisher logger and initialize it after the behaviortree_cpp_v3 tree is initialized. For an example, see the [working example](https://github.com/RobotCodeLab/navigation2/commit/4be5f56f705cb31cd7db207b251a7cc702465d4c)

# To build and launch just Canopy

- To ignore BehaviorTree.CPP in build
  `touch src/behaviortree_cpp_v3_ros2_publisher/COLCON_IGNORE`

- To ignore py_trees_ros in build
  `touch src/py_trees_ros_publisher/COLCON_IGNORE`

- Build
`colcon build --symlink-install`

- Run the main logging node
`. install/local_setup.bash`

`ros2 launch bt_canopy_bringup monitor.launch.py`
