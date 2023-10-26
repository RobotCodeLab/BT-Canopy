# BT-Canopy

Canopy is a tool for running code coverage on behavior trees in ROS2.

For a BT library to work with Canopy, that library must be able to publish its behavior tree state information in the format provided in the [tree_msgs package](src/tree_msgs). Out of the box, Canopy supports [BehaviorTree.cpp](https://github.com/BehaviorTree/BehaviorTree.CPP) (specifically v3) and [py_trees_ros](https://github.com/splintered-reality/py_trees_ros) as detailed below.

## Nav2 Case Study

The instructions/case study results for instrumenting Nav2 using Canopy can be found in <examples/nav2_with_canopy> and the corresponding [README.md](examples/nav2_with_canopy/README.md).

## Instrumenting Logging

Canopy requires no extra instrumentation if the behavior tree being monitored is implemented using `py_trees_ros`. However, if the behavior tree being monitored is implemented using `BehaviorTree.CPP`, some modifications to the code being monitored. This includes adding a logger that publishes to a ROS topic. A basic implementation of this can be found in [behaviortree\_cpp\_v3\_ros2\_publisher](src/behaviortree_cpp_v3_ros2_publisher/).

The project must add `behaviortree_cpp_v3_ros2_publisher` as a dependency for the ROS project.

`CMakeLists.txt`:

```cmake
find_package(behaviortree_cpp_v3_ros2_publisher REQUIRED)
```

`package.xml`:

```xml
<build_depend>behaviortree_cpp_v3_ros2_publisher</build_depend>
<exec_depend>behaviortree_cpp_v3_ros2_publisher</exec_depend>
```

The following code should also be added to the project.

```cpp
std::unique_ptr<BTRosPublisher> logger = std::make_unique<BTRosPublisher>(ros_node, tree, tree_uid);

logger->flush();
```

Where `ros_node` is the ROS node object that's running the behavior tree, `tree` is an instance of the `BehaviorTree.CPP` behavior tree, and `tree_uid` is some unique name for the behavior tree. The `flush` method must be called routinely in order for the logging information to get published to Canopy.

## Enabling `BehaviorTree.CPP` Logging

- The user needs to import the included BTRosPublisher logger and initialize it after the `behaviortree_cpp_v3` tree is initialized. For an example, see the [working example](https://github.com/RobotCodeLab/navigation2/commit/4be5f56f705cb31cd7db207b251a7cc702465d4c)

## Building Canopy

The ROS dependencies should first be installed in the ROS workspace.

```bash
rosdep install --from-paths src --ignore-src -r -y
```

Running the build can be done simply with

```bash
colcon build --symlink-install
```

## Running Canopy

Canopy can now be run simply by using the launch script.

```bash
. install/setup.sh
ros2 launch bt_canopy_bringup monitor.launch.py
```

In order to monitor `py_trees_ros`, the another node must be run in a separate terminal.

```bash
. install/setup.sh
ros2 launch py_trees_ros_publisher publisher.launch.py
```
