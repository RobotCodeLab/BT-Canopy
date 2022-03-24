cmake_minimum_required(VERSION 3.5)
project(btcpp_ros2_bridge)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tree_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

find_package(ZMQ)

ament_export_dependencies(rosidl_default_runtime)


add_executable(zmq_to_ros_republisher src/zmq_to_ros_republisher.cpp)
ament_target_dependencies(zmq_to_ros_republisher rclcpp tree_msgs behaviortree_cpp_v3 ZMQ)

install(TARGETS
  zmq_to_ros_republisher
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
