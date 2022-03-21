cmake_minimum_required(VERSION 3.5)
project(bt_monitor)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(ZMQ)

if( ZMQ_FOUND )
  message(STATUS "ZeroMQ found.")
  add_definitions( -DZMQ_FOUND )

  set(APP_CPPS ${APP_CPPS} ./bt_editor/sidepanel_monitor.cpp )
  set(FORMS_UI ${FORMS_UI} ./bt_editor/sidepanel_monitor.ui )

else()
  message(WARNING "ZeroMQ NOT found. Real-time monitoring disabled.")
endif()

set(msg_files
  "msg/BehaviorTreeLog.msg"
  "msg/BehaviorTreeStatusChange.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(tree_listener src/tree_listener.cpp)
ament_target_dependencies(tree_listener rclcpp)

install(TARGETS
  tree_listener
  DESTINATION lib/${PROJECT_NAME})

rosidl_target_interfaces(tree_listener
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
