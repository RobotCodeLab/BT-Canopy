# nav2 with canopy (from inside container)

`ros2 launch bt_canopy_bringup monitor.launch.py`

`cd ~/nav2_ws/build/nav2_system_tests/ && ctest -V`

# start new container *and* run nav2 system tests with canopy monitoring

`bash start_container_and_run_tests.sh <nav2_with_canopy docker image name>`