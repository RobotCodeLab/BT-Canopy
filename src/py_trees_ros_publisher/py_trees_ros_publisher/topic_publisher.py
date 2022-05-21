import rclpy
from rclpy.node import Node

from tree_msgs.msg import StatusChangeLog
from tree_msgs.msg import NodeStatus

import py_trees_ros

# https://github.com/splintered-reality/py_trees_ros/blob/d7e12ed24f6d298dd70b6625e6ae2f3f15afdcdd/py_trees_ros/trees.py#L744
# https://github.com/splintered-reality/py_trees_ros/blob/d7e12ed24f6d298dd70b6625e6ae2f3f15afdcdd/py_trees_ros/programs/tree_watcher.py