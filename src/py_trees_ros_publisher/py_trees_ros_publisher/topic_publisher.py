import rclpy
from rclpy.node import Node

from tree_msgs.msg import StatusChangeLog
from tree_msgs.msg import NodeStatus

from py_trees_ros_interfaces.msg import BehaviourTree
import py_trees_ros_interfaces.msg as py_trees_msgs  # noqa
import py_trees_ros_interfaces.srv as py_trees_srvs  # noqa

import py_trees_ros
import time

class Stream():

    def __init__(self, node : rclpy.Node, open_service_name, close_service_name) -> None:        
        
        self.service_types = {
            'open': py_trees_srvs.OpenSnapshotStream,
            'close': py_trees_srvs.CloseSnapshotStream
        }

        self.node = node

        self.most_recent_time = None

        self.open_service_name = open_service_name
        self.open_service = self.node.create_client(
            self.service_types["open"],
            self.open_service_name
            )

        self.close_service_name = close_service_name
        self.close_service = self.node.create_client(
            self.service_types["close"],
            self.close_service_name
            )

        self.stream_topic_name = "ERROR- Not set"

        self.publish_period = 2.0
        self.subscriber = None

    def open_stream(self, callback):

        request = self.open_service.Request()
        request.parameters.blackboard_data = False
        request.parameters.blackboard_activity = False
        # request.parameters.snapshot_period = float('inf')
        request.parameters.snapshot_period = self.publish_period

        future = self.open_service.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        self.stream_topic_name = response.topic_name

        self.subscriber = self.node.create_subscription(
            msg_type=py_trees_msgs.BehaviourTree,
            topic=self.stream_topic_name,
            callback=callback
        )
        self.most_recent_time = self.node.get_clock().now()

        return self.stream_topic_name

    def close_stream(self):

        # check if service still exists
        service_names_and_types = self.node.get_service_names_and_types()
        service_names = [name for name, types in service_names_and_types ]
        
        if self.close_service_name in service_names:

            request = self.service_types["close"].Request()
            request.topic_name = self.stream_topic_name
            future = self.close_service.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            response = future.result()

class TopicPublisher(Node):

    def __init__(self):
        super().__init__('py_trees_ros2_publisher')

        self.publisher = self.create_publisher(StatusChangeLog, 'status_change_log', 10)
        
        self.open_service_names = []
        self.close_service_names = []

        self.clock = rclpy.clock.Clock()

        self.streams = {} # {stream_topic_name: Stream()}

        # for stream_name in stream_names:
        #     self.create_publisher(
        #         topic_name=stream_name,
        #         msg_type=py_trees_msgs.BehaviourTree,
        #         callback=self.republisher_callback)

    def republisher_callback(self, msg):
        print(msg)

    def tree_discovery(self):

        for stream in self.streams.values():
            if stream.most_recent_time - self.get_clock().now() > stream.publish_period + 0.2:
                print("Stream {} has not been updated in {} seconds. Closing stream.".format(stream.stream_topic_name, stream.publish_period))
                stream.close_stream(self)
                del self.streams[stream.stream_topic_name]
                
        service_names_and_types = self.get_service_names_and_types()
        
        new_open_service_name = None
        new_close_service_name = None

        for service_name, service_types in service_names_and_types:
            if py_trees_srvs.OpenSnapshotStream in service_types:
                if not new_open_service_name and service_name not in self.streams.keys():

                    new_open_service_name = service_name
                    
            if py_trees_srvs.CloseSnapshotStream in service_types:
                if not new_close_service_name and service_name not in self.streams.keys():

                    new_close_service_name = service_name

            if new_open_service_name and new_close_service_name:
                break

        if new_open_service_name and new_close_service_name:

            new_stream = Stream(self, new_open_service_name, new_close_service_name)
            topic_name = new_stream.open_stream(self.republisher_callback)
            self.streams[topic_name] = new_stream

            print("Added new stream: " + topic_name)
            
        elif new_open_service_name or new_close_service_name:
            raise Exception("Missing open or close service for snapshot stream")

                
    def shutdown(self):
        for stream in self.streams.values():
            stream.close_stream(self)
            

def main(args=None):
    rclpy.init(args=args)

    republisher = TopicPublisher()

    while rclpy.ok():
        rclpy.spin_once(republisher)
        republisher.tree_discovery()

    republisher.destroy_node()
    republisher.shutdown()
    rclpy.shutdown()