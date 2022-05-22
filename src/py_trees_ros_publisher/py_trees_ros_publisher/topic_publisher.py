import rclpy
from rclpy.node import Node

from tree_msgs.msg import StatusChangeLog
from tree_msgs.msg import NodeStatus

# from py_trees_ros_interfaces.msg import BehaviourTree
import py_trees_ros_interfaces.msg as py_trees_msgs  # noqa
import py_trees_ros_interfaces.srv as py_trees_srvs  # noqa

import py_trees_ros

from tree_msgs.msg import StatusChangeLog
from tree_msgs.msg import StatusChange
from tree_msgs.msg import NodeStatus

from tree_msgs.msg import BehaviorTree
from tree_msgs.msg import TreeNode

import hashlib

class Stream():

    def __init__(self, node : rclpy.node.Node, open_service_name, close_service_name) -> None:        
        
        # print("Stream: initialising with open_service_name: {} and close_service_name: {}".format(open_service_name, close_service_name))

        self.service_types = {
            'open': py_trees_srvs.OpenSnapshotStream,
            'close': py_trees_srvs.CloseSnapshotStream
        }

        self.node = node

        self.most_recent_time = None

        self.open_service_name = open_service_name
        self.open_service = self.node.create_client(
            srv_type=self.service_types["open"],
            srv_name=self.open_service_name,
            qos_profile=rclpy.qos.qos_profile_services_default
            )

        self.close_service_name = close_service_name
        self.close_service = self.node.create_client(
            srv_type=self.service_types["close"],
            srv_name=self.close_service_name,
            qos_profile=rclpy.qos.qos_profile_services_default
            )

        self.stream_topic_name = "ERROR- Not set"

        self.publish_period = 2
        self.subscriber = None
        self.node_callback = None

    def stream_callback(self, msg):
        self.most_recent_time = self.node.get_clock().now()
        self.node_callback(msg, self.stream_topic_name )

    def open_stream(self, callback):

        self.node_callback = callback

        request = self.service_types['open'].Request()
        request.parameters.blackboard_data = False
        request.parameters.blackboard_activity = False
        request.parameters.snapshot_period = float(self.publish_period)

        future = self.open_service.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        self.stream_topic_name = response.topic_name

        self.subscriber = self.node.create_subscription(
            msg_type=py_trees_msgs.BehaviourTree,
            topic=self.stream_topic_name,
            callback=self.stream_callback,
            qos_profile=py_trees_ros.utilities.qos_profile_latched()
        )
        self.most_recent_time = self.node.get_clock().now()

        return self.stream_topic_name

    def close_stream(self):

        self.open_service.destroy()

        request = self.service_types['close'].Request()
        request.topic_name = self.stream_topic_name
        future = self.close_service.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=1)
        if future:
            response = future.result()

        self.close_service.destroy()

class TopicPublisher(Node):

    def __init__(self):
        super().__init__('py_trees_ros2_publisher')

        self.publisher = self.create_publisher(StatusChangeLog, 'bt_status_change_log', 10)
        
        self.open_service_names = set()
        self.close_service_names = set()

        self.clock = rclpy.clock.Clock()

        self.previous_status = {} # {stream_topic_name: {uid: NodeStatus}}

        self.streams = {} # {stream_topic_name: Stream()}

    def uuid_to_int32(self, uuid):
        # concatenate the 16 bytes into a single integer
        return int.from_bytes(list(uuid)[:4], byteorder='big')

    # called by the stream object
    def republisher_callback(self, msg : py_trees_msgs.BehaviourTree, stream_topic_name):

        if not msg.changed:
            return

        if self.previous_status.get(stream_topic_name) is None:
            self.previous_status[stream_topic_name] = {}

        canopy_tree_msg = BehaviorTree()

        status_change_log_msg = StatusChangeLog()

        tree_shape_uid = ""
        root_uid_set = False

        for behavior in msg.behaviours:

            tree_node = TreeNode()

            # hash the list of 16 bytes to get a unique id
            tree_node.uid = self.uuid_to_int32(behavior.own_id.uuid)

            if behavior.is_active:
                status_change_msg = StatusChange()
                status_change_msg.uid = tree_node.uid
                status_change_msg.prev_status = self.previous_status[stream_topic_name].get(tree_node.uid, NodeStatus(value=0))
                status_change_msg.status.value = behavior.status - 1

                # add the current status to the previous status
                self.previous_status[stream_topic_name][tree_node.uid] = status_change_msg.status

                status_change_log_msg.state_changes.append(status_change_msg)

            # set root node if no parents
            if not root_uid_set and list(behavior.parent_id.uuid) == [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]:
                canopy_tree_msg.root_uid = tree_node.uid
                root_uid_set = True

            # set child_uids
            for child_id in behavior.child_ids:
                tree_node.child_uids.append(self.uuid_to_int32(child_id.uuid))

            # convert py_trees_type to btcpp type
            if behavior.type == 0:
                tree_node.type = tree_node.UNDEFINED
            elif behavior.type == 1:
                tree_node.type = tree_node.ACTION
            elif behavior.type in [2,3,4,5]:
                tree_node.type = tree_node.CONTROL
            elif behavior.type == 6:
                tree_node.type = tree_node.DECORATOR

            tree_node.instance_name = behavior.name
            tree_node.registration_name = behavior.class_name

            canopy_tree_msg.nodes.append(tree_node)

            # append for tree_uid
            tree_shape_uid += str(tree_node.uid)
            tree_shape_uid += str(tree_node.instance_name)
            tree_shape_uid += "-"

        tree_shape_uid = hashlib.sha1(tree_shape_uid.encode('utf-8')).hexdigest()[:32]

        canopy_tree_msg.tree_uid = str(tree_shape_uid)

        status_change_log_msg.behavior_tree = canopy_tree_msg

        self.publisher.publish(status_change_log_msg)

    def tree_discovery(self):

        streams_to_delete = []
        for stream in self.streams.values():
            if self.get_clock().now() - stream.most_recent_time  > rclpy.duration.Duration(seconds=stream.publish_period,nanoseconds= int(0.2 * 1e9)):
                print("Stream {} has not been updated in {} seconds. Closing stream.".format(stream.stream_topic_name, stream.publish_period))
                stream.close_stream()
                
                streams_to_delete.append(stream.stream_topic_name)

                self.open_service_names.remove(stream.open_service_name)
                self.close_service_names.remove(stream.close_service_name)
                
        for stream_topic_name in streams_to_delete:
            del self.streams[stream_topic_name]
            del self.previous_status[stream_topic_name]
            
        service_names_and_types = self.get_service_names_and_types()
        
        new_open_service_name = None
        new_close_service_name = None

        for service_name, service_types in service_names_and_types:
            if 'py_trees_ros_interfaces/srv/OpenSnapshotStream' in service_types:
                if not new_open_service_name and service_name not in self.open_service_names:

                    new_open_service_name = service_name
                    
            if 'py_trees_ros_interfaces/srv/CloseSnapshotStream' in service_types:
                if not new_close_service_name and service_name not in self.close_service_names:

                    new_close_service_name = service_name

            if new_open_service_name and new_close_service_name:
                break

        if new_open_service_name and new_close_service_name:

            self.open_service_names.add(new_open_service_name)
            self.close_service_names.add(new_close_service_name)

            new_stream = Stream(self, new_open_service_name, new_close_service_name)
            topic_name = new_stream.open_stream(self.republisher_callback)
            self.streams[topic_name] = new_stream

            print("Monitoring stream: " + topic_name)
            
        elif new_open_service_name or new_close_service_name:
            raise Exception("Missing open or close service for snapshot stream")

                
    def shutdown(self):
        for stream in self.streams.values():
            stream.close_stream(self)
            
def main(args=None):
    rclpy.init(args=args)

    republisher = TopicPublisher()

    while rclpy.ok():
        rclpy.spin_once(republisher, timeout_sec=1) # timeout is how often to run tree_discovery (also handles removing deleted trees)
        republisher.tree_discovery()

    republisher.destroy_node()
    republisher.shutdown()
    rclpy.shutdown()