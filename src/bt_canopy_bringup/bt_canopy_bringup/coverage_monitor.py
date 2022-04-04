import rclpy
from rclpy.node import Node
# from std_msgs.msg import String

from tree_msgs.msg import BehaviorTreeLog
from tree_msgs.srv import GetTreeNodes
from tree_msgs.msg import TreeNode

import csv

out_file = "node_coverage.csv"
fields = ['node_uid', 'node_registration_id','node_instance_name', 'num_visits', 'num_failures','num_successes', 'num_running','num_idle']

class BTNode():
        
    def __init__(self, node_uid, instance_name, registration_id = None, node_type = None, child_uids = None):
        self.uid = node_uid
        self.instance_name = instance_name
        self.registration_id = registration_id
        self.type = node_type
        self.child_uids = child_uids

        self.num_visits = 0
        self.num_failures = 0
        self.num_successes = 0
        self.num_running = 0
        self.num_idle = 0

    def add_status_change_event(self, current_status):  # current_status: IDLE, RUNNING, SUCCESS or FAILURE
        
        self.num_visits += 1
        
        if current_status == 'IDLE':
            self.num_idle += 1
        elif current_status == 'RUNNING':
            self.num_running += 1
        elif current_status == 'SUCCESS':
            self.num_successes += 1
        elif current_status == 'FAILURE':
            self.num_failures += 1
        else:
            print('Error: unknown status change event')

class CoverageMonitor(Node):

    def __init__(self):
        super().__init__('coverage_monitor')

        self.tree_stats = {} # {node_uid: BTNode}

        # self.tree_shape = None
        self.got_tree_nodes = False
        self.get_tree_client = self.create_client(GetTreeNodes, 'get_tree_nodes')

        self.request = None
        self.future = None

        self.subscription = self.create_subscription(
            BehaviorTreeLog,
            'behavior_tree_log',
            self.log_callback,
            10)
        self.subscription  # prevent unused variable warning

    def request_tree_shape(self):
        self.request = GetTreeNodes.Request()
        self.future = self.get_tree_client.call_async(self.request)

    def log_callback(self, msg: BehaviorTreeLog): 
        for event in msg.event_log:
            # print(event.node_name)

            if event.node_uid not in self.tree_stats.keys():
                self.tree_stats[event.node_uid] = BTNode(event.node_uid, event.node_name)
            else:                
                self.tree_stats[event.node_uid].add_status_change_event(event.current_status)


    def add_tree_nodes(self, tree_nodes):

        for node in tree_nodes:
            
            node_uid = node.uid

            if node_uid not in self.tree_stats.keys():
                self.tree_stats[node_uid] = BTNode(node_uid, node.instance_name, node.registration_id, node.type, node.child_uids)
            else:
                self.tree_stats[node_uid].registration_id = node.registration_id
                self.tree_stats[node_uid].type = node.type
                self.tree_stats[node_uid].child_uids = node.child_uids
        

def main(args=None):
    rclpy.init(args=args)

    coverage_monitor = CoverageMonitor()

    
    tree_already_requested = False

    while rclpy.ok():
        rclpy.spin_once(coverage_monitor)

        if not coverage_monitor.got_tree_nodes:

            if not tree_already_requested and coverage_monitor.get_tree_client.service_is_ready() :
                coverage_monitor.request_tree_shape()
                tree_already_requested = True

            if not coverage_monitor == None and coverage_monitor.future.done():
                try:
                    behavior_tree_msg = coverage_monitor.future.result()

                    if behavior_tree_msg.success:
                        coverage_monitor.got_tree_nodes = True

                        coverage_monitor.add_tree_nodes(behavior_tree_msg.nodes)

                    else:
                        tree_already_requested = False

                except Exception as e:
                    coverage_monitor.get_logger().error('Failed to get tree nodes: %s' % e)
        else:
            # for node in coverage_monitor.tree_stats.values():
            #     print('%s: %s' % (node.instance_name, node.num_visits))

            with open(out_file, 'w') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=fields)
                writer.writeheader()

                for node in coverage_monitor.tree_stats.values():
                    writer.writerow({'node_uid': node.uid, 'node_registration_id': node.registration_id, \
                         'node_instance_name': node.instance_name, 'num_visits': node.num_visits, 'num_failures': \
                              node.num_failures, 'num_successes': node.num_successes, 'num_running': node.num_running, 'num_idle': node.num_idle})
            # pass

    # TODO: add a way to print/save the tree stats

    coverage_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()