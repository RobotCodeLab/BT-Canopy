import rclpy
from rclpy.node import Node

from tree_msgs.msg import StatusChangeLog
from tree_msgs.msg import NodeStatus

from pathlib import Path
import csv
import re

# out_file = "tree"
fields = ['uid', 'node_registration_name',\
    'node_instance_name', 'num_visits', \
        'num_failures','num_successes',\
             'num_running','num_idle']

class BTNode():
        
    def __init__(self, uid, child_uids = None, node_type = None, instance_name=None, registration_name = None, params = None):
        self.uid = uid
        self.child_uids = child_uids
        self.type = node_type
        self.instance_name = instance_name
        self.registration_name = registration_name
        self.params = params
        
        

        self.num_visits = 0
        self.num_failures = 0
        self.num_successes = 0
        self.num_running = 0
        self.num_idle = 0

    def add_status_change_event(self, status):  # status: IDLE, RUNNING, SUCCESS or FAILURE
        
        self.num_visits += 1
        
        if status == NodeStatus.IDLE:
            self.num_idle += 1
        elif status == NodeStatus.RUNNING:
            self.num_running += 1
        elif status == NodeStatus.SUCCESS:
            self.num_successes += 1
        elif status == NodeStatus.FAILURE:
            self.num_failures += 1
        else:
            print('Error: unknown status change event')

class CoverageMonitor(Node):

    def __init__(self):
        super().__init__('coverage_monitor')

        self.trees = {}
        self.trees_changed = {}
        self.trees_out_file = {}

        self.got_tree_nodes = False

        self.stats_updated = False

        self.request = None
        self.future = None

        self.subscription = self.create_subscription(
            StatusChangeLog,
            'bt_status_change_log',
            self.log_callback,
            10)
        self.subscription  # prevent unused variable warning        


    # long tree UIDs may exceed the maximum length of a filename
    # Map the tree UID to a shorter file name
    def _format_out_file(self, tree_uid):

        stem = Path(tree_uid).stem

        stem = re.sub('[^\w_.)( -]', '-', stem)

        if len(stem) > 128:
            stem = stem[:128]

        out_file = 'canopy_' + stem + '.csv'

        # If file name already exists for a different tree_uid, 
        # append a number to the end until a unique name is found
        i = 1
        while out_file in self.trees_out_file.items():
            out_file = 'canopy_' + stem + '_' + str(i) + '.csv'
            i += 1

        return out_file

    def log_callback(self, msg: StatusChangeLog): 

        tree_uid = msg.behavior_tree.tree_uid

        if tree_uid not in self.trees.keys():
            self.trees[tree_uid] = {}

            for TreeNode in msg.behavior_tree.nodes:
                self.trees[tree_uid][TreeNode.uid] \
                    = BTNode(TreeNode.uid, TreeNode.child_uids, TreeNode.type, TreeNode.instance_name, \
                        TreeNode.registration_name, TreeNode.params)
        
            self.trees_out_file[tree_uid] = self._format_out_file(tree_uid) 

        if msg.state_changes:

            self.trees_changed[tree_uid] = True

            for state_change in msg.state_changes:

                self.trees[tree_uid][state_change.uid].add_status_change_event(state_change.status.value) 


def main(args=None):
    rclpy.init(args=args)

    coverage_monitor = CoverageMonitor()

    while rclpy.ok():
        rclpy.spin_once(coverage_monitor)

        for tree_uid, tree in coverage_monitor.trees.items():

            if coverage_monitor.trees_changed[tree_uid]:

                with open(coverage_monitor.trees_out_file[tree_uid], 'w') as csvfile:
                    writer = csv.DictWriter(csvfile, fieldnames=fields)
                    writer.writeheader()

                    for node in tree.values():
                        writer.writerow({'uid': node.uid, 'node_registration_name': node.registration_name, \
                            'node_instance_name': node.instance_name, 'num_visits': node.num_visits, 'num_failures': \
                                node.num_failures, 'num_successes': node.num_successes, 'num_running': node.num_running, \
                                     'num_idle': node.num_idle})

                coverage_monitor.trees_changed[tree_uid] = False

    coverage_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()