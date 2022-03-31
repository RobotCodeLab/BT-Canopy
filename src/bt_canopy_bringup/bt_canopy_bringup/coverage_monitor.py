import rclpy
from rclpy.node import Node
# from std_msgs.msg import String

from tree_msgs.msg import BehaviorTreeLog
from tree_msgs.srv import GetTreeNodes
from tree_msgs.msg import TreeNode

class CoverageMonitor(Node):

    def __init__(self):
        super().__init__('coverage_monitor')

        self.tree_shape = None
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

    def log_callback(self,msg): 

        for event in msg.event_log:
            print(event.node_name)

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
                        coverage_monitor.tree_shape = behavior_tree_msg.nodes
                    else:
                        tree_already_requested = False

                except Exception as e:
                    coverage_monitor.get_logger().error('Failed to get tree nodes: %s' % e)
        else:
            print(coverage_monitor.tree_shape.nodes)
                    
    coverage_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()