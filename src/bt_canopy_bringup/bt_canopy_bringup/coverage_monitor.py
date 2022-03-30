import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from tree_msgs.msg import BehaviorTreeLog

class CoverageMonitor(Node):

    def __init__(self):
        super().__init__('coverage_monitor')
        self.subscription = self.create_subscription(
            BehaviorTreeLog,
            'behavior_tree_log',
            self.log_callback,
            10)
        self.subscription  # prevent unused variable warning

    def log_callback(self,msg): 
        for event in msg.event_log:
            print(event.node_name)

def main(args=None):
    rclpy.init(args=args)

    coverage_monitor = CoverageMonitor()

    rclpy.spin(coverage_monitor)

    coverage_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()