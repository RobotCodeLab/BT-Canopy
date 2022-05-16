#include "behaviortree_cpp_v3/loggers/abstract_logger.h"
#include "tf2_ros/buffer_interface.h"

#include "tree_msgs/msg/behavior_tree.hpp"
#include "tree_msgs/msg/node_parameter.hpp"
#include "tree_msgs/msg/node_status.hpp"
#include "tree_msgs/msg/status_change.hpp"
#include "tree_msgs/msg/status_change_log.hpp"

class BTRosPublisher : public BT::StatusChangeLogger
{
    public:
        BTRosPublisher(const rclcpp::Node::WeakPtr & ros_node, const BT::Tree & tree)
        : StatusChangeLogger(tree.rootNode())
        {
            auto node = ros_node.lock();
            clock_ = node->get_clock();
            // logger_ = node->get_logger();

            tree_name = tree.rootNode()->registrationName();
            status_change_log_pub_ = node->create_publisher<tree_msgs::msg::StatusChangeLog>(
                "/status_change_log", 10);
        }

        void callback(
            BT::Duration timestamp,
            const BT::TreeNode & node,
            BT::NodeStatus prev_status,
            BT::NodeStatus status) override
        {
            tree_msgs::msg::StatusChange event;

            event.timestamp = tf2_ros::toMsg(tf2::TimePoint(timestamp));
            event.uid = node.UID();

            event.prev_status.value = static_cast<int>(prev_status);
            event.status.value = static_cast<int>(status);
            state_changes.push_back(std::move(event));

        }

        void flush() override
        {
            if (!state_changes.empty()){
                auto msg = std::make_unique<tree_msgs::msg::StatusChangeLog>();
                msg->state_changes = state_changes;
                msg->tree_name = tree_name;
                status_change_log_pub_->publish(std::move(msg));
                state_changes.clear();
            }
        }
        

    protected:
        rclcpp::Clock::SharedPtr clock_;
        // rclcpp::Logger logger_{rclcpp::get_logger("bt_navigator")};
        rclcpp::Publisher<tree_msgs::msg::StatusChangeLog>::SharedPtr status_change_log_pub_;
        std::vector<tree_msgs::msg::StatusChange> state_changes;

        std::string tree_name;
    
};