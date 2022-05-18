#ifndef CANOPY_BT_ROS_PUBLISHER_HPP_
#define CANOPY_BT_ROS_PUBLISHER_HPP_

#include "behaviortree_cpp_v3/loggers/abstract_logger.h"
#include "tf2_ros/buffer_interface.h"

#include "tree_msgs/msg/behavior_tree.hpp"
#include "tree_msgs/msg/node_parameter.hpp"
#include "tree_msgs/msg/node_status.hpp"
#include "tree_msgs/msg/status_change.hpp"
#include "tree_msgs/msg/status_change_log.hpp"

#include "tree_msgs/srv/get_tree_nodes.hpp"

#include "rclcpp/rclcpp.hpp"

// #include "behaviortree_cpp_v3_ros2_publisher/bt_ros_publisher.hpp"

// class BTRosPublisher : public BT::StatusChangeLogger
class BTRosPublisher : public BT::StatusChangeLogger
{

    public:
    
        tree_msgs::msg::BehaviorTree create_behavior_tree(const BT::Tree & tree);
        BTRosPublisher(const rclcpp::Node::WeakPtr & ros_node, const BT::Tree & tree);

        void callback(
            BT::Duration timestamp,
            const BT::TreeNode & node,
            BT::NodeStatus prev_status,
            BT::NodeStatus status) override; // overrides BT::StatusChangeLogger::callback

        void flush() override; // overrides BT::StatusChangeLogger::flush

    protected:
        rclcpp::Clock::SharedPtr clock_;
        rclcpp::Publisher<tree_msgs::msg::StatusChangeLog>::SharedPtr status_change_log_pub_;
        std::vector<tree_msgs::msg::StatusChange> state_changes;

        std::string tree_name;

        std::vector<BT::TreeNode::Ptr> tree_nodes_;
        BT::TreeNode * root_node_;
        tree_msgs::msg::BehaviorTree * behavior_tree;
};

#include "behaviortree_cpp_v3_ros2_publisher/bt_ros_publisher_impl.hpp"

#endif // BT_ROS_PUBLISHER_HPP