#ifndef CANOPY_BT_ROS_PUBLISHER_HPP_
#define CANOPY_BT_ROS_PUBLISHER_HPP_

#include "behaviortree_cpp_v3/loggers/abstract_logger.h"
#include "tf2_ros/buffer_interface.h"

#include "tree_msgs/msg/behavior_tree.hpp"
#include "tree_msgs/msg/node_parameter.hpp"
#include "tree_msgs/msg/node_status.hpp"
#include "tree_msgs/msg/status_change.hpp"
#include "tree_msgs/msg/status_change_log.hpp"

#include "rclcpp/rclcpp.hpp"

/**
 * Class to log changes to the behavior tree and publish them over
 * the ROS topic /bt_status_change_log.
 */
class BTRosPublisher : public BT::StatusChangeLogger
{
private:
    /**
     * Actual constructor that stores the structure of the tree in a vector.
     *
     * @param ros_node The ROS2 node for publishing.
     * @param tree The behavior tree to track.
     * @param tree_uid The UID of the tree being tracked.
     */
    void init(const rclcpp::Node::WeakPtr &ros_node, const BT::Tree &tree, const std::string &tree_uid)
    {
        auto node = ros_node.lock();
        clock_ = node->get_clock();

        tree_nodes_ = tree.nodes;

        tree_name = tree.rootNode()->name();
        status_change_log_pub_ = node->create_publisher<tree_msgs::msg::StatusChangeLog>(
            "/bt_status_change_log", 10);

        behavior_tree.root_uid = tree.rootNode()->UID();
        behavior_tree.tree_uid = tree_uid;

        // Parse noes of the tree.
        for (auto tree_node_ptr : tree.nodes)
        {
            BT::TreeNode *tree_node = tree_node_ptr.get();
            tree_msgs::msg::TreeNode node_msg;

            node_msg.uid = tree_node->UID();

            switch (tree_node->type())
            {
            case BT::NodeType::UNDEFINED:
                node_msg.type = node_msg.UNDEFINED;
                break;
            case BT::NodeType::ACTION:
                node_msg.type = node_msg.ACTION;
                break;
            case BT::NodeType::CONDITION:
                node_msg.type = node_msg.CONDITION;
                break;
            case BT::NodeType::CONTROL:
                node_msg.type = node_msg.CONTROL;
                break;
            case BT::NodeType::DECORATOR:
                node_msg.type = node_msg.DECORATOR;
                break;
            case BT::NodeType::SUBTREE:
                node_msg.type = node_msg.SUBTREE;
                break;
            default:
                RCLCPP_ERROR(node->get_logger(), "Unknown node type read from tree");
                node_msg.type = node_msg.UNDEFINED;
                break;
            }

            if (auto control = dynamic_cast<const BT::ControlNode *>(tree_node))
            {
                for (const auto &child : control->children())
                {
                    node_msg.child_uids.push_back(child->UID());
                }
            }
            else if (auto decorator = dynamic_cast<const BT::DecoratorNode *>(tree_node))
            {
                node_msg.child_uids.push_back(decorator->child()->UID());
            }

            node_msg.instance_name = tree_node->name();
            node_msg.registration_name = tree_node->registrationName();

            behavior_tree.nodes.push_back(node_msg);
        }
    }

public:
    /**
     * Constructor for logger when tree_uid is provided.
     *
     * @param ros_node The ROS2 node for publishing.
     * @param tree The behavior tree to track.
     * @param tree_uid The UID of the tree being tracked.
     */
    BTRosPublisher(const rclcpp::Node::WeakPtr &ros_node, const BT::Tree &tree, const std::string &tree_uid)
        : StatusChangeLogger(tree.rootNode())
    {
        init(ros_node, tree, tree_uid);
    }

    /**
     * Constructor for logger when tree_uid is not provided. A hash is calculated in its place.
     *
     * @param ros_node The ROS2 node for publishing.
     * @param tree The behavior tree to track.
     */
    BTRosPublisher(const rclcpp::Node::WeakPtr &ros_node, const BT::Tree &tree)
        : StatusChangeLogger(tree.rootNode())
    {

        BT::TreeNode *root_node_ = tree.rootNode();

        std::string tree_shape_uid = "";

        BT::applyRecursiveVisitor(
            root_node_,
            [&tree_shape_uid](BT::TreeNode *node)
            {
                tree_shape_uid += node->UID();
                tree_shape_uid += node->name();
                tree_shape_uid += "-";
            });

        if (tree_shape_uid.empty())
        {
            throw std::runtime_error("Tree is empty");
        }

        std::hash<std::string> hash_fn;
        tree_shape_uid = std::to_string(hash_fn(tree_shape_uid));

        // Only get the first 31 characters of the hash (It will probably be shorter).
        tree_shape_uid = tree_shape_uid.substr(0, 31);

        // Prepend '#' to the front of the tree_shape_uid to indicate that it's a hash.
        tree_shape_uid.insert(tree_shape_uid.begin(), '#');

        init(ros_node, tree, tree_shape_uid);
    }

    /**
     * Called routinely to store the node change in a vector.
     *
     * @param timestamp How much time has passed.
     * @param node The node of the tree that changed.
     * @param prev_status The previous status of the tree.
     * @param status The current status of the tree.
     */
    void callback(
        BT::Duration timestamp,
        const BT::TreeNode &node,
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

    /**
     * Publish list of node status changes to Canopy.
     */
    void flush() override
    {
        if (!state_changes.empty())
        {
            auto msg = std::make_unique<tree_msgs::msg::StatusChangeLog>();
            msg->state_changes = state_changes;
            msg->behavior_tree = behavior_tree;
            status_change_log_pub_->publish(std::move(msg));
            state_changes.clear();
        }
    }

protected:
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Publisher<tree_msgs::msg::StatusChangeLog>::SharedPtr status_change_log_pub_;

    std::string tree_name;
    std::vector<BT::TreeNode::Ptr> tree_nodes_;

    std::vector<tree_msgs::msg::StatusChange> state_changes;
    tree_msgs::msg::BehaviorTree behavior_tree;
};

#endif // BT_ROS_PUBLISHER_HPP
