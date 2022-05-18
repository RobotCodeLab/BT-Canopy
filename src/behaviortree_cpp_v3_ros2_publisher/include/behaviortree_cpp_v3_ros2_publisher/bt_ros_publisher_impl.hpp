#ifndef CANOPY_BT_ROS_PUBLISHER_IMPL_HPP_
#define CANOPY_BT_ROS_PUBLISHER_IMPL_HPP_

#include "behaviortree_cpp_v3/loggers/abstract_logger.h"
#include "tf2_ros/buffer_interface.h"

#include "tree_msgs/msg/behavior_tree.hpp"
#include "tree_msgs/msg/node_parameter.hpp"
#include "tree_msgs/msg/node_status.hpp"
#include "tree_msgs/msg/status_change.hpp"
#include "tree_msgs/msg/status_change_log.hpp"

#include "tree_msgs/srv/get_tree_nodes.hpp"

#include "rclcpp/rclcpp.hpp"

#include "behaviortree_cpp_v3_ros2_publisher/bt_ros_publisher.hpp"

// class BTRosPublisher : public BT::StatusChangeLogger
// {
    // tree_msgs::msg::BehaviorTree create_behavior_tree(const BT::Tree & tree);

    // public:
        BTRosPublisher::BTRosPublisher(const rclcpp::Node::WeakPtr & ros_node, const BT::Tree & tree)
        : BT::StatusChangeLogger(tree.rootNode())
        {
            auto node = ros_node.lock();
            clock_ = node->get_clock();

            root_node_ = tree.rootNode();
            tree_nodes_ = tree.nodes;      

            tree_name = tree.rootNode()->name();
            status_change_log_pub_ = node->create_publisher<tree_msgs::msg::StatusChangeLog>(
                "/status_change_log", 10);   
        
            behavior_tree = &create_behavior_tree(tree);

        }  

        tree_msgs::msg::BehaviorTree BTRosPublisher::create_behavior_tree(const BT::Tree & tree){

            tree_msgs::msg::BehaviorTree behavior_tree;
            behavior_tree.root_uid = tree.rootNode()->UID();
            behavior_tree.tree_name = tree.rootNode()->name();

            for (auto tree_node_ptr : tree.nodes) {
                BT::TreeNode * tree_node = tree_node_ptr.get();
                tree_msgs::msg::TreeNode node_msg;
                
                node_msg.uid = tree_node->UID();
                
                if (auto control = dynamic_cast<const BT::ControlNode*>(tree_node))
                {
                    for (const auto& child : control->children())
                    {
                        node_msg.children_uid.push_back(child->UID());
                    }
                } else if (auto decorator = dynamic_cast<const BT::DecoratorNode*>(tree_node))
                {
                    node_msg.children_uid.push_back(decorator->child()->UID());
                }

                switch(tree_node->type()){
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
                        break;
                }

                node_msg.instance_name = tree_node->name();
                node_msg.registration_name = tree_node->registrationName();

                behavior_tree.nodes.push_back(node_msg);
            }
            return behavior_tree;
        }

        // void callback(
        void BTRosPublisher::callback(
            BT::Duration timestamp,
            const BT::TreeNode & node,
            BT::NodeStatus prev_status,
            BT::NodeStatus status) 
        {
            tree_msgs::msg::StatusChange event;

            event.timestamp = tf2_ros::toMsg(tf2::TimePoint(timestamp));
            event.uid = node.UID();

            event.prev_status.value = static_cast<int>(prev_status);
            event.status.value = static_cast<int>(status);

            state_changes.push_back(std::move(event));

        }
        // void flush()
        void BTRosPublisher::flush()
        {
            if (!state_changes.empty()){
                auto msg = std::make_unique<tree_msgs::msg::StatusChangeLog>();
                msg->state_changes = state_changes;
                msg->behavior_tree = *behavior_tree;
                status_change_log_pub_->publish(std::move(msg));
                state_changes.clear();
            }
        }
        

    // protected:
    //     rclcpp::Clock::SharedPtr clock_;
    //     rclcpp::Publisher<tree_msgs::msg::StatusChangeLog>::SharedPtr status_change_log_pub_;
    //     std::vector<tree_msgs::msg::StatusChange> state_changes;

    //     std::string tree_name;

    //     std::vector<BT::TreeNode::Ptr> tree_nodes_;
    //     BT::TreeNode * root_node_;
    //     tree_msgs::msg::BehaviorTree * behavior_tree;
    
// };

#endif // BT_ROS_PUBLISHER_HPP