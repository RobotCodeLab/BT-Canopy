#include "behaviortree_cpp_v3/loggers/abstract_logger.h"
#include "tf2_ros/buffer_interface.h"

#include "tree_msgs/msg/behavior_tree.hpp"
#include "tree_msgs/msg/node_parameter.hpp"
#include "tree_msgs/msg/node_status.hpp"
#include "tree_msgs/msg/status_change.hpp"
#include "tree_msgs/msg/status_change_log.hpp"

#include "tree_msgs/srv/get_tree_nodes.hpp"

class BTRosPublisher : public BT::StatusChangeLogger
{
    public:
        BTRosPublisher(const rclcpp::Node::WeakPtr & ros_node, const BT::Tree & tree)
        : StatusChangeLogger(tree.rootNode())
        {
            auto node = ros_node.lock();
            clock_ = node->get_clock();
            // logger_ = node->get_logger();

            //
            root_node_ = tree.rootNode();
            tree_nodes_ = tree.nodes;
            //

            tree_name = tree.rootNode()->name();
            status_change_log_pub_ = node->create_publisher<tree_msgs::msg::StatusChangeLog>(
                "/status_change_log", 10);


            //
            auto handle_add = [this](const std::shared_ptr<rmw_request_id_t> request_header,
                                const std::shared_ptr<tree_msgs::srv::GetTreeNodes::Request> request,
                                const std::shared_ptr<tree_msgs::srv::GetTreeNodes::Response> response) -> void {

            // Set request to void otherwise colcon throws an error
            (void) request_header;
            (void) request;

            response->behavior_tree.root_uid = root_node_->UID();
            response->behavior_tree.tree_name = root_node_->name();

            for (auto tree_node_ptr : tree_nodes_) {
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

                switch(tree_node->status()){
                    case BT::NodeStatus::IDLE:
                        node_msg.status.value = node_msg.status.IDLE;
                        break;
                    case BT::NodeStatus::RUNNING:
                        node_msg.status.value = node_msg.status.RUNNING;
                        break;
                    case BT::NodeStatus::SUCCESS:
                        node_msg.status.value = node_msg.status.SUCCESS;
                        break;
                    case BT::NodeStatus::FAILURE:
                        node_msg.status.value = node_msg.status.FAILURE;
                        break;
                    default:
                        break;
                }

                node_msg.instance_name = tree_node->name();
                node_msg.registration_name = tree_node->registrationName();


            }

            response->success = true;
        };

        service_ = node->create_service<tree_msgs::srv::GetTreeNodes>("get_tree_nodes", handle_add);

        //
        
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

        rclcpp::Service<tree_msgs::srv::GetTreeNodes>::SharedPtr service_;
        std::vector<BT::TreeNode::Ptr> tree_nodes_;
        BT::TreeNode * root_node_;
    
};