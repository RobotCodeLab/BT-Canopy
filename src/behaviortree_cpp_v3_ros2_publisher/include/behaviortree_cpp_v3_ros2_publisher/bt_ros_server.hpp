#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "tree_msgs/srv/get_tree_nodes.hpp"

// class BTRosServer : public rclcpp::Node
class BTRosServer 
{
public:
    // BTRosServer(const rclcpp::Node::WeakPtr & ros_node, const BT::Tree & tree)
    //  : rclcpp::Node::Node("BTRosServer")
        BTRosServer(const rclcpp::Node::WeakPtr & ros_node, const BT::Tree & tree)
    {

        auto node = ros_node.lock();
        clock_ = node->get_clock();
        root_node_ = tree.rootNode();
        tree_nodes_ = tree.nodes;

        // auto handle_add = [this](const std::shared_ptr<rmw_request_id_t> request_header,
        //                         const std::shared_ptr<tree_msgs::srv::GetTreeNodes::Request> request,
        //                         const std::shared_ptr<tree_msgs::srv::GetTreeNodes::Response> response) -> void {
        auto handle_add = [this](const std::shared_ptr<rmw_request_id_t> request_header,
                                const std::shared_ptr<tree_msgs::srv::GetTreeNodes::Request> request,
                                const std::shared_ptr<tree_msgs::srv::GetTreeNodes::Response> response) -> void {

            // Set request to void otherwise colcon throws an error
            (void) request_header;
            (void) request;

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Received request to send tree");

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

                response->behavior_tree.nodes.push_back(node_msg);

            }

            response->success = true;
        };

        // service_ = this->create_service<tree_msgs::srv::GetTreeNodes>("get_tree_nodes", handle_add);
        service_ = node->create_service<tree_msgs::srv::GetTreeNodes>("get_tree_nodes", handle_add);

    }


protected:
    rclcpp::Service<tree_msgs::srv::GetTreeNodes>::SharedPtr service_;
    rclcpp::Clock::SharedPtr clock_;
    std::vector<BT::TreeNode::Ptr> tree_nodes_;
    BT::TreeNode * root_node_;
};

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<BTRosServer>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }