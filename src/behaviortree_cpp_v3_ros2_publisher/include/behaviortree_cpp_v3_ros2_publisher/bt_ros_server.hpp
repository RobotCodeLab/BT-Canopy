#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "tree_msgs/srv/get_tree_nodes.hpp"

class BTRosServer : public rclcpp::Node
{
public:
    BTRosServer(const rclcpp::Node::WeakPtr & ros_node, const BT::Tree & tree)
     : rclcpp::Node::Node("BTRosServer")
    {

        auto node = ros_node.lock();
        clock_ = node->get_clock();
        root_node_ = tree.rootNode();
        tree_nodes_ = tree.nodes;

        auto handle_add = [this](const std::shared_ptr<rmw_request_id_t> request_header,
                                const std::shared_ptr<tree_msgs::srv::GetTreeNodes::Request> request,
                                const std::shared_ptr<tree_msgs::srv::GetTreeNodes::Response> response) -> void {

            response->behavior_tree.root_uid = root_node_->UID();
            response->behavior_tree.tree_name = root_node_->name();

            for (auto & node : tree_nodes_) {
                tree_msgs::msg::TreeNode tree_node;
                tree_node.uid = node->UID();
                
                BT::NodeType type = node->type();

                switch(type){
                    case BT::NodeType::UNDEFINED:
                        tree_node.type = 0;
                        break;
                    case BT::NodeType::ACTION:
                        tree_node.type = 1;
                        break;
                    case BT::NodeType::CONDITION:
                        tree_node.type = 2;
                        break;
                    case BT::NodeType::CONTROL:

                        for (auto & child : node->children_nodes_) {
                            tree_node.children_uid.push_back(child->UID());
                        }

                        tree_node.type = 3;
                    case BT::NodeType::DECORATOR:
                        tree_node.children_uid.push_back(node->child_node_->UID());
                        tree_node.type = 4;
                    case BT::NodeType::SUBTREE:
                        tree_node.children_uid.push_back(node->child_node_->UID());
                        tree_node.type = 5;
                    default:
                        break;
                }

                // tree_node.children_uid = node->childrenUIDs();
            }

            response->success = true;
        };

        service_ = this->create_service<tree_msgs::srv::GetTreeNodes>("get_tree_nodes", handle_add);

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