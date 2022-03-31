#include "rclcpp/rclcpp.hpp"
#include "tree_msgs/srv/get_tree_nodes.hpp"
#include "btcpp_ros2_bridge/deserializer.hpp"
#include <zmq.hpp>

class get_tree_nodes_service : public rclcpp::Node
{
public:
    get_tree_nodes_service() : Node("get_tree_nodes_service")
    {

    this->declare_parameter<std::string>("server_port", "1667");
    this->declare_parameter<std::string>("server_ip", "localhost");

    auto handle_add = [this](const std::shared_ptr<rmw_request_id_t> request_header,
                            const std::shared_ptr<tree_msgs::srv::GetTreeNodes::Request> request,
                            const std::shared_ptr<tree_msgs::srv::GetTreeNodes::Response> response) -> void {

        (void) request_header;
        (void) request;

        zmq::context_t zmq_context = zmq::context_t(1);
        zmq::socket_t zmq_subscriber = zmq::socket_t(zmq_context, ZMQ_SUB);

        std::string reqPort = this->get_parameter("server_port").as_string();
        std::string serverIP = this->get_parameter("server_ip").as_string();

        std::string connection_address_req = "tcp://" + serverIP + ":" + reqPort;

        zmq::message_t zmq_request(0);
        zmq::message_t reply;

        zmq::socket_t zmq_client(zmq_context, ZMQ_REQ);

        int timeout_ms = 1000;
        zmq_client.setsockopt(ZMQ_RCVTIMEO,&timeout_ms, sizeof(int));
        zmq_client.setsockopt(ZMQ_LINGER, 0);

        zmq_client.connect(connection_address_req); // address for signalling server to send tree info

        
        zmq_client.send(zmq_request);

        bool got_reply = zmq_client.recv(&reply);
        if(! got_reply)
        {
            response->success = false;
            return;
        }

        const char* buffer = reinterpret_cast<const char*>(reply.data());
        auto fb_behavior_tree = Serialization::GetBehaviorTree(buffer);

        std::unordered_map<uint16_t, tree_node> uid_tree = BuildTreeFromFlatbuffers(fb_behavior_tree);

        for(auto &node : uid_tree)
        {
            tree_msgs::msg::TreeNode tree_node_msg;

            tree_node_msg.uid = node.second.uid;
            tree_node_msg.type = NodeTypeToStr(node.second.type);
            tree_node_msg.instance_name = node.second.instance_name;
            tree_node_msg.registration_id = node.second.registration_ID;
            tree_node_msg.child_uids = node.second.children_uid;

            response->nodes.push_back(tree_node_msg);

        }

        response->success = true;
    };

    service_ = this->create_service<tree_msgs::srv::GetTreeNodes>("get_tree_nodes", handle_add);

    }
private:
    rclcpp::Service<tree_msgs::srv::GetTreeNodes>::SharedPtr service_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<get_tree_nodes_service>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}