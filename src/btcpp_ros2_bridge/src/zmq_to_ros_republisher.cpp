#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "tree_msgs/msg/behavior_tree_log.hpp"
#include "tree_msgs/srv/get_tree_nodes.hpp"
#include "btcpp_ros2_bridge/deserializer.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include <chrono>

#include <zmq.hpp>

#include <memory>


class zmq_to_ros_republisher : public rclcpp::Node
{
  public:
    
    bool connected_to_pub = false;
    bool got_tree = false;

    std::unordered_map<uint16_t, tree_node> uid_tree; // store the tree nodes in a map ordered by their uid

    zmq::context_t zmq_context;
    zmq::socket_t  zmq_subscriber;

    zmq_to_ros_republisher() : Node("zmq_to_ros_republisher")
    {

      this->declare_parameter<std::string>("pub_port", "1666");
      this->declare_parameter<std::string>("server_port", "1667");
      this->declare_parameter<std::string>("server_ip", "localhost");

      zmq_context = zmq::context_t(1);
      zmq_subscriber = zmq::socket_t(zmq_context, ZMQ_SUB);

      auto timer = create_wall_timer(std::chrono::milliseconds(100), std::bind(&zmq_to_ros_republisher::timer_callback, this));

      publisher_ = this->create_publisher<tree_msgs::msg::BehaviorTreeLog>("/behavior_tree_log", 10);
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&zmq_to_ros_republisher::timer_callback, this));

    }

    bool connect()
    {

      pubPort = this->get_parameter("pub_port").as_string();
      reqPort = this->get_parameter("server_port").as_string();
      serverIP = this->get_parameter("server_ip").as_string();

      connection_address_pub = "tcp://" + serverIP + ":" + pubPort;
      connection_address_req = "tcp://" + serverIP + ":" + reqPort;

      if(!got_tree && !getTree())
      {
        RCLCPP_WARN(this->get_logger(), "Could not get tree from server %s:%s", serverIP.c_str(), reqPort.c_str());
        return false;
      }
      else{
        RCLCPP_INFO(this->get_logger(), "Got tree from server");
        got_tree = true;

      }

      if(connected_to_pub)
      {
        return true;
      }

      try{
        
        int timeout_ms = 1;
        zmq_subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);
        zmq_subscriber.setsockopt(ZMQ_LINGER, 0);
        zmq_subscriber.setsockopt(ZMQ_RCVTIMEO,&timeout_ms, sizeof(int) );
        zmq_subscriber.connect(connection_address_pub);

      }
      catch(zmq::error_t e)
      {
        // connected_to_pub = false;
        std::cout << "Error connecting to publisher: " << e.what() << std::endl;
        return false;
      }
      return true;
    }

  private:

    void timer_callback()
    {

      if(connected_to_pub)
      {
        zmq::message_t message;

        try
        {
          while(zmq_subscriber.recv(&message))
          {
            
            const char* buffer = reinterpret_cast<const char*>(message.data());

            const uint32_t header_size = flatbuffers::ReadScalar<uint32_t>(buffer);
            const uint32_t num_transitions = flatbuffers::ReadScalar<uint32_t>(&buffer[4+header_size]);

            if(!event_log.empty())
            {
              event_log.clear();
            }
      

            for(size_t t=0; t < num_transitions; t++)
            {
                size_t offset = 8 + header_size + 12*t;

                const double t_sec  = flatbuffers::ReadScalar<uint32_t>( &buffer[offset] );
                const double t_usec = flatbuffers::ReadScalar<uint32_t>( &buffer[offset+4] );

                tree_msgs::msg::BehaviorTreeStatusChange event;

                const uint16_t uid = flatbuffers::ReadScalar<uint16_t>(&buffer[offset+8]);

                BT::NodeStatus prev_status = convert(flatbuffers::ReadScalar<Serialization::NodeStatus>(&buffer[offset+10] ));
                BT::NodeStatus status  = convert(flatbuffers::ReadScalar<Serialization::NodeStatus>(&buffer[offset+11] ));

                event.node_name = uid_tree.at(uid).instance_name;
                event.previous_status = toStr(prev_status);
                event.current_status = toStr(status);
                event.timestamp.sec = t_sec;
                event.timestamp.nanosec = t_usec*1000;

                event_log.push_back(event);

            }

            log.event_log = event_log;
            log.timestamp = this->get_clock()->now();

            publisher_->publish(log);

          }
          
        }
        catch( zmq::error_t& err)
        {
            std::cout << "Error: " << err.what() << std::endl;
        }

      }else{
        connected_to_pub = connect();
      }

    }

    bool getTree()
    {
      try{
        zmq::message_t request(0);
        zmq::message_t reply;

        zmq::socket_t zmq_client(zmq_context, ZMQ_REQ);

        zmq_client.setsockopt(ZMQ_LINGER, 0);
        int timeout_ms = 1000;
        zmq_client.setsockopt(ZMQ_RCVTIMEO,&timeout_ms, sizeof(int) );

        zmq_client.connect(connection_address_req); // address for signalling server to send tree info

        zmq_client.send(request);

        bool got_reply = zmq_client.recv(&reply);
        if(! got_reply)
        {
          return false;
        }

        const char* buffer = reinterpret_cast<const char*>(reply.data());
        auto fb_behavior_tree = Serialization::GetBehaviorTree(buffer);

        uid_tree = BuildTreeFromFlatbuffers(fb_behavior_tree);

      }
      catch(zmq::error_t e)
      {
        return false;
      }

      return true;

    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<tree_msgs::msg::BehaviorTreeLog>::SharedPtr publisher_;

  protected:
    rclcpp::Clock::SharedPtr clock;
    tree_msgs::msg::BehaviorTreeLog log;
  
    std::vector<tree_msgs::msg::BehaviorTreeStatusChange> event_log;

    std::string pubPort;
    std::string reqPort;
    std::string connection_address_pub;
    std::string connection_address_req;
    std::string serverIP;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<zmq_to_ros_republisher>());
  rclcpp::shutdown();
  return 0;

}
