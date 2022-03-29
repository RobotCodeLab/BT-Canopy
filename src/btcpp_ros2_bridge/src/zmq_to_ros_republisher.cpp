#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "tree_msgs/msg/behavior_tree_log.hpp"
#include "btcpp_ros2_bridge/deserializer.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include <chrono>

#include <zmq.hpp>

// using namespace BT;

class zmq_to_ros_republisher : public rclcpp::Node
{
  public:
    
    bool connected = false;
    bool got_tree = false;

    std::unordered_map<uint16_t, tree_node> uid_tree; // store the tree nodes in a map ordered by their uid

    zmq::context_t zmq_context;
    zmq::socket_t  zmq_subscriber;

    std::string pubPort = "1666";
    std::string reqPort = "1667";

    std::string serverIP = "localhost";
    std::string connection_address_pub = "tcp://" + serverIP + ":" + pubPort;
    std::string connection_address_req = "tcp://" + serverIP + ":" + reqPort;

    zmq_to_ros_republisher() : Node("zmq_to_ros_republisher")
    {

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

      if(!got_tree)
      {
        got_tree = getTree();
      }

      if(connected)
      {
        return true;
      }


      try{
        
        int timeout_ms = 1;
        zmq_subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);
        zmq_subscriber.setsockopt(ZMQ_RCVTIMEO,&timeout_ms, sizeof(int) );
        
        zmq_subscriber.connect(connection_address_pub);


      }
      catch(zmq::error_t e)
      {
        connected = false;
        std::cout << "Error connecting to publisher: " << e.what() << std::endl;
        return false;
      }

      connected = true;

    }



  private:

    void timer_callback()
    {

      if(connected)
      {
        zmq::message_t message;

        try
        {
          while(zmq_subscriber.recv(&message))
          {
            
            const char* buffer = reinterpret_cast<const char*>(message.data());

            const uint32_t header_size = flatbuffers::ReadScalar<uint32_t>(buffer);
            const uint32_t num_transitions = flatbuffers::ReadScalar<uint32_t>(&buffer[4+header_size]);

            std::cout << "Header size: " << header_size << std::endl;
            std::cout << "Received " << num_transitions << " transitions" << std::endl;

            // std::vector<std::pair<uint16_t, BT::NodeStatus>> node_status;
            // std::vector<std::pair<uint16_t, BT::NodeStatus>> previous_node_status;

            
            if(!event_log.empty())
            {
              event_log.clear();
            }
      

            for(size_t t=0; t < num_transitions; t++)
            {
                size_t offset = 8 + header_size + 12*t;

                const double t_sec  = flatbuffers::ReadScalar<uint32_t>( &buffer[offset] );
                const double t_usec = flatbuffers::ReadScalar<uint32_t>( &buffer[offset+4] );
                // double timestamp = t_sec + t_usec* 0.000001;

                tree_msgs::msg::BehaviorTreeStatusChange event;

                const uint16_t uid = flatbuffers::ReadScalar<uint16_t>(&buffer[offset+8]);

                // std::cout << "Transition " << t << ": " << uid << std::endl;
                
                BT::NodeStatus prev_status = convert(flatbuffers::ReadScalar<Serialization::NodeStatus>(&buffer[offset+10] ));
                BT::NodeStatus status  = convert(flatbuffers::ReadScalar<Serialization::NodeStatus>(&buffer[offset+11] ));


                event.node_name = uid_tree.at(uid).instance_name;
                event.previous_status = toStr(prev_status);
                event.current_status = toStr(status);
                event.timestamp.sec = t_sec;
                event.timestamp.nanosec = t_usec*1000;

                event_log.push_back(event);

                // previous_node_status.push_back( {uid, prev_status});
                // node_status.push_back( {uid, status} );

            }

            log.event_log = event_log;
            log.timestamp = this->get_clock()->now();

            publisher_->publish(log);

            // std::cout << "\n" << std::endl;
            // std::cout << "Current status: " << std::endl;

            // // print node status
            // for(const auto& status: node_status)
            // {

            //   // TODO: Publish to ros here
            //   // Need to double check that reported status matches up with status in Groot

            //   tree_node node = uid_tree.at(status.first);
            //   std::cout << "Node " << node.instance_name << ": " << status.second << std::endl;

            // }
            // // print previous node status

            // std::cout << "Previous node status: " << std::endl;

            // for(const auto& status: previous_node_status)
            // {

            //   tree_node node = uid_tree.at(status.first);
            //   std::cout << "Node " << node.instance_name << ": " << status.second << std::endl;     
            // }

          }
          
        }
        catch( zmq::error_t& err)
        {
            std::cout << "Error: " << err.what() << std::endl;
        }

      }else{
        connect();
      }

      // receive a zmq message
    }

    bool getTree()
    {
      try{
        zmq::message_t request(0);
        zmq::message_t reply;

        zmq::socket_t zmq_client(zmq_context, ZMQ_REQ);
        zmq_client.connect(connection_address_req); // address for signalling server to send tree info

        int timeout_ms = 1000;
        zmq_client.setsockopt(ZMQ_RCVTIMEO,&timeout_ms, sizeof(int) );
        zmq_client.send(request);

        bool got_reply = zmq_client.recv(&reply);
        if(! got_reply)
        {
          std::cout << "No reply from server while getting tree" << std::endl;
          return false;
        }

        const char* buffer = reinterpret_cast<const char*>(reply.data());
        auto fb_behavior_tree = Serialization::GetBehaviorTree(buffer);

        uid_tree = BuildTreeFromFlatbuffers(fb_behavior_tree);

      }
      catch(zmq::error_t e)
      {
        std::cout << "Error getting tree: " << e.what() << std::endl;
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

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<zmq_to_ros_republisher>());
  rclcpp::shutdown();
  return 0;

}
