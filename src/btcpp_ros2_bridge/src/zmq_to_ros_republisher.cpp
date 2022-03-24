#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "tree_msgs/msg/behavior_tree_log.hpp"
#include "deserializer.h"

#include <chrono>

#include <zmq.hpp>

// using namespace BT;

class zmq_to_ros_republisher : public rclcpp::Node
{
  public:
    
    bool connected = false;

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

    // bool getTreeFromServer()
    // {
    //   try{
    //     zmq::message_t request(0);
    //     zmq::message_t reply;

    //     zmq::socket_t zmq_client(zmq_context, ZMQ_REQ);
    //     zmq_client.connect(connection_address_req);

    //     int timeout_ms = 1000;
    //     zmq_client.setsockopt(ZMQ_RCVTIMEO,&timeout_ms, sizeof(int) );

    //     zmq_client.send(request);

    //     bool received = zmq_client.recv(&reply);
    //     if (! received)
    //     {
    //       std::cout << "Error receiving from server" << std::endl;
    //       return false;
    //     }

    //     for (const auto& tree_node: _loaded_tree.nodes())
    //     {
    //       std::cout << tree_node.name() << std::endl;
    //     }

    //   }
    // }


  private:

    void timer_callback()
    {

      //print current time
      auto now = std::chrono::system_clock::now();
      auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
      auto value = now_ms.time_since_epoch();
      auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(value);
      std::cout << "Current time: " << millis.count() << std::endl;

      if(connected)
      {
        zmq::message_t message;

        try{
          while(zmq_subscriber.recv(&message))
          {
            
            const char* buffer = reinterpret_cast<const char*>(message.data());

            const uint32_t header_size = flatbuffers::ReadScalar<uint32_t>(buffer);
            const uint32_t num_transitions = flatbuffers::ReadScalar<uint32_t>(&buffer[4+header_size]);

            std::vector<std::pair<int, BT::NodeStatus>> node_status;

            
            for(size_t t=0; t < num_transitions; t++)
            {
                size_t offset = 8 + header_size + 12*t;

                // const double t_sec  = flatbuffers::ReadScalar<uint32_t>( &buffer[offset] );
                // const double t_usec = flatbuffers::ReadScalar<uint32_t>( &buffer[offset+4] );
                // double timestamp = t_sec + t_usec* 0.000001;
                const uint16_t uid = flatbuffers::ReadScalar<uint16_t>(&buffer[offset+8]);
                // const uint16_t index = _uid_to_index.at(uid);
                const uint16_t index = 10;
                BT::NodeStatus prev_status = convert(flatbuffers::ReadScalar<Serialization::NodeStatus>(&buffer[index+10] ));
                BT::NodeStatus status  = convert(flatbuffers::ReadScalar<Serialization::NodeStatus>(&buffer[offset+11] ));

                // _loaded_tree.node(index)->status = status;
                node_status.push_back( {index, status} );

            }

          // print node status
          for(const auto& status: node_status)
          {
            std::cout << "Node " << status.first << ": " << status.second << std::endl;
          }
                    
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
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<tree_msgs::msg::BehaviorTreeLog>::SharedPtr publisher_;

    
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<zmq_to_ros_republisher>());
  rclcpp::shutdown();
  return 0;

}
