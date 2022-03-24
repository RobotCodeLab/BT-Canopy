#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "tree_msgs/msg/behavior_tree_log.hpp"

#include <chrono>

#include <zmq.hpp>

class zmq_to_ros_republisher : public rclcpp::Node
{
  public:
    
    bool connected = false;

    zmq::context_t zmq_context;
    // zmq::socket_t zmq_socket;
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
            std::string msg(static_cast<char*>(message.data()), message.size());
            std::cout << "Received: " << msg << std::endl;
            
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
