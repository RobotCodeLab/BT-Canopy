#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "tree_msgs/msg/behavior_tree_log.hpp"

#include <chrono>

#include <zmq.hpp>

class zmq_to_ros_republisher : public rclcpp::Node
{
  public:
    
    zmq::context_t zmq_context;
    zmq::socket_t zmq_socket;

    std::string pubPort = "1666";
    std::string reqPort = "1667";

    std::string serverIP = "localhost";
    std::string connection_address_pub = "tcp://" + serverIP + ":" + pubPort;
    std::string connection_address_req = "tcp://" + serverIP + ":" + reqPort;

    zmq_to_ros_republisher() : Node("zmq_to_ros_republisher")
    {

      zmq_context = zmq::context_t(1);
      zmq_socket = zmq::socket_t(zmq_context, ZMQ_SUB);

      zmq_socket.connect(connection_address_req);
      zmq_socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);

      auto timer = create_wall_timer(std::chrono::milliseconds(100), std::bind(&zmq_to_ros_republisher::timer_callback, this));

      publisher_ = this->create_publisher<tree_msgs::msg::BehaviorTreeLog>("/behavior_tree_log", 10);
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&zmq_to_ros_republisher::timer_callback, this));

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

      // receive a zmq message
      zmq::message_t message;
      zmq_socket.recv(&message);
      
      // print the message
      std::string message_str(static_cast<char*>(message.data()), message.size());
      std::cout << "Received message: " << message_str << std::endl;


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
