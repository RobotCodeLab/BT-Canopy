#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "tree_msgs/msg/behavior_tree_log.hpp"

#include <chrono>

#include <zmq.hpp>

class zmq_to_ros_republisher : public rclcpp::Node
{
  public:


    
    zmq_to_ros_republisher() : Node("zmq_to_ros_republisher")
    {
      publisher_ = this->create_publisher<tree_msgs::msg::BehaviorTreeLog>("/behavior_tree_log", 10);
      timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&zmq_to_ros_republisher::timer_callback, this));

    }


  private:

    void timer_callback()
    {
      zmq::context_t context(1);
      // zmq::socket_t socket(context, ZMQ_SUB);
      // socket.connect("tcp://localhost:5555");
      // socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);


      //print current time
      auto now = std::chrono::system_clock::now();
      auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
      auto value = now_ms.time_since_epoch();
      auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(value);
      std::cout << "Current time: " << millis.count() << std::endl;

      // tree_msgs::msg::BehaviorTreeLog msg;
      // msg.timestamp = std::chrono::system_clock::now();
      // msg.event_log.push_back(tree_msgs::msg::BehaviorTreeStatusChange());
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
