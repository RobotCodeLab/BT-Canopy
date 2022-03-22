#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "tree_msgs/msg/behavior_tree_log.hpp"

#include <zmq.hpp>

class TreeListener : public rclcpp::Node
{
  public:
    TreeListener() : Node("zmq_to_ros_republisher")
    {
      // call callback function when a message is received
      subscription_ = this->create_subscription<tree_msgs::msg::BehaviorTreeLog>(
        "behavior_tree_log", 10, std::bind(&TreeListener::callback, this, std::placeholders::_1));
    }

  private:
    void callback(const tree_msgs::msg::BehaviorTreeLog::SharedPtr msg)
    {
      // std::cout << "Received message: " << msg-> << std::endl;

      auto events = msg.get()->event_log;

      // RCLCPP_INFO(this->get_logger(), "Received message: %s", msg->event_log);

      for (auto event : events)
      {
        std::cout << event.node_name << " " << event.current_status << std::endl;
      }

    }

    rclcpp::Subscription<tree_msgs::msg::BehaviorTreeLog>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TreeListener>());
  rclcpp::shutdown();
  return 0;

}
