#include <iostream>
#include <memory>
#include <string>
using namespace std::literals;

#include "ping_pong_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Pong : public rclcpp::Node {
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

public:
  Pong(const std::string node_name, const std::string pong_topic_name,
       const std::string ping_topic_name)
      : Node(node_name) {
    subscriber_ = this->create_subscription<std_msgs::msg::String>(
        ping_topic_name, rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const std_msgs::msg::String::SharedPtr message_pointer) {
          publisher_->publish(*message_pointer); // simply echo back
        });

    publisher_ = this->create_publisher<std_msgs::msg::String>(pong_topic_name,
                                                               rclcpp::QoS(rclcpp::KeepLast(10)));
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node_counts = 10;
  auto nodes = std::vector<std::shared_ptr<Pong>>{};

  for (auto i = 0; i < node_counts; ++i) {
    nodes.push_back(
        std::make_shared<Pong>(pong_node_name(i), pong_topic_name(i), ping_topic_name(i)));
  }

  auto thread_counts = nodes.size();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), thread_counts);
  std::for_each(std::begin(nodes), std::end(nodes),
                [&executor](auto node) { executor.add_node(node); });

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
