#include <iostream>
#include <memory>
#include <string>
using namespace std::literals;

#include "ping_pong_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Pong : public rclcpp::Node {
private:
  uint id_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

public:
  Pong(const uint id) : Node(pong_node_name(id)), id_(id) {
    subscriber_ = this->create_subscription<std_msgs::msg::String>(
        ping_topic_name(id_), rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const std_msgs::msg::String::SharedPtr message_pointer) {
          publisher_->publish(*message_pointer); // simply echo back
        });

    publisher_ = this->create_publisher<std_msgs::msg::String>(pong_topic_name(id_),
                                                               rclcpp::QoS(rclcpp::KeepLast(10)));
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  const ppm_options options = get_options(argc, argv);

  const auto node_counts = get_node_counts(options);
  auto nodes = std::vector<std::shared_ptr<Pong>>(node_counts);

  for (auto i = 0u; i < nodes.size(); ++i) {
    nodes.at(i) = std::make_shared<Pong>(i);
  }

  const auto thread_counts = nodes.size();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), thread_counts);
  for (const auto node : nodes) {
    executor.add_node(node);
  }

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
