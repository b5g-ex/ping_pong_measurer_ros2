#include <iomanip>
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
  std::string index_;
  std::string ping_pub_type_;
  std::string ping_sub_type_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

  std::string node_name_(uint id) {
    std::ostringstream index;
    index << std::setfill('0') << std::setw(3) << std::to_string(id);

    return "pong"s + index.str();
  }

  std::string ping_topic_name_() {
    std::ostringstream index;
    index << std::setfill('0') << std::setw(3) << std::to_string(id_);

    if (ping_pub_type_ == "single"s)
      return "/ping000"s;

    if (ping_pub_type_ == "multiple"s)
      return "/ping"s + index.str();

    throw std::runtime_error("ping_pub_type is invalid!!!");
  }

  std::string pong_topic_name_() {
    std::ostringstream index;
    index << std::setfill('0') << std::setw(3) << std::to_string(id_);

    if (ping_sub_type_ == "single"s)
      return "/pong000"s;

    if (ping_sub_type_ == "multiple"s)
      return "/pong"s + index.str();

    throw std::runtime_error("ping_sub_type is invalid!!!");
  }

public:
  Pong(const uint id, std::string ping_pub_type, std::string ping_sub_type)
      : Node(node_name_(id)), id_(id), ping_pub_type_(ping_pub_type),
        ping_sub_type_(ping_sub_type) {

    std::ostringstream ss;
    ss << std::setfill('0') << std::setw(3) << std::to_string(id);
    index_ = ss.str();

    subscriber_ = this->create_subscription<std_msgs::msg::String>(
        ping_topic_name_(), rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const std_msgs::msg::String::SharedPtr message_pointer) {
          std_msgs::msg::String message;
          message.data = index_ + message_pointer->data.substr(3);
          publisher_->publish(message);
        });

    publisher_ = this->create_publisher<std_msgs::msg::String>(pong_topic_name_(),
                                                               rclcpp::QoS(rclcpp::KeepLast(10)));
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  const ppm_options options = get_options(argc, argv);

  const auto node_counts = get_node_counts(options);
  auto nodes = std::vector<std::shared_ptr<Pong>>(node_counts);

  const auto ping_pub_type = get_ping_pub_type(options);
  const auto ping_sub_type = get_ping_sub_type(options);

  for (auto i = 0u; i < nodes.size(); ++i) {
    nodes.at(i) = std::make_shared<Pong>(i, ping_pub_type, ping_sub_type);
  }

  const auto thread_counts = nodes.size();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), thread_counts);
  for (const auto &node : nodes) {
    executor.add_node(node);
  }

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
