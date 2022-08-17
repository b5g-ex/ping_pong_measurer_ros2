#include <iostream>
#include <memory>
#include <string>
using namespace std::literals;

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Pong : public rclcpp::Node {
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

public:
  Pong() : Node("pong") {
    subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "ping", rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const std_msgs::msg::String::SharedPtr message_pointer) {
          publisher_->publish(*message_pointer); // simply echo back
        });

    publisher_ = this->create_publisher<std_msgs::msg::String>(
        "pong", rclcpp::QoS(rclcpp::KeepLast(10)));
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Pong>());
  rclcpp::shutdown();
  return 0;
}
