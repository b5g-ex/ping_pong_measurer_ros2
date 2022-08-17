#include <iostream>
#include <memory>
#include <string>
using namespace std::literals;

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Ping : public rclcpp::Node {
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

  void ping() {
    auto message = std_msgs::msg::String();
    message.data = "ping";
    publisher_->publish(message);
  }

public:
  Ping() : Node("ping") {
    publisher_ = this->create_publisher<std_msgs::msg::String>(
        "ping", rclcpp::QoS(rclcpp::KeepLast(10)));

    subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "pong", rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const std_msgs::msg::String::SharedPtr msg) {
          std::cout << msg->data << ": received\n"s << std::flush;
          ping();
        });

    ping();
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ping>());
  rclcpp::shutdown();
  return 0;
}
