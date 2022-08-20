#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::literals;

class Starter : public rclcpp::Node {
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

public:
  Starter(std::string node_name) : Node(node_name) {
    const auto topic_name = "command"s;
    publisher_ = this->create_publisher<std_msgs::msg::String>(topic_name,
                                                               rclcpp::QoS(rclcpp::KeepLast(10)));
  }

  void start_measurement() {
    auto message = std_msgs::msg::String();
    message.data = "start"s;
    publisher_->publish(message);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  const auto node_name = "starter"s;
  auto node = std::make_shared<Starter>(node_name);

  node->start_measurement();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
