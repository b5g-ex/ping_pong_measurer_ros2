#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::literals;

class Starter : public rclcpp::Node {
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;

public:
  Starter(std::string node_name) : Node(node_name) {
    const auto topic_name = "command"s;
    publisher_ = this->create_publisher<std_msgs::msg::String>(topic_name,
                                                               rclcpp::QoS(rclcpp::KeepLast(10)));
    subscriber_ = this->create_subscription<std_msgs::msg::String>(
        topic_name, rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const std_msgs::msg::String::SharedPtr message_pointer) {
          const auto data = message_pointer->data;
          if (data == "measurements completed"s) {
            stop_os_info_measurement();
          }
        });
  }

  void publish_impl(const std::string data) {
    auto message = std_msgs::msg::String();
    message.data = data;
    publisher_->publish(message);
  }

  void start_measurement() { publish_impl("start"s); }
  void stop_measurement() { publish_impl("stop"s); }
  void start_os_info_measurement() { publish_impl("start os info measurement"s); }
  void stop_os_info_measurement() { publish_impl("stop os info measurement"s); }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  const auto node_name = "starter"s;
  auto node = std::make_shared<Starter>(node_name);

  // WE MUST WAIT PUBLISHER START UP
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // start_os_info_measurement first
  node->start_os_info_measurement();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  node->start_measurement();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
