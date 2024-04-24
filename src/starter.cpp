#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::literals;

class Starter : public rclcpp::Node {
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint measurement_counts_ = 0;

public:
  Starter(std::string node_name) : Node(node_name) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("from_starter"s,
                                                               rclcpp::QoS(rclcpp::KeepLast(10)));

    subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "from_ping_to_starter"s, rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const std_msgs::msg::String::SharedPtr message_pointer) {
          const auto data = message_pointer->data;

          if (data == "a measurement completed"s) {
            RCLCPP_INFO(this->get_logger(),
                        "%s received, measure next, current measurement counts is %d.",
                        data.c_str(), ++measurement_counts_);
            start_measurement();
          } else if (data == "measurements completed"s) {
            RCLCPP_INFO(this->get_logger(), "%s received, current measurement counts is %d.",
                        data.c_str(), ++measurement_counts_);
            RCLCPP_INFO(this->get_logger(), "Ctrl + C to exit this program.");
          }
        });

    // create one shot timer
    timer_ = this->create_wall_timer(0s, [this]() {
      std::cout << "start measurement"s << std::endl;
      start_measurement();
      timer_->cancel();
    });
  }

  void publish_impl(const std::string data) {
    auto message = std_msgs::msg::String();
    message.data = data;
    publisher_->publish(message);
  }

  void start_measurement() { publish_impl("start"s); }
  void stop_measurement() { publish_impl("stop"s); }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  const auto node_name = "starter"s;
  auto node = std::make_shared<Starter>(node_name);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
