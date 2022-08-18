#include <iostream>
#include <memory>
#include <string>
using namespace std::literals;

#include "ping_pong_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Ping : public rclcpp::Node {

  class measurement {

  private:
    std::chrono::system_clock::time_point send_time_, recv_time_;

  public:
    measurement(std::chrono::system_clock::time_point now) : send_time_(now) {}

    std::chrono::system_clock::time_point &recv_time() { return recv_time_; }

    int took_time() {
      auto took_time =
          std::chrono::duration_cast<std::chrono::microseconds>(recv_time_ - send_time_).count();
      std::cout << took_time << "\n"s;
      return took_time;
    }
  };

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
  std::vector<measurement> measurements_;
  uint measurement_times_;
  uint measurement_counts_ = 0;
  uint ping_times_;
  uint ping_counts_ = 0;
  uint payload_size_byte_;

  void ping(std::string payload = "ping"s) {
    auto message_pointer = std::make_unique<std_msgs::msg::String>();
    message_pointer->data = payload;

    publisher_->publish(*message_pointer);
  }

  void ping_for_measurement(std::string payload = "ping"s) {
    ping(payload);
    ++ping_counts_;
  }

  void reset_ping_counts() { ping_counts_ = 0; }

  void start_measurement() {
    measurements_.emplace_back(std::chrono::system_clock::now());
    ++measurement_counts_;
  }
  void stop_measurement() {
    measurements_.back().recv_time() = std::chrono::system_clock::now();
    measurements_.back().took_time();
  }

public:
  Ping(const std::string node_name, const std::string ping_topic_name,
       const std::string pong_topic_name)
      : Node(node_name) {

    this->declare_parameter("measurement_times"s, 100);
    this->declare_parameter("ping_times"s, 100);
    this->declare_parameter("payload_size_byte"s, 10);

    measurement_times_ = this->get_parameter("measurement_times"s).as_int();
    ping_times_ = this->get_parameter("ping_times"s).as_int();
    payload_size_byte_ = this->get_parameter("payload_size_byte"s).as_int();

    publisher_ = this->create_publisher<std_msgs::msg::String>(ping_topic_name,
                                                               rclcpp::QoS(rclcpp::KeepLast(10)));

    subscriber_ = this->create_subscription<std_msgs::msg::String>(
        pong_topic_name, rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const std_msgs::msg::String::SharedPtr message_pointer) {
          if (ping_counts_ < ping_times_) {
            ping_for_measurement(message_pointer->data);
            return;
          }

          stop_measurement();

          if (measurement_counts_ < measurement_times_) {
            reset_ping_counts();
            start_measurement();
            ping_for_measurement(message_pointer->data);
          } else {
            RCLCPP_INFO(this->get_logger(),
                        "measurement completed, Ctrl + C to exit this program.");
          }
        });

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    start_measurement();
    ping_for_measurement(std::string(payload_size_byte_, 'a'));
  }

  ~Ping() { RCLCPP_INFO(this->get_logger(), "destructed %d"s, measurements_.size()); }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node_counts = 10;
  auto nodes = std::vector<std::shared_ptr<Ping>>{};

  for (auto i = 0; i < node_counts; ++i) {
    nodes.push_back(
        std::make_shared<Ping>(ping_node_name(i), ping_topic_name(i), pong_topic_name(i)));
  }

  auto thread_counts = nodes.size();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), thread_counts);
  std::for_each(std::begin(nodes), std::end(nodes),
                [&executor](auto node) { executor.add_node(node); });

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
