#include <iostream>
#include <memory>
#include <string>
using namespace std::literals;

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
      auto took_time = std::chrono::duration_cast<std::chrono::microseconds>(
                           recv_time_ - send_time_)
                           .count();
      std::cout << took_time << "\n"s;
      return took_time;
    }
  };

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
  std::vector<measurement> measurements_;
  uint measurement_times = 100;
  uint measurement_counts = 0;
  uint ping_times = 100;
  uint ping_counts = 0;

  void ping() {
    auto message = std_msgs::msg::String();
    message.data = "ping";

    publisher_->publish(message);
  }

  void start_measurement() {
    measurements_.emplace_back(std::chrono::system_clock::now());
  }
  void stop_measurement() {
    measurements_.back().recv_time() = std::chrono::system_clock::now();
    measurements_.back().took_time();
  }

public:
  Ping() : Node("ping"), measurement_times(100), ping_times(100) {
    publisher_ = this->create_publisher<std_msgs::msg::String>(
        "ping", rclcpp::QoS(rclcpp::KeepLast(10)));

    subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "pong", rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const std_msgs::msg::String::SharedPtr msg) {
          if (ping_counts < ping_times) {
            ping();
            ++ping_counts;
            return;
          }

          stop_measurement();
          ++measurement_counts;

          if (measurement_counts < measurement_times) {
            ping_counts = 0;
            start_measurement();
            ping();
            ++ping_counts;
          }
        });

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    start_measurement();
    ping();
    ++ping_counts;
  }

  ~Ping() {
    RCLCPP_INFO(this->get_logger(), "destructed %d"s, measurements_.size());
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ping>());
  rclcpp::shutdown();
  return 0;
}
