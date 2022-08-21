#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
using namespace std::literals;

#include "ping_pong_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

static int measurements_completed_node_counts_g = 0;
static int node_counts_g = 0;

class Ping : public rclcpp::Node {

  class measurement {

  private:
    std::chrono::system_clock::time_point send_time_, recv_time_;

  public:
    measurement(std::chrono::system_clock::time_point now) : send_time_(now) {}

    std::chrono::system_clock::time_point &recv_time() { return recv_time_; }

    int took_time() {
      return std::chrono::duration_cast<std::chrono::microseconds>(recv_time_ - send_time_).count();
    }
  };

private:
  uint id_;
  std::filesystem::path data_directory_path_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr starter_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr starter_subscriber_;
  std::vector<measurement> measurements_;
  uint measurement_times_;
  uint measurement_counts_ = 0;
  uint ping_times_;
  uint ping_counts_ = 0;
  uint payload_bytes_;

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

  void stop_measurement() { measurements_.back().recv_time() = std::chrono::system_clock::now(); }

  std::filesystem::path csv_file_path() {
    // zero padding, csv_file_name is like 0100.csv
    const auto id_string = std::to_string(id_);
    const uint padding_digits = 4;
    const auto csv_file_name =
        std::string(padding_digits - id_string.length(), '0') + id_string + ".csv"s;
    return data_directory_path_ / csv_file_name;
  }

  void dump_measurements_to_csv(const std::filesystem::path &csv_file_path) {
    std::ofstream csv_file_stream(csv_file_path.string());

    // header
    csv_file_stream << "took_time[ms]"s
                    << "\n"s;
    // body
    for (auto measurement : measurements_) {
      csv_file_stream << std::to_string(measurement.took_time() / 1000.0) << "\n"s;
    }
  }

  void tell_measurements_completed_to_starter() {
    auto message = std_msgs::msg::String();
    message.data = "measurements completed"s;
    starter_publisher_->publish(message);
  }

public:
  Ping(const uint id, const std::filesystem::path data_directory_path, ppm_options options)
      : Node(ping_node_name(id)), id_(id), data_directory_path_(data_directory_path) {

    measurement_times_ = get_measurement_times(options);
    ping_times_ = 100;
    payload_bytes_ = get_payload_bytes(options);

    publisher_ = this->create_publisher<std_msgs::msg::String>(ping_topic_name(id_),
                                                               rclcpp::QoS(rclcpp::KeepLast(10)));

    subscriber_ = this->create_subscription<std_msgs::msg::String>(
        pong_topic_name(id_), rclcpp::QoS(rclcpp::KeepLast(10)),
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
            if (++measurements_completed_node_counts_g != node_counts_g)
              return;

            tell_measurements_completed_to_starter();
            RCLCPP_INFO(this->get_logger(),
                        "measurements completed, Ctrl + C to exit this program.");
          }
        });

    starter_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "command"s, rclcpp::QoS(rclcpp::KeepLast(10)));

    starter_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "command"s, rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const std_msgs::msg::String::SharedPtr message_pointer) {
          const auto command = message_pointer->data;
          if (command == "start"s) {
            measurements_completed_node_counts_g = 0;
            start_measurement();
            ping_for_measurement(std::string(payload_bytes_, 'a'));
          }
        });
  }

  ~Ping() {
    RCLCPP_INFO(this->get_logger(), "destructed, measurements size is %d."s, measurements_.size());

    if (measurements_.size() > 0) {
      dump_measurements_to_csv(csv_file_path());
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  const ppm_options options = get_options(argc, argv);

  node_counts_g = get_node_counts(options);
  const auto data_directory_name = create_data_directory_name(options);
  const auto data_directory_path = create_data_directory(data_directory_name);
  auto nodes = std::vector<std::shared_ptr<Ping>>(node_counts_g);

  for (auto i = 0u; i < nodes.size(); ++i) {
    nodes.at(i) = std::make_shared<Ping>(i, data_directory_path, options);
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
