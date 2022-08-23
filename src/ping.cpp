#include <cassert>
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

//// GLOBALS VARIABLES ON THREADS ->
// lock_guard targets and mutex
static std::filesystem::path data_directory_path_g;
static bool is_measuring_g = false;
static uint measurements_completed_node_counts_g = 0;
static std::mutex mutex_g;

// common mesurement settings, rhs is default value,  set once on start up
static ppm_options options_g;
static uint node_counts_g = 100;
static uint measurement_times_g = 100;
static uint ping_times_g = 100;
static uint payload_bytes_g = 10;
//// <- GLOBALS VARIABLES ON THREADS

class Ping : public rclcpp::Node {

  class measurement {

  private:
    std::chrono::system_clock::time_point send_time_, recv_time_;

  public:
    measurement(std::chrono::system_clock::time_point now) : send_time_(now) {}

    std::chrono::system_clock::time_point &send_time() { return send_time_; }
    std::chrono::system_clock::time_point &recv_time() { return recv_time_; }

    int took_time() {
      return std::chrono::duration_cast<std::chrono::microseconds>(recv_time_ - send_time_).count();
    }
  };

private:
  uint id_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr starter_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr starter_subscriber_;
  std::vector<measurement> measurements_;
  uint ping_counts_ = 0;

  void ping(std::string payload = "ping"s) {
    auto message = std_msgs::msg::String();
    message.data = payload;

    publisher_->publish(message);
  }

  void ping_for_measurement(std::string payload = "ping"s) {
    ping(payload);
    ++ping_counts_;
  }

  void reset_ping_counts() { ping_counts_ = 0; }

  void start_repeat_measurement() { measurements_.emplace_back(std::chrono::system_clock::now()); }

  void stop_repeat_measurement() {
    measurements_.back().recv_time() = std::chrono::system_clock::now();
  }

  std::filesystem::path csv_file_path(const std::filesystem::path &data_directory_path) {
    // zero padding, csv_file_name is like 0099.csv
    const auto id_string = std::to_string(id_);
    const uint padding_digits = 4;
    const auto csv_file_name =
        std::string(padding_digits - id_string.length(), '0') + id_string + ".csv"s;
    return data_directory_path / csv_file_name;
  }

  void dump_measurements_to_csv(const std::filesystem::path &csv_file_path) {
    std::ofstream csv_file_stream(csv_file_path.string());

    // header
    csv_file_stream << "send_time[ms]"s
                    << ","s
                    << "recv_time[ms]"s
                    << ","s
                    << "took_time[ms]"s
                    << ","s
                    << "\n"s;

    // body
    for (auto measurement : measurements_) {

      const auto send_time = time_since_epoch_milliseconds(measurement.send_time());
      const auto recv_time = time_since_epoch_milliseconds(measurement.recv_time());

      csv_file_stream << std::to_string(send_time) << ","s << std::to_string(recv_time) << ","s
                      << std::to_string(measurement.took_time() / 1000.0) << ","s
                      << "\n"s;
    }

    measurements_.clear();
  }

  void ready_measurements() {
    std::lock_guard<std::mutex> lock(mutex_g);

    if (is_measuring_g)
      return;

    data_directory_path_g = create_data_directory(options_g);
    is_measuring_g = true;
  }

  void publish_to_starter(const std::string payload) {
    auto message = std_msgs::msg::String();
    message.data = payload;
    starter_publisher_->publish(message);
  }

  bool is_all_nodes_measurements_completed() {
    std::lock_guard<std::mutex> lock(mutex_g);

    ++measurements_completed_node_counts_g;
    return measurements_completed_node_counts_g == node_counts_g;
  }

  void finish_measurements() {
    std::lock_guard<std::mutex> lock(mutex_g);

    publish_to_starter("measurements completed"s);
    // reset global variables
    is_measuring_g = false;
    data_directory_path_g.clear();
    measurements_completed_node_counts_g = 0;
  }

public:
  Ping(const uint id) : Node(ping_node_name(id)), id_(id) {

    publisher_ = this->create_publisher<std_msgs::msg::String>(ping_topic_name(id_),
                                                               rclcpp::QoS(rclcpp::KeepLast(10)));

    subscriber_ = this->create_subscription<std_msgs::msg::String>(
        pong_topic_name(id_), rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const std_msgs::msg::String::SharedPtr message_pointer) {
          if (ping_counts_ < ping_times_g) {
            ping_for_measurement(message_pointer->data);
            return;
          }

          stop_repeat_measurement();
          reset_ping_counts();

          if (measurements_.size() < measurement_times_g) {
            start_repeat_measurement();
            ping_for_measurement(message_pointer->data);
          } else {
            assert(measurements_.size() == measurement_times_g);

            dump_measurements_to_csv(csv_file_path(data_directory_path_g));

            if (!is_all_nodes_measurements_completed())
              return;

            finish_measurements();
            RCLCPP_INFO(this->get_logger(), "Ctrl + C to exit this program.");
          }
        });

    starter_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "from_ping_to_starter"s, rclcpp::QoS(rclcpp::KeepLast(10)));

    starter_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "from_starter"s, rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const std_msgs::msg::String::SharedPtr message_pointer) {
          const auto command = message_pointer->data;
          if (command == "start"s) {
            ready_measurements();
            start_repeat_measurement();
            ping_for_measurement(std::string(payload_bytes_g, 'a'));
          }
        });
  }

  ~Ping() {
    if (is_measuring_g) {
      dump_measurements_to_csv(csv_file_path(data_directory_path_g));
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  options_g = get_options(argc, argv);
  node_counts_g = get_node_counts(options_g);
  measurement_times_g = get_measurement_times(options_g);
  payload_bytes_g = get_payload_bytes(options_g);

  auto nodes = std::vector<std::shared_ptr<Ping>>(node_counts_g);

  for (auto i = 0u; i < nodes.size(); ++i) {
    nodes.at(i) = std::make_shared<Ping>(i);
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
