#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
using namespace std::literals;

// #include "ping_pong_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using options = std::tuple<uint, uint, uint, std::string, std::string>;

//// GLOBALS VARIABLES ON THREADS ->
// lock_guard targets and mutex
static std::filesystem::path data_directory_path_g;
static bool is_measuring_g = false;
static uint measurements_completed_node_counts_g = 0;
static std::mutex mutex_g;

// common mesurement settings, rhs is default value,  set once on start up
static options options_g;
static uint pong_node_count_g = 100;
static uint measurement_times_g = 100;
static uint ping_times_g = 100;
static uint payload_bytes_g = 10;
//// <- GLOBALS VARIABLES ON THREADS

inline options get_options(int argc, char *argv[]) {
  uint pong_node_count = 1;
  uint payload_bytes = 10;
  uint measurement_times = 100;
  std::string ping_pub_type = "single"s;
  std::string ping_sub_type = "single"s;

  for (auto i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--pong-node-count"s) {
      pong_node_count = std::stoi(argv[++i]);
    } else if (std::string(argv[i]) == "--payload-bytes"s) {
      payload_bytes = std::stoi(argv[++i]);
    } else if (std::string(argv[i]) == "--measurement-times"s) {
      measurement_times = std::stoi(argv[++i]);
    } else if (std::string(argv[i]) == "--pub"s) {
      ping_pub_type = std::string(argv[++i]);
    } else if (std::string(argv[i]) == "--sub"s) {
      ping_sub_type = std::string(argv[++i]);
    }
  }

  return {pong_node_count, payload_bytes, measurement_times, ping_pub_type, ping_sub_type};
}

inline uint get_pong_node_count(options options) { return std::get<0>(options); }
inline uint get_payload_bytes(options options) { return std::get<1>(options); }
inline uint get_measurement_times(options options) { return std::get<2>(options); }
inline std::string get_pub_type(options options) { return std::get<3>(options); }
inline std::string get_sub_type(options options) { return std::get<4>(options); }

class Ping : public rclcpp::Node {

  class measurement {
  public:
    measurement(uint pong_node_count, std::string pub_type) {
      if (pub_type == "single"s) {
        send_times.resize(1);
        recv_times.resize(pong_node_count);
      } else if (pub_type == "multiple"s) {
        send_times.resize(pong_node_count);
        recv_times.resize(pong_node_count);
      } else {
        throw std::runtime_error("pub_type is invalid!!!");
      }
    }
    std::vector<std::chrono::system_clock::time_point> send_times;
    std::vector<std::chrono::system_clock::time_point> recv_times;
  };

private:
  uint id_;
  std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> publishers_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscribers_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr starter_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr starter_subscriber_;
  std::vector<measurement *> measurements_;
  measurement *current_measurement_ = nullptr;
  uint ping_counts_ = 0;
  std::mutex pong_count_mutex_;
  uint pong_count_ = 0;
  uint measurement_count_ = 0;
  uint pong_node_count_ = 0;
  std::string pub_type_ = "single"s;
  std::string sub_type_ = "single"s;

  void ping(const std::string payload) {
    auto message = std_msgs::msg::String();
    message.data = payload;

    // TODO: publisher の並びで send_times をママつめてよいかは要確認し、修正すること
    uint i = 0;
    for (const auto publisher : publishers_) {
      current_measurement_->send_times.at(i) = std::chrono::system_clock::now();
      publisher->publish(message);
      ++i;
    }
  }

  void ping_for_measurement(const std::string payload) {
    ping(payload);
    ++ping_counts_;
  }

  void reset_ping_counts() { ping_counts_ = 0; }

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
    /*
    csv_file_stream << "send_time[ms]"s
                    << ","s
                    << "recv_time[ms]"s
                    << ","s
                    << "took_time[ms]"s
                    << ","s
                    << "\n"s;
    */

    // body
    for (auto measurement : measurements_) {
      for (auto send_time : measurement->send_times) {
        auto count =
            std::chrono::duration_cast<std::chrono::microseconds>(send_time.time_since_epoch())
                .count();
        csv_file_stream << std::to_string(count) << ","s;
      }
      for (auto recv_time : measurement->recv_times) {
        auto count =
            std::chrono::duration_cast<std::chrono::microseconds>(recv_time.time_since_epoch())
                .count();
        csv_file_stream << std::to_string(count) << ","s;
      }
      for (auto i = 0lu; i < measurement->recv_times.size(); ++i) {
        std::chrono::system_clock::time_point recv_time = measurement->recv_times.at(i);
        std::chrono::system_clock::time_point send_time;
        if (measurement->send_times.size() == 1) {
          send_time = measurement->send_times.at(0);
        } else {
          send_time = measurement->send_times.at(i);
        }

        auto rc =
            std::chrono::duration_cast<std::chrono::microseconds>(recv_time.time_since_epoch())
                .count();
        auto sc =
            std::chrono::duration_cast<std::chrono::microseconds>(send_time.time_since_epoch())
                .count();

        csv_file_stream << std::to_string(rc - sc) << ","s;
      }
      csv_file_stream << "\n"s;
    }

    csv_file_stream.flush();
    for (auto measurement : measurements_) {
      delete measurement;
    }
  }

  /*
    void cut_measurements_start_line() {
      std::lock_guard<std::mutex> lock(mutex_g);

      if (is_measuring_g)
        return;

      data_directory_path_g = create_data_directory(options_g);
      is_measuring_g = true;
      RCLCPP_INFO(this->get_logger(), "measurements start line is cut!! GOGO!!!");
    }
  */
  void publish_to_starter(const std::string payload) {
    auto message = std_msgs::msg::String();
    message.data = payload;
    starter_publisher_->publish(message);
  }

  bool is_all_nodes_measurements_completed() {
    std::lock_guard<std::mutex> lock(mutex_g);

    ++measurements_completed_node_counts_g;
    return measurements_completed_node_counts_g == pong_node_count_g;
  }

  void finish_a_measurement() {
    std::lock_guard<std::mutex> lock(mutex_g);

    publish_to_starter("a measurement completed"s);
    measurements_completed_node_counts_g = 0;
  }

  void finish_measurements() {
    std::lock_guard<std::mutex> lock(mutex_g);

    publish_to_starter("measurements completed"s);
    // reset global variables
    measurements_completed_node_counts_g = 0;
    is_measuring_g = false;
    data_directory_path_g.clear();
  }

  static std::string ping_node_name_() { return "ping"s; }

  static std::string ping_topic_name_(uint i) {
    std::ostringstream index;
    index << std::setfill('0') << std::setw(3) << std::to_string(i);
    return "/ping"s + index.str();
  }

  static std::string pong_topic_name_(uint i) {
    std::ostringstream index;
    index << std::setfill('0') << std::setw(3) << std::to_string(i);
    return "/pong"s + index.str();
  }

public:
  Ping(const uint pong_node_count, std::string pub_type, std::string sub_type)
      : Node(ping_node_name_()), pong_node_count_(pong_node_count), pub_type_(pub_type),
        sub_type_(sub_type) {

    if (pub_type == "single"s) {
      publishers_.push_back(this->create_publisher<std_msgs::msg::String>(
          ping_topic_name_(0), rclcpp::QoS(rclcpp::KeepLast(10))));
    } else if (pub_type == "multiple"s) {
      for (auto i = 0u; i < pong_node_count; ++i) {
        publishers_.push_back(this->create_publisher<std_msgs::msg::String>(
            ping_topic_name_(i), rclcpp::QoS(rclcpp::KeepLast(10))));
      }
    } else {
      throw std::runtime_error("pub_type is invalid!!!");
    }

    auto callback = [this](const std_msgs::msg::String::SharedPtr message_pointer) {
      // ここで計測する
      auto i = std::stoi(message_pointer->data.substr(0, 3));
      current_measurement_->recv_times.at(i) = std::chrono::system_clock::now();

      std::lock_guard<std::mutex> lock(pong_count_mutex_);
      if (++pong_count_ == pong_node_count_g) {
        measurements_.push_back(current_measurement_);
        if (++measurement_count_ < measurement_times_g) {
          RCLCPP_INFO(this->get_logger(), "GO NEXT %d/%d", measurement_count_, measurement_times_g);
          publish_to_starter("a measurement completed"s);
        } else {
          RCLCPP_INFO(this->get_logger(), "THE END %d/%d", measurement_count_, measurement_times_g);
          dump_measurements_to_csv("data/test.csv");
        }
        pong_count_ = 0;
      }
    };

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group =
        create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    if (sub_type == "single"s) {
      subscribers_.push_back(this->create_subscription<std_msgs::msg::String>(
          pong_topic_name_(0), rclcpp::QoS(rclcpp::KeepLast(10)), callback, subscription_options));
    } else if (sub_type == "multiple"s) {
      for (auto i = 0u; i < pong_node_count; ++i) {
        subscribers_.push_back(this->create_subscription<std_msgs::msg::String>(
            pong_topic_name_(i), rclcpp::QoS(rclcpp::KeepLast(10)), callback,
            subscription_options));
      }
    } else {
      throw std::runtime_error("sub_type is invalid!!!");
    }

    starter_publisher_ = this->create_publisher<std_msgs::msg::String>(
        "from_ping_to_starter"s, rclcpp::QoS(rclcpp::KeepLast(10)));

    starter_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "from_starter"s, rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const std_msgs::msg::String::SharedPtr message_pointer) {
          const auto command = message_pointer->data;
          if (command == "start"s) {
            current_measurement_ = new measurement(pong_node_count_, pub_type_);
            ping(std::string(payload_bytes_g, '0'));
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
  pong_node_count_g = get_pong_node_count(options_g);
  measurement_times_g = get_measurement_times(options_g);
  payload_bytes_g = get_payload_bytes(options_g);
  const auto pub_type = get_pub_type(options_g);
  const auto sub_type = get_sub_type(options_g);

  auto node = std::make_shared<Ping>(pong_node_count_g, pub_type, sub_type);

  // TODO: executor のスレッド数はいくつにするべきか要検討
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), pong_node_count_g);
  executor.add_node(node);

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
