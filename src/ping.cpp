#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
using namespace std::literals;

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using options = std::tuple<uint, uint, uint, std::string, std::string>;

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
  std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> publishers_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscribers_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr starter_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr starter_subscriber_;
  std::vector<measurement *> measurements_;
  measurement *current_measurement_ = nullptr;
  std::mutex pong_count_mutex_;
  uint pong_count_ = 0;
  uint measurement_count_ = 0;

  // initialized by constructor
  uint pong_node_count_;
  std::string pub_type_;
  std::string sub_type_;
  uint measurement_times_;
  uint payload_bytes_;

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

  void dump_measurements_to_csv(const std::filesystem::path &csv_file_path) {
    std::ofstream csv_file_stream(csv_file_path.string());

    // header
    {
      const auto m = measurements_.at(0);
      for (auto i = 0lu; i < m->send_times.size(); ++i) {
        std::ostringstream index;
        index << std::setfill('0') << std::setw(3) << std::to_string(i);
        csv_file_stream << "st_"s << index.str() << "[us],"s;
      }
      for (auto i = 0lu; i < m->recv_times.size(); ++i) {
        std::ostringstream index;
        index << std::setfill('0') << std::setw(3) << std::to_string(i);
        csv_file_stream << "rt_"s << index.str() << "[us],"s;
      }
      for (auto i = 0lu; i < m->recv_times.size(); ++i) {
        std::ostringstream index;
        index << std::setfill('0') << std::setw(3) << std::to_string(i);
        csv_file_stream << "tt_"s << index.str() << "[us],"s;
      }
      csv_file_stream << "\n"s;
    }

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

  void publish_to_starter(const std::string payload) {
    auto message = std_msgs::msg::String();
    message.data = payload;
    starter_publisher_->publish(message);
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
  Ping(const uint pong_node_count, std::string pub_type, std::string sub_type,
       const uint measurement_times, const uint payload_bytes)
      : Node(ping_node_name_()), pong_node_count_(pong_node_count), pub_type_(pub_type),
        sub_type_(sub_type), measurement_times_(measurement_times), payload_bytes_(payload_bytes) {

    // publisher を作成
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

    // subscription の callback を定義
    auto callback = [this](const std_msgs::msg::String::SharedPtr message_pointer) {
      // 計測
      auto i = std::stoi(message_pointer->data.substr(0, 3));
      current_measurement_->recv_times.at(i) = std::chrono::system_clock::now();

      std::lock_guard<std::mutex> lock(pong_count_mutex_);
      if (++pong_count_ < pong_node_count_)
        return;

      // 全ての pong を受信したら、次回計測判定をする
      assert(pong_count_ == pong_node_count_);
      measurements_.push_back(current_measurement_);
      if (++measurement_count_ < measurement_times_) {
        RCLCPP_INFO(this->get_logger(), "GO NEXT %d/%d", measurement_count_, measurement_times_);
        publish_to_starter("a measurement completed"s);
      } else {
        RCLCPP_INFO(this->get_logger(), "THE END %d/%d", measurement_count_, measurement_times_);
        dump_measurements_to_csv("data/test.csv");
        publish_to_starter("measurements completed"s);
        RCLCPP_INFO(this->get_logger(), "Ctrl + C to exit this program.");
      }
      pong_count_ = 0;
    };

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group =
        create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // subscription を作成
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
            ping(std::string(payload_bytes_, '0'));
          }
        });
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  const auto options = get_options(argc, argv);
  const auto pong_node_count = get_pong_node_count(options);
  const auto measurement_times = get_measurement_times(options);
  const auto payload_bytes = get_payload_bytes(options);
  const auto pub_type = get_pub_type(options);
  const auto sub_type = get_sub_type(options);

  auto node =
      std::make_shared<Ping>(pong_node_count, pub_type, sub_type, measurement_times, payload_bytes);

  // TODO: executor のスレッド数はいくつにするべきか要検討
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), pong_node_count);
  executor.add_node(node);

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
