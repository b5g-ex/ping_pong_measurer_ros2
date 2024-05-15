#include <cassert>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
using namespace std::literals;

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "measurements.hpp"

class Options {
private:
  uint pong_node_count_ = 1;
  uint payload_bytes_ = 10;
  uint measurement_times_ = 100;
  std::string ping_pub_type_ = "single"s;
  std::string ping_sub_type_ = "single"s;
  bool enable_os_info_measuring_ = false;

public:
  Options(int argc, char *argv[]) {
    for (auto i = 1; i < argc; ++i) {
      if (std::string(argv[i]) == "--pong-node-count"s) {
        pong_node_count_ = std::stoi(argv[++i]);
      } else if (std::string(argv[i]) == "--payload-bytes"s) {
        payload_bytes_ = std::stoi(argv[++i]);
      } else if (std::string(argv[i]) == "--measurement-times"s) {
        measurement_times_ = std::stoi(argv[++i]);
      } else if (std::string(argv[i]) == "--ping-pub"s) {
        ping_pub_type_ = std::string(argv[++i]);
      } else if (std::string(argv[i]) == "--ping-sub"s) {
        ping_sub_type_ = std::string(argv[++i]);
      } else if (std::string(argv[i]) == "--enable-os-info-measuring"s) {
        enable_os_info_measuring_ = true;
      }
    }
  }

  uint pong_node_count() const { return pong_node_count_; }
  uint payload_bytes() const { return payload_bytes_; }
  uint measurement_times() const { return measurement_times_; }
  std::string ping_pub_type() const { return ping_pub_type_; }
  std::string ping_sub_type() const { return ping_sub_type_; }
  bool enable_os_info_measuring() const { return enable_os_info_measuring_; }
};

class Ping : public rclcpp::Node {
private:
  std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> publishers_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscribers_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr starter_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr starter_subscriber_;
  Measurements measurements_;
  std::mutex pong_count_mutex_;
  uint pong_count_ = 0;
  uint measurement_count_ = 0;
  FILE *os_info_measurer_ = nullptr;

  // initialized by constructor
  uint pong_node_count_;
  std::string pub_type_;
  std::string sub_type_;
  uint measurement_times_;
  uint payload_bytes_;
  bool enable_os_info_measuring_;

  void ping(const std::string payload) {
    auto message = std_msgs::msg::String();
    message.data = payload;
    std::vector<std::thread> threads;
    std::condition_variable cv;
    std::mutex mutex;
    std::vector<bool> readies(publishers_.size(), false);

    for (const auto &publisher : publishers_) {
      threads.emplace_back([&]() {
        // ex. /ping010 -> 010 -> 10
        std::string topic_name = publisher->get_topic_name();
        uint i = std::stoi(topic_name.substr(5, 3));

        std::unique_lock<std::mutex> lock(mutex);
        readies.at(i) = true;
        if (std::all_of(readies.begin(), readies.end(), [](bool b) { return b; })) {
          cv.notify_all();
        } else {
          cv.wait(lock);
        }
        if (!enable_os_info_measuring_) {
          measurements_.measure_send(i);
        }
        publisher->publish(message);
      });
    }

    for (auto &thread : threads) {
      thread.join();
    }
  }

  void publish_to_starter(const std::string payload) {
    auto message = std_msgs::msg::String();
    message.data = payload;
    starter_publisher_->publish(message);
  }

  std::string measurement_id() {
    std::ostringstream pong_node_count;
    pong_node_count << std::setfill('0') << std::setw(3) << std::to_string(pong_node_count_);
    std::string payload_bytes = std::to_string(payload_bytes_);
    return "rclcpp"s + "_" + pong_node_count.str() + "_"s + pub_type_ + "_"s + sub_type_ + "_"s +
           payload_bytes;
  }

  std::filesystem::path data_directory_path() {
    std::filesystem::path data_directory_path = "data";
    return data_directory_path / measurement_id();
  }

  std::string csv_file_name() { return measurement_id() + ".csv"s; }

  static std::string ping_node_name_() { return "ping000"s; }

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

  void start_os_info_measurement() {
    std::system("pwd");

    std::string bin_path = "../os_info_measurer/_build/dev/lib/os_info_measurer/priv/measurer";
    std::string directory_path = data_directory_path();
    std::string file_name_prefix = measurement_id() + "_";
    std::string intervals_ms = "100";

    std::string command =
        bin_path + " -d " + directory_path + " -f " + file_name_prefix + " -i " + intervals_ms;

    os_info_measurer_ = popen(command.c_str(), "w");
    std::string start = "start\n";
    fwrite(start.c_str(), sizeof(char), start.size(), os_info_measurer_);
    fflush(os_info_measurer_);
  }
  void stop_os_info_measurement() { pclose(os_info_measurer_); }

public:
  Ping(const uint pong_node_count, std::string pub_type, std::string sub_type,
       const uint measurement_times, const uint payload_bytes, bool enable_os_info_measuring)
      : Node(ping_node_name_()), pong_node_count_(pong_node_count), pub_type_(pub_type),
        sub_type_(sub_type), measurement_times_(measurement_times), payload_bytes_(payload_bytes),
        enable_os_info_measuring_(enable_os_info_measuring) {

    // publisher を作成
    if (pub_type == "single"s) {

      if (!enable_os_info_measuring_) {
        measurements_.resize_measurement(1, pong_node_count);
      }

      publishers_.push_back(this->create_publisher<std_msgs::msg::String>(
          ping_topic_name_(0), rclcpp::QoS(rclcpp::KeepLast(10))));
    } else if (pub_type == "multiple"s) {
      if (!enable_os_info_measuring_) {
        measurements_.resize_measurement(pong_node_count, pong_node_count);
      }

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
      if (!enable_os_info_measuring_) {
        measurements_.measure_recv(i);
      }

      std::lock_guard<std::mutex> lock(pong_count_mutex_);
      if (++pong_count_ < pong_node_count_)
        return;

      // 全ての pong を受信したら、次回計測判定をする
      assert(pong_count_ == pong_node_count_);
      if (!enable_os_info_measuring_) {
        measurements_.prepare_next_measurement();
      }
      if (++measurement_count_ < measurement_times_) {
        RCLCPP_INFO(this->get_logger(), "GO NEXT %d/%d", measurement_count_, measurement_times_);
        publish_to_starter("a measurement completed"s);
      } else {
        RCLCPP_INFO(this->get_logger(), "THE END %d/%d", measurement_count_, measurement_times_);

        if (enable_os_info_measuring_) {
          // OS 情報を 1s 余分に計測
          std::this_thread::sleep_for(1s);
          stop_os_info_measurement();
        } else {
          measurements_.dump_to_csv(data_directory_path(), csv_file_name());
        }
        publish_to_starter("measurements completed"s);

        rclcpp::shutdown();
      }
      pong_count_ = 0;
    };

    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group =
        create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // subscription を作成
    if (sub_type == "single"s) {
      subscribers_.push_back(this->create_subscription<std_msgs::msg::String>(
          pong_topic_name_(0), rclcpp::QoS(rclcpp::KeepLast(100)), callback, subscription_options));
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
            if (measurement_count_ == 0) { // 初回のみ実行
              if (enable_os_info_measuring_) {
                // OS 情報を 1s 余分に計測
                start_os_info_measurement();
                std::this_thread::sleep_for(1s);
              }
            }

            ping(std::string(payload_bytes_, '0'));
          }
        });
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  const auto opts = Options(argc, argv);

  auto node = std::make_shared<Ping>(opts.pong_node_count(), opts.ping_pub_type(),
                                     opts.ping_sub_type(), opts.measurement_times(),
                                     opts.payload_bytes(), opts.enable_os_info_measuring());

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
