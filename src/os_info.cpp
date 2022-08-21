#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
using namespace std::literals;
using namespace std::chrono_literals;

#include "ping_pong_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class OsInfo : public rclcpp::Node {
  class cpu_info {
  private:
    std::chrono::system_clock::time_point measurement_time_;
    std::string data_;

  public:
    cpu_info(std::chrono::system_clock::time_point measurement_time, std::string data)
        : measurement_time_(measurement_time), data_(data) {}

    std::chrono::system_clock::time_point measurement_time() const { return measurement_time_; }
    std::string data() const { return data_; }

    static std::string get_cpu_line() {
      std::ifstream proc_stat("/proc/stat");
      std::string line;
      //"cpu  228602 66 124305 99146269 4121 0 1163 0 0 0";
      std::getline(proc_stat, line);

      return line;
    }

    static std::string convert_cpu_line(std::string line) {
      //"228602 66 124305 99146269 4121 0 1163 0 0 0";
      line = line.substr("cpu  "s.length(), line.length());
      //"228602,66,124305,99146269,4121,0,1163,0,0,0";
      std::replace(line.begin(), line.end(), ' ', ',');

      return line;
    }

    static void dump_to_csv(std::filesystem::path data_directory_path,
                            std::vector<cpu_info> &cpu_infos) {
      auto csv_file_path = data_directory_path / "cpu.csv";
      std::ofstream csv_file_stream(csv_file_path.string());

      // header, see man 5 proc, /proc/stat
      csv_file_stream << "measurement_time[ms]"
                      << ","s
                      << "user,nice,system,idle,iowait,irq,softirq,steal,guest,guest_nice"
                      << ","s
                      << "\n"s;
      // body
      for (auto cpu_info : cpu_infos) {
        const auto measurement_time = time_since_epoch_milliseconds(cpu_info.measurement_time());
        csv_file_stream << std::to_string(measurement_time) << ","s << cpu_info.data() << "\n"s;
      }

      cpu_infos.clear();
    }
  };

  class memory_info {
  private:
    std::chrono::system_clock::time_point measurement_time_;
    std::string data_;

  public:
    memory_info(std::chrono::system_clock::time_point measurement_time, std::string data)
        : measurement_time_(measurement_time), data_(data) {}

    std::chrono::system_clock::time_point measurement_time() const { return measurement_time_; }
    std::string data() const { return data_; }

    static std::string get_memory_line() {
      std::stringstream result(exec("free"s));
      std::string line;
      // "              total        used        free      shared  buff/cache   available"
      std::getline(result, line); // abandon header
      // "Mem:       32789156     3483060    24676784       38796     4629312    28879052"
      std::getline(result, line);

      return line;
    }

    static std::string convert_memory_line(std::string line) {
      // "Mem: 32789156 3483060 24676784 38796 4629312 28879052"
      line = remove_multiple_space(line);
      // "32789156 3483060 24676784 38796 4629312 28879052"
      line = line.substr("Mem: "s.length(), line.length());
      // "32789156,3483060,24676784,38796,4629312,28879052"
      std::replace(line.begin(), line.end(), ' ', ',');

      return line;
    }

    static void dump_to_csv(std::filesystem::path data_directory_path,
                            std::vector<memory_info> &memory_infos) {
      auto csv_file_path = data_directory_path / "memory.csv";
      std::ofstream csv_file_stream(csv_file_path.string());

      csv_file_stream << "measurement_time[ms]"
                      << ","s
                      << "total,used,free,shared,buff/cache,available"
                      << ","s
                      << "\n"s;
      // body
      for (auto memory_info : memory_infos) {
        const auto measurement_time = time_since_epoch_milliseconds(memory_info.measurement_time());
        csv_file_stream << std::to_string(measurement_time) << ","s << memory_info.data() << ","s
                        << "\n"s;
      }

      memory_infos.clear();
    }
  };

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
  std::filesystem::path data_directory_path_;
  std::vector<cpu_info> cpu_infos_{};
  std::vector<memory_info> memory_infos_{};
  bool is_measuring_ = false;
  void start_measurement() {
    const auto data_directory_name = get_datetime_utc_now_string();
    data_directory_path_ = create_data_directory(data_directory_name);
    is_measuring_ = true;
  }
  void stop_measurement() {
    is_measuring_ = false;
    dump_measurements_to_csv(data_directory_path_);
  }

  void dump_measurements_to_csv(const std::filesystem::path data_directory_path) {
    cpu_info::dump_to_csv(data_directory_path, cpu_infos_);
    memory_info::dump_to_csv(data_directory_path, memory_infos_);
  }

  void measure_cpu() {
    auto line = cpu_info::get_cpu_line();
    line = cpu_info::convert_cpu_line(line);
    cpu_infos_.emplace_back(std::chrono::system_clock::now(), line);
  }

  void measure_memory() {
    auto line = memory_info::get_memory_line();
    line = memory_info::convert_memory_line(line);
    memory_infos_.emplace_back(std::chrono::system_clock::now(), line);
  }

public:
  OsInfo() : Node("os_info") {
    timer_ = this->create_wall_timer(10ms, [this]() {
      if (!is_measuring_)
        return;

      measure_cpu();
      measure_memory();
    });

    subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "command"s, rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](const std_msgs::msg::String::SharedPtr message_pointer) {
          const auto command = message_pointer->data;
          std::cout << command << " received"s << std::endl;
          if (command == "start os info measurement"s) {
            start_measurement();
          } else if (command == "stop os info measurement"s) {
            stop_measurement();
          }
        });
  }

  ~OsInfo() {
    if (is_measuring_)
      stop_measurement();
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OsInfo>());
  rclcpp::shutdown();
  return 0;
}
