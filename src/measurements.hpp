#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sys/types.h>
#include <vector>
using namespace std::literals;

class Measurements {
  class Measurement {
  public:
    std::vector<std::chrono::system_clock::time_point> send_times;
    std::vector<std::chrono::system_clock::time_point> recv_times;

    Measurement() {}

    void resize(uint send_times, uint recv_times) {
      this->send_times.resize(send_times);
      this->recv_times.resize(recv_times);
    }

    void reset() {
      std::chrono::system_clock::time_point default_time_point;
      std::fill(send_times.begin(), send_times.end(), default_time_point);
      std::fill(recv_times.begin(), recv_times.end(), default_time_point);
    }
  };

private:
  Measurement current_measurement_;
  std::vector<Measurement> measurements_;

public:
  Measurements(){};

  void resize_measurement(uint send_times, uint recv_times) {
    current_measurement_.resize(send_times, recv_times);
  };

  void measure_send(uint index) {
    current_measurement_.send_times.at(index) = std::chrono::system_clock::now();
  }

  void measure_recv(uint index) {
    current_measurement_.recv_times.at(index) = std::chrono::system_clock::now();
  }

  void prepare_next_measurement() {
    measurements_.emplace_back(current_measurement_);
    current_measurement_.reset();
  }

  void dump_to_csv(const std::filesystem::path &data_directory_path, const std::string &file_name) {
    if (!std::filesystem::exists(data_directory_path)) {
      std::filesystem::create_directories(data_directory_path);
    }

    std::filesystem::path csv_file_path = data_directory_path / file_name;
    std::ofstream csv_file_stream(csv_file_path.string());

    // header
    {
      const auto m = measurements_.at(0);
      for (auto i = 0lu; i < m.send_times.size(); ++i) {
        std::ostringstream index;
        index << std::setfill('0') << std::setw(3) << std::to_string(i);
        csv_file_stream << "st_"s << index.str() << "[us],"s;
      }
      for (auto i = 0lu; i < m.recv_times.size(); ++i) {
        std::ostringstream index;
        index << std::setfill('0') << std::setw(3) << std::to_string(i);
        csv_file_stream << "rt_"s << index.str() << "[us],"s;
      }
      for (auto i = 0lu; i < m.recv_times.size(); ++i) {
        std::ostringstream index;
        index << std::setfill('0') << std::setw(3) << std::to_string(i);
        csv_file_stream << "tt_"s << index.str() << "[us],"s;
      }
      csv_file_stream << "\n"s;
    }

    // body
    for (const auto m : measurements_) {
      for (const auto send_time : m.send_times) {
        auto count =
            std::chrono::duration_cast<std::chrono::microseconds>(send_time.time_since_epoch())
                .count();
        csv_file_stream << std::to_string(count) << ","s;
      }
      for (auto recv_time : m.recv_times) {
        auto count =
            std::chrono::duration_cast<std::chrono::microseconds>(recv_time.time_since_epoch())
                .count();
        csv_file_stream << std::to_string(count) << ","s;
      }
      for (auto i = 0lu; i < m.recv_times.size(); ++i) {
        std::chrono::system_clock::time_point recv_time = m.recv_times.at(i);
        std::chrono::system_clock::time_point send_time;
        if (m.send_times.size() == 1) {
          send_time = m.send_times.at(0);
        } else {
          send_time = m.send_times.at(i);
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
  };
};
