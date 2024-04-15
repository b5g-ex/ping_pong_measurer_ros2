#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
using namespace std::literals;

using ppm_options = std::tuple<uint, uint, uint, std::string, std::string>;

inline std::string ping_node_name(uint id) { return "ping_node"s + std::to_string(id); }
inline std::string pong_node_name(uint id) { return "pong_node"s + std::to_string(id); }

inline std::string ping_topic_name(uint id) { return "ping"s + std::to_string(id); }
inline std::string pong_topic_name(uint id) { return "pong"s + std::to_string(id); }

inline ppm_options get_options(int argc, char *argv[]) {
  uint node_counts = 1;
  uint payload_bytes = 10;
  uint measurement_times = 100;
  std::string ping_pub_type = "single"s;
  std::string ping_sub_type = "single"s;

  for (auto i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--node-counts"s) {
      node_counts = std::stoi(argv[++i]);
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

  return {node_counts, payload_bytes, measurement_times, ping_pub_type, ping_sub_type};
}

inline uint get_node_counts(ppm_options options) { return std::get<0>(options); }
inline uint get_payload_bytes(ppm_options options) { return std::get<1>(options); }
inline uint get_measurement_times(ppm_options options) { return std::get<2>(options); }
inline std::string get_ping_pub_type(ppm_options options) { return std::get<3>(options); }
inline std::string get_ping_sub_type(ppm_options options) { return std::get<4>(options); }

inline std::string get_datetime_utc_now_string(
    std::chrono::system_clock::time_point time_point = std::chrono::system_clock::now()) {
  const auto t = std::chrono::system_clock::to_time_t(time_point);
  const auto tm = std::gmtime(&t);
  char buffer[16];

  // YYYYMMDDHHMMSS
  std::strftime(buffer, sizeof(buffer), "%Y%m%d%H%M%S", tm);
  return std::string(buffer);
}

inline auto time_since_epoch_milliseconds(const std::chrono::system_clock::time_point &tp) {
  return std::chrono::duration_cast<std::chrono::milliseconds>(tp.time_since_epoch()).count();
}

inline std::string create_data_directory_name(const ppm_options &options) {
  const auto nc = std::to_string(get_node_counts(options));
  const auto pb = std::to_string(get_payload_bytes(options));
  const auto mt = std::to_string(get_measurement_times(options));

  return get_datetime_utc_now_string() + "_"s + "nc"s + nc + "_"s + "pb"s + pb + "_"s + "mt"s + mt;
}

inline std::filesystem::path create_data_directory(const std::string &directory_name) {
  const auto data_directory_path = std::filesystem::current_path() / "data"s / directory_name;

  std::filesystem::create_directories(data_directory_path);
  return data_directory_path;
}

inline std::filesystem::path create_data_directory(const ppm_options &options) {
  const auto data_directory_name = create_data_directory_name(options);
  return create_data_directory(data_directory_name);
}

std::string exec(const std::string command) {
  std::array<char, 256> buffer;
  std::string result;
  // custom deleter
  std::unique_ptr<std::FILE, decltype(&pclose)> pipe(popen(command.c_str(), "r"), pclose);
  if (!pipe) {
    throw std::runtime_error("popen() failed!");
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }

  return result;
}

std::string remove_multiple_space(std::string line) {
  auto is_multiple_space = [](char const &lhs, char const &rhs) -> bool {
    return lhs == rhs && iswspace(lhs);
  };
  auto iterator = std::unique(line.begin(), line.end(), is_multiple_space);
  line.erase(iterator, line.end());

  return line;
}
