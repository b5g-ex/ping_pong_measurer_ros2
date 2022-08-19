#include <chrono>
#include <filesystem>
#include <string>
using namespace std::literals;

inline std::string ping_node_name(uint id) { return "ping_node"s + std::to_string(id); }
inline std::string pong_node_name(uint id) { return "pong_node"s + std::to_string(id); }

inline std::string ping_topic_name(uint id) { return "ping"s + std::to_string(id); }
inline std::string pong_topic_name(uint id) { return "pong"s + std::to_string(id); }

inline int get_node_counts_from_option(int argc, char *argv[], int when_no_option = 1) {
  for (auto i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--node-counts"s) {
      return std::stoi(argv[++i]);
    }
  }
  return when_no_option;
}

inline std::string get_datetime_utc_now_string(
    std::chrono::system_clock::time_point time_point = std::chrono::system_clock::now()) {
  const auto t = std::chrono::system_clock::to_time_t(time_point);
  const auto tm = std::gmtime(&t);
  char buffer[16];

  // YYYYMMDDHHMMSS
  std::strftime(buffer, sizeof(buffer), "%Y%m%d%H%M%S", tm);
  return std::string(buffer);
}

inline std::filesystem::path create_data_directory() {
  const auto data_directory_path =
      std::filesystem::current_path() / "data"s / get_datetime_utc_now_string();

  std::filesystem::create_directories(data_directory_path);
  return data_directory_path;
}
