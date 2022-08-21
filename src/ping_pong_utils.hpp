#include <chrono>
#include <filesystem>
#include <string>
using namespace std::literals;

using ppm_options = std::tuple<uint, uint, uint>;

inline std::string ping_node_name(uint id) { return "ping_node"s + std::to_string(id); }
inline std::string pong_node_name(uint id) { return "pong_node"s + std::to_string(id); }

inline std::string ping_topic_name(uint id) { return "ping"s + std::to_string(id); }
inline std::string pong_topic_name(uint id) { return "pong"s + std::to_string(id); }

inline ppm_options get_options(int argc, char *argv[]) {
  uint node_counts = 1;
  uint payload_bytes = 10;
  uint measurement_times = 100;

  for (auto i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--node-counts"s) {
      node_counts = std::stoi(argv[++i]);
    } else if (std::string(argv[i]) == "--payload-bytes"s) {
      payload_bytes = std::stoi(argv[++i]);
    } else if (std::string(argv[i]) == "--measurement-times"s) {
      measurement_times = std::stoi(argv[++i]);
    }
  }

  return {node_counts, payload_bytes, measurement_times};
}

inline uint get_node_counts(ppm_options options) { return std::get<0>(options); }
inline uint get_payload_bytes(ppm_options options) { return std::get<1>(options); }
inline uint get_measurement_times(ppm_options options) { return std::get<2>(options); }

inline std::string get_datetime_utc_now_string(
    std::chrono::system_clock::time_point time_point = std::chrono::system_clock::now()) {
  const auto t = std::chrono::system_clock::to_time_t(time_point);
  const auto tm = std::gmtime(&t);
  char buffer[16];

  // YYYYMMDDHHMMSS
  std::strftime(buffer, sizeof(buffer), "%Y%m%d%H%M%S", tm);
  return std::string(buffer);
}

inline std::string create_data_directory_name(ppm_options options) {
  const auto [node_counts, payload_bytes, measurement_times] = options;
  const auto nc = std::to_string(node_counts);
  const auto pb = std::to_string(payload_bytes);
  const auto mt = std::to_string(measurement_times);

  return get_datetime_utc_now_string() + "_"s + "nc"s + nc + "_"s + "pb"s + pb + "_"s + "mt"s + mt;
}

inline std::filesystem::path create_data_directory(std::string directory_name) {
  const auto data_directory_path = std::filesystem::current_path() / "data"s / directory_name;

  std::filesystem::create_directories(data_directory_path);
  return data_directory_path;
}
