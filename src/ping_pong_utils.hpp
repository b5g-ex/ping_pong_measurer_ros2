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
