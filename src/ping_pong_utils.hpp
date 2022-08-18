#include <string>
using namespace std::literals;

inline std::string ping_node_name(int index) { return "ping_node"s + std::to_string(index); }
inline std::string pong_node_name(int index) { return "pong_node"s + std::to_string(index); }

inline std::string ping_topic_name(int index) { return "ping"s + std::to_string(index); }
inline std::string pong_topic_name(int index) { return "pong"s + std::to_string(index); }

inline int get_node_counts_from_option(int argc, char *argv[], int when_no_option = 1) {
  for (auto i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--node-counts"s) {
      return std::stoi(argv[++i]);
    }
  }
  return when_no_option;
}
