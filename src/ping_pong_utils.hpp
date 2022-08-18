#include <string>
using namespace std::literals;

inline std::string ping_node_name(int index) { return "ping_node"s + std::to_string(index); }
inline std::string pong_node_name(int index) { return "pong_node"s + std::to_string(index); }

inline std::string ping_topic_name(int index) { return "ping"s + std::to_string(index); }
inline std::string pong_topic_name(int index) { return "pong"s + std::to_string(index); }
