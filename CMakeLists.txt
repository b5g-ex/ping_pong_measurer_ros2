cmake_minimum_required(VERSION 3.5)
project(ping_pong_measurer)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

add_executable(ping src/ping.cpp)
target_compile_features(ping PUBLIC cxx_std_17)
ament_target_dependencies(ping rclcpp std_msgs)

add_executable(pong src/pong.cpp)
target_compile_features(pong PUBLIC cxx_std_17)
ament_target_dependencies(pong rclcpp std_msgs)

add_executable(starter src/starter.cpp)
target_compile_features(starter PUBLIC cxx_std_17)
ament_target_dependencies(starter rclcpp std_msgs)

install(TARGETS
        ping
        pong
        starter
        talker
        listener
        DESTINATION lib/${PROJECT_NAME})

ament_package()
