cmake_minimum_required(VERSION 3.8)
project(vehicle_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(can_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)

include_directories(include)

# Create the sender node executable
add_executable(can_message_sender_node
  src/sender_main.cpp
  src/can_message_sender_node.cpp
)

# Create the receiver node executable
add_executable(can_message_receiver_node
  src/receiver_main.cpp
  src/can_message_receiver_node.cpp
)

# Link dependencies
ament_target_dependencies(can_message_sender_node
  rclcpp
  can_msgs
  std_msgs
)

ament_target_dependencies(can_message_receiver_node
  rclcpp
  can_msgs
  std_msgs
  autoware_vehicle_msgs
)

# Install targets
install(TARGETS
  can_message_sender_node
  can_message_receiver_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
