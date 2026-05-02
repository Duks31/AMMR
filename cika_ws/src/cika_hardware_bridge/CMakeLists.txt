cmake_minimum_required(VERSION 3.22)
project(cika_hardware_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)

add_executable(unpacker_node src/unpacker_node.cpp)
target_include_directories(unpacker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(unpacker_node PUBLIC c_std_99 cxx_std_17)  # Require C++17

ament_target_dependencies(
  unpacker_node
  "rclcpp"
  "sensor_msgs"
  "control_msgs"
)

install(TARGETS unpacker_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()