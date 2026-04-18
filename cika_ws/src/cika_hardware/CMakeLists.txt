cmake_minimum_required(VERSION 3.8)
project(cika_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(cika_drive_interface SHARED
  src/cika_drive_interface.cpp
)

target_include_directories(cika_drive_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(cika_drive_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  nav_msgs
  std_msgs
  geometry_msgs
)

pluginlib_export_plugin_description_file(hardware_interface cika_hardware_plugins.xml)

install(TARGETS cika_drive_interface
  EXPORT export_cika_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES cika_hardware_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(cika_drive_interface)
ament_export_targets(export_cika_hardware)

ament_package()