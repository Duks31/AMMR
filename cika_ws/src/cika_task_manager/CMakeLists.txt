cmake_minimum_required(VERSION 3.8)
project(cika_task_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# ── Action interface ──────────────────────────────────────────────────────────
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PickAndDispose.action"
  DEPENDENCIES geometry_msgs
)

# ── Python package ────────────────────────────────────────────────────────────
ament_python_install_package(cika_task_manager_core)

install(PROGRAMS
  cika_task_manager_core/task_manager_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  action
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()