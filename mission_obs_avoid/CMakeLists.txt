cmake_minimum_required(VERSION 3.8)
project(mission_obs_avoid)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

add_executable(mission_interceptor_node src/mission_interceptor_node.cpp)
ament_target_dependencies(mission_interceptor_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  diagnostic_msgs
  mavros_msgs
)

add_executable(cmd_vel_arbiter_node src/cmd_vel_arbiter_node.cpp)
ament_target_dependencies(cmd_vel_arbiter_node
  rclcpp
  geometry_msgs
  std_msgs
)

install(TARGETS
  mission_interceptor_node
  cmd_vel_arbiter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/start_mission_obs_avoid.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
