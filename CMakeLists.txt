cmake_minimum_required(VERSION 3.8)
project(obs_avoid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  set(ament_cmake_copyright_FOUND TRUE)
 
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


add_executable(user_ctrl src/offboard_node_user_ctrl.cpp)
ament_target_dependencies(user_ctrl rclcpp mavros_msgs geometry_msgs )
install(TARGETS user_ctrl DESTINATION lib/${PROJECT_NAME})

add_executable(spiral_mapping_mode src/spiral_mapping_mode.cpp)
ament_target_dependencies(spiral_mapping_mode rclcpp mavros_msgs geometry_msgs)
install(TARGETS spiral_mapping_mode DESTINATION lib/${PROJECT_NAME})


add_executable(local_planner_mode_a src/local_planner_mode_a.cpp)
ament_target_dependencies(local_planner_mode_a rclcpp mavros_msgs geometry_msgs sensor_msgs nav_msgs tf2)
install(TARGETS local_planner_mode_a DESTINATION lib/${PROJECT_NAME})

add_executable(local_planner_sector_mode src/local_planner_sector_mode.cpp)
ament_target_dependencies(local_planner_sector_mode rclcpp mavros_msgs geometry_msgs sensor_msgs nav_msgs tf2)
install(TARGETS local_planner_sector_mode DESTINATION lib/${PROJECT_NAME})

add_executable(local_planner_hybrid_mode src/local_planner_hybrid_mode.cpp)
ament_target_dependencies(local_planner_hybrid_mode rclcpp mavros_msgs geometry_msgs sensor_msgs nav_msgs tf2)
install(TARGETS local_planner_hybrid_mode DESTINATION lib/${PROJECT_NAME})

ament_package()
