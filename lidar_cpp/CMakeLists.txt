cmake_minimum_required(VERSION 3.20)
project(lidar_cpp)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find all required ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED) # Needed for math headers

# 2. Define the executable from your source file
add_executable(tilt_broadcaster src/tilt_broadcaster.cpp)

# 3. Handle include paths and linking
# We use this macro INSTEAD of target_link_libraries to fix the "not found" error
ament_target_dependencies(tilt_broadcaster
  rclcpp
  sensor_msgs
  tf2_ros
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

# 4. Install the executable so 'ros2 run' can find it
install(TARGETS
  tilt_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

# 5. Handle testing dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 6. Finalize the ament package
ament_package()