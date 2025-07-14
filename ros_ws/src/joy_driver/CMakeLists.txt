cmake_minimum_required(VERSION 3.8)
project(joy_driver)

# Set default compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(std_srvs REQUIRED)


# Add the executable
add_executable(joy_driver_node src/joy_driver_node.cpp)

# Set C++ standard
target_compile_features(joy_driver_node PUBLIC cxx_std_17)

# Link against dependencies
ament_target_dependencies(joy_driver_node
  rclcpp
  sensor_msgs
  geometry_msgs
  custom_interfaces
  std_srvs
)

# Install the executable
install(TARGETS joy_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# Configure linting and testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
