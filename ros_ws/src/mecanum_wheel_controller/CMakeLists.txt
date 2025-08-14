cmake_minimum_required(VERSION 3.8)
project(mecanum_wheel_controller)

# Set default compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 20)
# Find required ROS 2 and other packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(std_srvs REQUIRED)
find_package(custom_interfaces REQUIRED)

# Add the executable
add_executable(mecanum_wheel_controller_node 
  src/main.cpp 
  src/mecanum_wheel_controller_node.cpp 
  src/motor_controller.cpp
)

# Include directories
target_include_directories(mecanum_wheel_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Set C++ standard
target_compile_features(mecanum_wheel_controller_node PUBLIC cxx_std_17)

# Link against dependencies
ament_target_dependencies(mecanum_wheel_controller_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  std_srvs
  custom_interfaces
)

target_link_libraries(mecanum_wheel_controller_node
  ${Boost_LIBRARIES}
)

# Install the executable
install(TARGETS mecanum_wheel_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the config file
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Configure linting and testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
