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
find_package(nav_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

# Add the executable
add_executable(joy_driver_node src/main.cpp src/joy_driver_node.cpp)

# Include directories
target_include_directories(joy_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Set C++ standard
target_compile_features(joy_driver_node PUBLIC cxx_std_17)

# Link against dependencies
ament_target_dependencies(joy_driver_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  custom_interfaces
  std_srvs
)

# Install the executable
install(TARGETS joy_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include headers
install(DIRECTORY include/
  DESTINATION include
)

# Configure linting and testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # Disable specific linting tests and use clang-format instead
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)  # Disable uncrustify to use clang-format
  
  ament_lint_auto_find_test_dependencies()

  # Add unit tests
  ament_add_gtest(${PROJECT_NAME}_test
    test/test_joy_driver.cpp
    src/joy_driver_node.cpp  # テスト対象のソースファイル
  )

  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  ament_target_dependencies(${PROJECT_NAME}_test
    rclcpp
    sensor_msgs
    std_msgs
    geometry_msgs
    custom_interfaces
    std_srvs
  )
endif()

ament_package()
