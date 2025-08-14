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

# Create a library from joy_driver_node.cpp
add_library(joy_driver_lib src/joy_driver_node.cpp)
target_include_directories(joy_driver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(joy_driver_lib
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  custom_interfaces
  std_srvs
)

# Executable that uses the library
add_executable(joy_driver_node src/main.cpp)
target_link_libraries(joy_driver_node joy_driver_lib)
target_include_directories(joy_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(joy_driver_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  custom_interfaces
  std_srvs
)
target_compile_features(joy_driver_node PUBLIC cxx_std_17)

# Install the executable and headers
install(TARGETS joy_driver_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

# Configure linting and testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # Disable some lint checks
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()

  # Add unit test linking to the library
  ament_add_gtest(${PROJECT_NAME}_test
    test/test_joy_driver.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test joy_driver_lib)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test
    rclcpp
    sensor_msgs
    std_msgs
    geometry_msgs
    nav_msgs
    custom_interfaces
    std_srvs
  )
endif()

ament_package()
