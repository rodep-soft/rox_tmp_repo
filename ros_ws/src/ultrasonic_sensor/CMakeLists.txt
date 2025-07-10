cmake_minimum_required(VERSION 3.8)
project(ultrasonic_sensor)

# Note: This package requires libgpiod for GPIO access
# Install with: sudo apt install libgpiod-dev libgpiod2

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Create executable
add_executable(ultrasonic_sensor src/ultrasonic_sensor.cpp)

# Link gpiod library
target_link_libraries(ultrasonic_sensor gpiod)

# Install executable
install(TARGETS ultrasonic_sensor
  DESTINATION lib/${PROJECT_NAME})

# Install header files
#install(DIRECTORY include/
#  DESTINATION include/)

# Install launch files if any
# install(DIRECTORY launch/
#   DESTINATION share/${PROJECT_NAME}/launch/)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
