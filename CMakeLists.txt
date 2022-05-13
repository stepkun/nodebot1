cmake_minimum_required(VERSION 3.5)
project(nodebot1)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Include C++ "include" directory
include_directories(include)

# Create C++ executable
add_executable(control_cpp src/control.cpp)
ament_target_dependencies(control_cpp rclcpp geometry_msgs std_msgs)

# Install C++ executables
install(
  TARGETS control_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(nodebot1)

# Install Python executables as scripts
install(PROGRAMS
  scripts/control_py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files ets
install(
  DIRECTORY config description launch worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
