cmake_minimum_required(VERSION 3.5)
project(teleop_twist_controller2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python packages
ament_python_install_package(${PROJECT_NAME})

# Install Python executable script
install(PROGRAMS
  scripts/teleop_twist_controller2.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_package()
