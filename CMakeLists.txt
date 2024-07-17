cmake_minimum_required(VERSION 3.8)
project(combine_lidar)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

SET(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++20")
# Default to C++20

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Find dependencies for C++ and ROS
find_package(rclcpp REQUIRED)

# Find dependencies for ROS messages format
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# external from ROS
find_package(Eigen3 REQUIRED)

include_directories(
        include
        ${EIGEN3_INCLUDE_DIR}
)

# Add executable the generate | name must be the same as the launch file
add_executable(combine_lidar_node src/combine_livox.cpp src/math_utils.cpp)
# Link dependencies to the executable | name must be the same as the launch file
ament_target_dependencies(combine_lidar_node rclcpp std_msgs sensor_msgs)

# install the executable
install(TARGETS
combine_lidar_node
  DESTINATION lib/${PROJECT_NAME})
# install the launch executable
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)


ament_export_include_directories(include)
ament_package()
