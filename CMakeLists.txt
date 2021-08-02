cmake_minimum_required(VERSION 3.8)
project(ros2_sample_publishers)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

add_executable(path_pub src/path_publisher.cpp)
ament_target_dependencies(path_pub geometry_msgs nav_msgs rclcpp std_msgs tf2_msgs)

add_executable(point_cloud_pub src/point_cloud_publisher.cpp)
ament_target_dependencies(point_cloud_pub rclcpp sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  path_pub
  point_cloud_pub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
