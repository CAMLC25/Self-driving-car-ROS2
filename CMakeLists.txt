cmake_minimum_required(VERSION 3.8)
project(my_self_driving_bot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Tìm các thư viện cần thiết
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)

# 2. Cài đặt các thư mục tài nguyên
install(DIRECTORY
  config
  launch
  worlds
  urdf
  mesh
  DESTINATION share/${PROJECT_NAME}
)

# 3. Cài đặt file Python (Node điều khiển chính)
install(PROGRAMS
  src/line_follower.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()