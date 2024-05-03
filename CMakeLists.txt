cmake_minimum_required(VERSION 3.8)
project(imu_serial)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(
  imu_serial
  src/imu_serial_node.cpp
  src/crc8_crc16.cpp
)

target_link_libraries(
  imu_serial
  serial
)

ament_target_dependencies(
  imu_serial
  rclcpp
  sensor_msgs
)

install(TARGETS
  imu_serial
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
