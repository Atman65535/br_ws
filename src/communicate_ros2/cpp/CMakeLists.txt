cmake_minimum_required(VERSION 3.8)
project(communicate_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_interfaces REQUIRED)

# serial 库（libserial-dev 提供 <serial/serial.h> 和 libserial.so）
find_library(SERIAL_LIBRARY NAMES serial)

add_executable(communicate_node src/communicate_node.cpp)
target_include_directories(communicate_node PUBLIC include)
target_link_libraries(communicate_node ${SERIAL_LIBRARY})
ament_target_dependencies(communicate_node rclcpp robot_interfaces)

install(TARGETS communicate_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
