cmake_minimum_required(VERSION 3.8)
project(mid360_agras_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(Threads REQUIRED)

# the standalone driver is linux/ at the top of this package tree (mid360_agras_driver/linux)
set(MID360_DRIVER_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../linux" CACHE PATH "directory with mid360_agras.h/.cpp")
if(NOT EXISTS "${MID360_DRIVER_DIR}/mid360_agras.cpp")
  message(FATAL_ERROR "mid360_agras.cpp not found in ${MID360_DRIVER_DIR} (set -DMID360_DRIVER_DIR=...)")
endif()

add_executable(mid360_agras_node src/mid360_agras_node.cpp ${MID360_DRIVER_DIR}/mid360_agras.cpp)
target_include_directories(mid360_agras_node PRIVATE ${MID360_DRIVER_DIR})
target_compile_options(mid360_agras_node PRIVATE -Wall -Wextra)
ament_target_dependencies(mid360_agras_node rclcpp sensor_msgs livox_ros_driver2)
target_link_libraries(mid360_agras_node Threads::Threads)

install(TARGETS mid360_agras_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})

ament_package()
