cmake_minimum_required(VERSION 3.10)
project(mid360_agras CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Threads REQUIRED)

add_library(mid360_agras STATIC mid360_agras.cpp)
target_include_directories(mid360_agras PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(mid360_agras PUBLIC Threads::Threads)
target_compile_options(mid360_agras PRIVATE -Wall -Wextra)

add_executable(mid360_agras_demo example_main.cpp)
target_link_libraries(mid360_agras_demo PRIVATE mid360_agras)
