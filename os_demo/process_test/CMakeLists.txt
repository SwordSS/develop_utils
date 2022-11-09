cmake_minimum_required(VERSION 2.8.3)
project(process_test)

add_compile_options(-std=c++11 -std=c++14
)

include_directories(
  include
)

add_executable(process_test src/process_test.cpp)
add_executable(cmd_test src/cmd_test.cpp)
