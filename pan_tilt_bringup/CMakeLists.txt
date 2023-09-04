cmake_minimum_required(VERSION 3.5)
project(pan_tilt_bringup)

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_STANDARD 14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-dev)
endif()

find_package(ament_cmake REQUIRED
)

install(
  DIRECTORY launch DESTINATION share/${PROJECT_NAME}/
)

ament_package()
