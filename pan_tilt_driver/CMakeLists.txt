cmake_minimum_required(VERSION 3.5)
project(pan_tilt_driver)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pan_tilt_msgs REQUIRED)


find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  include
)

set(serial_SRCS
    src/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h
)
list(APPEND serial_SRCS src/impl/unix.cc)
list(APPEND serial_SRCS src/impl/list_ports/list_ports_linux.cc)

add_executable(PanTiltDriverNode
  src/PanTiltDriverNode.cpp
  src/PanTiltDriver.cpp
  src/QThread.cpp
  src/modbus_rtu_master.cpp
  ${serial_SRCS}
)  

ament_target_dependencies(PanTiltDriverNode rclcpp std_msgs tf2 tf2_ros pan_tilt_msgs)

install(TARGETS  
  PanTiltDriverNode  
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  DESTINATION share/${PROJECT_NAME}/)
  
ament_export_include_directories(
  include
)

ament_package()

