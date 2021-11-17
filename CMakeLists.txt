cmake_minimum_required(VERSION 3.5)
project(shipcondev_gyro_jg35fd)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hardware_communication_lib REQUIRED)
find_package(Boost REQUIRED COMPONENTS regex )

# Service Generation
set(srv_files
  "srv/CalibrateGyro.srv"
  "srv/ControlCalculate.srv"
  "srv/ControlOutput.srv"
  "srv/ResetAngle.srv"
  "srv/SetAnalogRange.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  DEPENDENCIES std_msgs
)


add_executable(node
  src/driver.cc
  src/node.cc
)
ament_target_dependencies(node
  rclcpp
  hardware_communication_lib
  Boost
)
target_include_directories(node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
  TARGETS node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
