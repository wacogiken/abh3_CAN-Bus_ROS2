cmake_minimum_required(VERSION 3.5)
project(abh3_can_topic)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-function)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(abh3_can_interface REQUIRED)

set(CAN_NODES ros2ABH3Brd; ros2ABH3Cmd)
foreach(target IN LISTS CAN_NODES)
  #add_executable(${target} src/${target}.cpp src/canABH3++.cpp src/canABH3.c)
  add_executable(${target} src/${target}.cpp src/canABH3++ src/canABH3)
  #ament_target_dependencies(${target} rclcpp std_msgs abh3_can_interface)
  ament_target_dependencies(${target} rclcpp abh3_can_interface)
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endforeach()

set(OTHER_NODES ros2ABH3Cnv; testBRD; testCMD)
foreach(target IN LISTS OTHER_NODES)
  add_executable(${target} src/${target}.cpp)
  ament_target_dependencies(${target} rclcpp std_msgs geometry_msgs sensor_msgs abh3_can_interface)
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endforeach()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
