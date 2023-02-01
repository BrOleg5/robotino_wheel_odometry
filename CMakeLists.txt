cmake_minimum_required(VERSION 3.8)

include("cmake/read_package_version.cmake")
read_package_version_from_xml()

project(robotino_wheel_odometry VERSION ${PACKAGE_VERSION})

unset(PACKAGE_VERSION)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robotino2 REQUIRED)

add_library(wheel_odometry src/WheelOdometry.cpp)

set_target_properties(wheel_odometry PROPERTIES
  VERSION ${PROJECT_VERSION}
  PUBLIC_HEADER "include/${PROJECT_NAME}/WheelOdometry.hpp"
  DEBUG_POSTFIX d
)

target_include_directories(wheel_odometry PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}
)

ament_target_dependencies(wheel_odometry rclcpp geometry_msgs tf2 tf2_ros nav_msgs sensor_msgs robotino2)


add_executable(wheel_odometry_node src/wheel_odometry.cpp)

# source: https://forum.kde.org/viewtopic.php?f=74&t=153982
if(MSVC AND ${MSVC_VERSION} GREATER_EQUAL 1915)
  # You must acknowledge that you understand MSVC resolved a byte alignment issue in this compiler
  # We get this due to using Eigen objects and allocating those objects with make_shared
  target_compile_definitions(wheel_odometry_node PRIVATE _ENABLE_EXTENDED_ALIGNED_STORAGE)
endif()

target_link_libraries(wheel_odometry_node wheel_odometry)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  wheel_odometry
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  PUBLIC_HEADER DESTINATION include/${PROJECT_NAME}
)

install(TARGETS
  wheel_odometry_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories("include")

ament_package()
