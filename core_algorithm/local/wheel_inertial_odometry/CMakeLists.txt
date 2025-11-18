cmake_minimum_required(VERSION 3.8)
project(wheel_inertial_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_BINARY_DIR}/cmake)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
include_directories( EIGEN3_INCLUDE_DIR )
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# add executables
add_executable(local_filter_LPF src/local_filter_LPF.cpp)
ament_target_dependencies(
  local_filter_LPF 
  rclcpp nav_msgs 
  sensor_msgs 
  geometry_msgs 
  Eigen3
  tf2
  tf2_ros
  tf2_geometry_msgs
)


# install executables
install(TARGETS
  local_filter_LPF
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  include
  param
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY 
    launch
    DESTINATION share/${PROJECT_NAME}
)

# install(
#   # TARGETS local_filter_LPF
#   EXPORT export_${PROJECT_NAME}
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
# )

# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# ament_export_dependencies(some_dependency)

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

ament_package()
