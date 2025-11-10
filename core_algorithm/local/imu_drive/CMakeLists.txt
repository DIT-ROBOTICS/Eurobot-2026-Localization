cmake_minimum_required(VERSION 3.8)
project(imu_drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(imu_drive_lib lib/imu_drive.cpp)
target_include_directories(imu_drive_lib PUBLIC include)
ament_target_dependencies(imu_drive_lib rclcpp std_srvs sensor_msgs geometry_msgs nav_msgs std_msgs)

add_executable(imu_drive_node src/imu_drive_node.cpp)
ament_target_dependencies(imu_drive_node rclcpp std_srvs sensor_msgs geometry_msgs nav_msgs std_msgs)
target_link_libraries(imu_drive_node imu_drive_lib)

install(TARGETS
  imu_drive_lib
  imu_drive_node

  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
    launch
    config
    DESTINATION share/${PROJECT_NAME}
)

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
