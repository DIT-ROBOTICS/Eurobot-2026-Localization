cmake_minimum_required(VERSION 3.8)
project(imm_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(imm_filter_lib lib/EKF.cpp lib/IMM.cpp lib/ModelGenerator.cpp)
target_include_directories(imm_filter_lib PUBLIC include)
ament_target_dependencies(imm_filter_lib rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs tf2 tf2_ros)

add_executable(imm_filter_node src/imm_filter_node.cpp)
ament_target_dependencies(imm_filter_node rclcpp sensor_msgs geometry_msgs nav_msgs std_msgs tf2 tf2_ros)
target_link_libraries(imm_filter_node imm_filter_lib)

install(TARGETS
  imm_filter_lib
  imm_filter_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
    launch
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
