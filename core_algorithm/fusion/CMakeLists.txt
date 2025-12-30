cmake_minimum_required(VERSION 3.8)
project(fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include Cpp "include" directory
include_directories(include)

# Declare a C++ library
add_library(ekf
 src/ekf.cpp
)

# Create Cpp executable
add_executable(fusion_node src/fusion_node.cpp)
ament_target_dependencies(
  fusion_node
  rclcpp nav_msgs 
  geometry_msgs 
  Eigen3
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_link_libraries(fusion_node
  ekf
)

# Install Cpp executables
install(TARGETS
  fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/ekf_node.py
  scripts/fake_cam.py
  scripts/test_br.py
  DESTINATION lib/${PROJECT_NAME}
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
