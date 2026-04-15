cmake_minimum_required(VERSION 3.8)
project(helper_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
add_executable(twist_to_stamped src/twist_to_stamped.cpp)

ament_target_dependencies(twist_to_stamped
  rclcpp
  geometry_msgs
)

install(TARGETS
  twist_to_stamped
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(scan_frame_fixer src/scan_frame_fixer.cpp)

ament_target_dependencies(scan_frame_fixer
  rclcpp
  sensor_msgs
)

install(TARGETS
  scan_frame_fixer
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

