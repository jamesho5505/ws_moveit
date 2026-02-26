cmake_minimum_required(VERSION 3.8)
project(ft_compensation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tm_msgs REQUIRED)


add_executable(ft_compensation_node src/ft_compensation_node.cpp)
ament_target_dependencies(ft_compensation_node
  rclcpp
  geometry_msgs
  tf2_geometry_msgs
  tf2
)

add_executable(tm_ft_compensation_node src/tm_ft_compensation_node.cpp)
ament_target_dependencies(tm_ft_compensation_node
  rclcpp
  geometry_msgs
  tf2_geometry_msgs
  tf2
  tm_msgs
)

install(TARGETS
  ft_compensation_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  tm_ft_compensation_node
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
