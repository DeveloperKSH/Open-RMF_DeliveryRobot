cmake_minimum_required(VERSION 3.8)
project(status_publisher_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Include directories
include_directories(include)

include_directories(
  ${nlohmann_json_INCLUDE_DIRS}
)

# Executables
add_executable(status_publisher_node src/status_publisher_node.cpp)
ament_target_dependencies(status_publisher_node rclcpp sensor_msgs std_msgs tf2_geometry_msgs nav_msgs tf2) 

target_link_libraries(status_publisher_node
  ${nlohmann_json_LIBRARIES}
)

# Install targets
install(TARGETS
  status_publisher_node
  DESTINATION lib/${PROJECT_NAME})
  
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
