cmake_minimum_required(VERSION 3.8)
project(plan_to_gps_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(GeographicLib REQUIRED)

include_directories(
  ${Boost_INCLUDE_DIRS}
)

add_executable(plan_to_gps_converter_node src/plan_to_gps_converter_node.cpp)

ament_target_dependencies(plan_to_gps_converter_node
  rclcpp
  nav_msgs
  sensor_msgs
  tf2_geometry_msgs
)

target_link_libraries(plan_to_gps_converter_node 
  ${GeographicLib_LIBRARIES}
)

install(TARGETS
  plan_to_gps_converter_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skip linter checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip cpplint checks
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
