cmake_minimum_required(VERSION 3.8)
project(nav_path_recorder)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(dependencies
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  nav_util
  nav_msgs)

find_package(ament_cmake REQUIRED)
foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/path_recorder.cpp)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies})
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "${PROJECT_NAME}::PathRecorder"
  EXECUTABLE ${PROJECT_NAME}_node)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
