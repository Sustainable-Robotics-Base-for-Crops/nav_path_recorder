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
  nav_msgs
  GeographicLib
  nlohmann_json
  angles)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
# Geographiclib installs FindGeographicLib.cmake to this non-standard location
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
foreach(dep ${dependencies})
  find_package(${dep} REQUIRED)
endforeach()

include_directories(include)

add_library(${PROJECT_NAME}_lib SHARED
  src/path2d.cpp
  src/least_squares.cpp)
target_link_libraries (${PROJECT_NAME}_lib
  Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME}_lib
  ${dependencies})
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

add_library(${PROJECT_NAME} SHARED
  src/path_recorder.cpp)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies})
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "${PROJECT_NAME}::PathRecorder"
  EXECUTABLE ${PROJECT_NAME}_node)

install(TARGETS
  ${PROJECT_NAME}
  ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
