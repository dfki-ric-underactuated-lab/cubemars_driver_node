cmake_minimum_required(VERSION 3.8)
project(cubemars_hardware_interface)


set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_COMPILE_WARNING_AS_ERROR ON)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_srvs
)
find_package(robot_control_msgs REQUIRED)


# # Specify the required version of ros2_control
# find_package(controller_manager 4.0.0)
# # Handle the case where the required version is not found
# if(NOT controller_manager_FOUND)
#   message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
#   "Are you using the correct branch of the repository?")
# endif()

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  src/cubemars_hardware_interface.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(
  ${PROJECT_NAME} PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface cubemars_hardware_interface.xml)

install(
  DIRECTORY ros2_control
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)


# add node
add_executable(cubemars_hardware_node
  src/cubemars_can.cpp
  src/mab_fd_can.cpp
  src/cubemars_hardware_node.cpp
)

set_property(TARGET cubemars_hardware_node PROPERTY CXX_STANDARD 23)
target_compile_options(cubemars_hardware_node PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_include_directories(cubemars_hardware_node PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(
  cubemars_hardware_node PUBLIC
  rclcpp
  rclcpp_lifecycle
  robot_control_msgs
  sensor_msgs
  std_srvs
)

install(TARGETS ${PROJECT_NAME} cubemars_hardware_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/motor_sine_tester.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
