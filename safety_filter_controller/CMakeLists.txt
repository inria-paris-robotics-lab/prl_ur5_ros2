cmake_minimum_required(VERSION 3.16)
project(safety_filter_controller LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Pinocchio ships a CMake config; requires being built WITH_COLLISION_SUPPORT=ON
# (coal + broadphase) for pinocchio/collision/broadphase-manager.hpp to exist.
find_package(pinocchio REQUIRED)
find_package(coal REQUIRED)

add_library(safety_filter_controller SHARED
  src/safety_filter_controller.cpp
)
target_include_directories(safety_filter_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(safety_filter_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  std_msgs
)

# pinocchio::pinocchio / coal::coal are exported as CMake targets by
# their respective configs — link them directly rather than via ament_target_dependencies.
target_link_libraries(safety_filter_controller
  pinocchio::pinocchio
  coal::coal
)

target_compile_definitions(safety_filter_controller PRIVATE
  "SAFETY_FILTER_CONTROLLER_BUILDING_DLL"
)

pluginlib_export_plugin_description_file(controller_interface safety_filter_controller_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS safety_filter_controller
  EXPORT export_safety_filter_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_safety_filter_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  pinocchio
  coal
)
ament_package()
