cmake_minimum_required(VERSION 3.22)
project(prl_ur5_moveit)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(wait_for_joint_states src/wait_for_joint_states.cpp)
ament_target_dependencies(wait_for_joint_states rclcpp sensor_msgs)

install(TARGETS wait_for_joint_states DESTINATION lib/${PROJECT_NAME})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
    install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
