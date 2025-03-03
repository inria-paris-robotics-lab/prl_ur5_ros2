cmake_minimum_required(VERSION 3.8)
project(wsg_50_simulation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch urdf controllers meshes
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()