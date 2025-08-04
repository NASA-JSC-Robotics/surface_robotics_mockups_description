cmake_minimum_required(VERSION 3.16.3)
project(express_rack)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    urdf
    meshes
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
