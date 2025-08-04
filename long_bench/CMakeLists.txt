cmake_minimum_required(VERSION 3.16.3)
project(long_bench)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    urdf
  DESTINATION
    share/${PROJECT_NAME}
)

ament_package()
