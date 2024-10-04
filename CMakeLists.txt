cmake_minimum_required(VERSION 3.5)
project(clr_trainer_hatch_offsets)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
