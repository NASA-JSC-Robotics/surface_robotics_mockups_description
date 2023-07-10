cmake_minimum_required(VERSION 3.5)
project(trainer)

find_package(ament_cmake REQUIRED)

install(
DIRECTORY launch meshes rviz urdf
DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

install(PROGRAMS scripts/trainer_simple_publisher.py scripts/trainer_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
