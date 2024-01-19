cmake_minimum_required(VERSION 3.5)
project(hatch_4040)

find_package(ament_cmake REQUIRED)

install(
DIRECTORY launch meshes rviz urdf
DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

install(PROGRAMS scripts/hatch4040_simple_publisher.py scripts/hatch4040_manager.py scripts/mockup_state_manager.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
