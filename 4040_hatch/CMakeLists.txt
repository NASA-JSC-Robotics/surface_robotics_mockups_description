cmake_minimum_required(VERSION 3.5)
project(4040_hatch)

find_package(ament_cmake REQUIRED)

  install(
    DIRECTORY launch rviz urdf
    DESTINATION share/${PROJECT_NAME}
  )

  ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

  ament_package()
