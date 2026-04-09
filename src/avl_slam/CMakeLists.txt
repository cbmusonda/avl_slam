cmake_minimum_required(VERSION 3.8)
project(avl_slam)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/rtabmap_watchdog.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
