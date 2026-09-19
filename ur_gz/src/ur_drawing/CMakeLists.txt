cmake_minimum_required(VERSION 3.5)
project(ur_drawing)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/draw_p.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
