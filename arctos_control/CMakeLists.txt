cmake_minimum_required(VERSION 3.22)
project(arctos_control)

find_package(ament_cmake REQUIRED)

install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config urdf DESTINATION share/${PROJECT_NAME})

ament_package()
