cmake_minimum_required(VERSION 3.22)
project(arctos_gazebo)

find_package(ament_cmake REQUIRED)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(gate_a test/test_gate_a.py)
endif()

ament_package()
