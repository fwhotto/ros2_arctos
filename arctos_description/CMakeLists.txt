cmake_minimum_required(VERSION 3.5)
project(arctos_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes rviz launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(model test/test_model.py TIMEOUT 120)
endif()

ament_package()
