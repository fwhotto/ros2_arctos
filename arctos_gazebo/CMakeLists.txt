cmake_minimum_required(VERSION 3.22)
project(arctos_gazebo)

find_package(ament_cmake REQUIRED)

install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch urdf worlds config models DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/planning_ground.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/scene_objects.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/scene_objects.py DESTINATION share/${PROJECT_NAME}/scripts)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(gate_a test/test_gate_a.py)
  ament_add_pytest_test(gate_b test/test_gate_b.py TIMEOUT 120)
  ament_add_pytest_test(gripper test/test_gripper.py TIMEOUT 120)
  ament_add_pytest_test(gazebo_model test/test_gazebo_model.py TIMEOUT 120)
  ament_add_pytest_test(scene test/test_scene.py TIMEOUT 120)
  ament_add_pytest_test(voron_model test/test_voron_model.py TIMEOUT 120)
endif()

ament_package()
