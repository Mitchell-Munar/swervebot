cmake_minimum_required(VERSION 3.20)
project(swervebot_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_simulation.py TIMEOUT 150)
  add_launch_test(test/test_lidar.py TIMEOUT 90)
  # Both integration tests use the same Gazebo world and ROS topic names.
  set_tests_properties(test_test_simulation.py test_test_lidar.py PROPERTIES RUN_SERIAL TRUE)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_description test/test_description.py)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
