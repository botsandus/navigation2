cmake_minimum_required(VERSION 3.8)
project(nav2_navigation_test)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python module (SCRIPTS_DESTINATION enables console_scripts from setup.cfg)
ament_python_install_package(${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, config, and maps
install(DIRECTORY
  launch config maps
  DESTINATION share/${PROJECT_NAME}/
)

# Install cmake helpers
install(DIRECTORY
  cmake
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  add_launch_test(
    test/example_loopback_test.py
    TIMEOUT 120
  )
  add_launch_test(
    test/example_parameterized_test.py
    TIMEOUT 180
  )
endif()

ament_package(
  CONFIG_EXTRAS "nav2_navigation_test-extras.cmake"
)
