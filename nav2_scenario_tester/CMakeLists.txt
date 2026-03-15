cmake_minimum_required(VERSION 3.8)
project(nav2_scenario_tester)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# ── C++ RViz plugin library ────────────────────────────────────────────
set(rviz_lib nav2_scenario_tester_rviz)

add_library(${rviz_lib} SHARED
  src/nav_test_designer_panel.cpp
  src/start_pose_tool.cpp
  src/goal_pose_tool.cpp
  src/obstacle_tool.cpp
  include/nav2_scenario_tester/nav_test_designer_panel.hpp
  include/nav2_scenario_tester/start_pose_tool.hpp
  include/nav2_scenario_tester/goal_pose_tool.hpp
  include/nav2_scenario_tester/obstacle_tool.hpp
)

target_include_directories(${rviz_lib} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${rviz_lib} PUBLIC
  rclcpp::rclcpp
  rviz_common::rviz_common
  rviz_default_plugins::rviz_default_plugins
  rviz_rendering::rviz_rendering
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  tf2::tf2
  Qt5::Widgets
  yaml-cpp::yaml-cpp
)

install(
  TARGETS ${rviz_lib}
  EXPORT ${rviz_lib}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

# ── Python module ──────────────────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME}
  SCRIPTS_DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, config, maps, and rviz configs
install(DIRECTORY
  launch config maps rviz
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

ament_export_targets(${rviz_lib} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  geometry_msgs
  pluginlib
  rclcpp
  rviz_common
  rviz_default_plugins
  rviz_rendering
  tf2
  visualization_msgs
)

ament_package(
  CONFIG_EXTRAS "nav2_scenario_tester-extras.cmake"
)
