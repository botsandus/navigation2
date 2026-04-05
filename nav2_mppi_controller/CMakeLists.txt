cmake_minimum_required(VERSION 3.8)
project(nav2_mppi_controller)

find_package(ament_cmake REQUIRED)
find_package(angles REQUIRED)
find_package(backward_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(Eigen3 REQUIRED)

# ── Optional GPU acceleration ──────────────────────────────────────────
option(NAV2_CUDA_SUPPORT "Enable CUDA GPU acceleration for MPPI" OFF)
if(NAV2_CUDA_SUPPORT)
  include(CheckLanguage)
  check_language(CUDA)
  if(CMAKE_CUDA_COMPILER)
    cmake_policy(SET CMP0104 NEW)
    if(NOT CMAKE_CUDA_ARCHITECTURES)
      set(CMAKE_CUDA_ARCHITECTURES native)
    endif()
    enable_language(CUDA)
    find_package(CUDAToolkit REQUIRED)
    message(STATUS "CUDA support enabled for MPPI controller (${CMAKE_CUDA_COMPILER})")
  else()
    message(WARNING "NAV2_CUDA_SUPPORT requested but no CUDA compiler found. Disabling.")
    set(NAV2_CUDA_SUPPORT OFF)
  endif()
endif()

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

nav2_package()

# When CUDA is enabled, wrap directory-level compile options in generator
# expressions so GCC/Clang flags (e.g. -Wdeprecated) are not passed to nvcc.
if(NAV2_CUDA_SUPPORT AND CMAKE_CUDA_COMPILER)
  get_directory_property(_dir_compile_opts COMPILE_OPTIONS)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "")
  foreach(_opt IN LISTS _dir_compile_opts)
    add_compile_options("$<$<COMPILE_LANGUAGE:C,CXX>:${_opt}>")
  endforeach()
endif()

include(CheckCXXCompilerFlag)

check_cxx_compiler_flag("-mfma" COMPILER_SUPPORTS_FMA)

if(COMPILER_SUPPORTS_FMA)
  add_compile_options("$<$<COMPILE_LANGUAGE:C,CXX>:-mfma>")
endif()

# If building one the same hardware to be deployed on, try `-march=native`!

# ── Optional CUDA kernels library ──────────────────────────────────────
if(NAV2_CUDA_SUPPORT)
  add_library(mppi_cuda SHARED
    src/cuda/mppi_kernels.cu
  )
  set_target_properties(mppi_cuda PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_STANDARD 17
    POSITION_INDEPENDENT_CODE ON
  )
  target_include_directories(mppi_cuda
    PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
  target_link_libraries(mppi_cuda PUBLIC
    CUDA::cudart
  )
endif()

add_library(mppi_controller SHARED
  src/controller.cpp
  src/critic_manager.cpp
  src/noise_generator.cpp
  src/optimizer.cpp
  src/parameters_handler.cpp
  src/trajectory_visualizer.cpp
)
target_compile_options(mppi_controller PUBLIC -O3)
target_include_directories(mppi_controller
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(mppi_controller PUBLIC
  angles::angles
  ${geometry_msgs_TARGETS}
  nav2_core::nav2_core
  nav2_ros_common::nav2_ros_common
  nav2_costmap_2d::layers
  nav2_costmap_2d::nav2_costmap_2d_core
  ${nav2_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_msgs_TARGETS}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  ${visualization_msgs_TARGETS}
)

if(NAV2_CUDA_SUPPORT)
  target_compile_definitions(mppi_controller PUBLIC NAV2_CUDA_SUPPORT)
  target_link_libraries(mppi_controller PUBLIC mppi_cuda)
endif()

add_library(mppi_critics SHARED
  src/critics/constraint_critic.cpp
  src/critics/cost_critic.cpp
  src/critics/goal_critic.cpp
  src/critics/goal_angle_critic.cpp
  src/critics/obstacles_critic.cpp
  src/critics/path_align_critic.cpp
  src/critics/path_angle_critic.cpp
  src/critics/path_follow_critic.cpp
  src/critics/prefer_forward_critic.cpp
  src/critics/twirling_critic.cpp
  src/critics/velocity_deadband_critic.cpp
)
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND APPLE)
    # Apple Clang: use C++20 and optimization, omit -fconcepts
    target_compile_features(mppi_critics PUBLIC cxx_std_20)
    target_compile_options(mppi_critics PUBLIC -O3)
else()
    target_compile_options(mppi_critics PUBLIC -fconcepts -O3)
endif()
target_include_directories(mppi_critics
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(mppi_critics PUBLIC
  angles::angles
  ${geometry_msgs_TARGETS}
  nav2_core::nav2_core
  nav2_ros_common::nav2_ros_common
  nav2_costmap_2d::layers
  nav2_costmap_2d::nav2_costmap_2d_core
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_msgs_TARGETS}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  ${visualization_msgs_TARGETS}
)
target_link_libraries(mppi_critics PRIVATE
  pluginlib::pluginlib
)

if(NAV2_CUDA_SUPPORT)
  target_compile_definitions(mppi_critics PUBLIC NAV2_CUDA_SUPPORT)
endif()

add_library(mppi_trajectory_validators SHARED
  src/trajectory_validators/optimal_trajectory_validator.cpp
)
target_compile_options(mppi_trajectory_validators PUBLIC -O3)
target_include_directories(mppi_trajectory_validators
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
    "$<BUILD_INTERFACE:${nav2_ros_common_INCLUDE_DIRS}>")
target_link_libraries(mppi_trajectory_validators PUBLIC
  angles::angles
  ${geometry_msgs_TARGETS}
  nav2_core::nav2_core
  nav2_costmap_2d::layers
  nav2_costmap_2d::nav2_costmap_2d_core
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_msgs_TARGETS}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros
  ${visualization_msgs_TARGETS}
)
target_link_libraries(mppi_trajectory_validators PRIVATE
  pluginlib::pluginlib
)

install(TARGETS mppi_controller mppi_critics mppi_trajectory_validators
  EXPORT nav2_mppi_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(NAV2_CUDA_SUPPORT)
  install(TARGETS mppi_cuda
    EXPORT nav2_mppi_controller
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
endif()

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_find_gtest()

  add_subdirectory(test)
  add_subdirectory(benchmark)
endif()

ament_export_libraries(${libraries})
ament_export_dependencies(
  angles
  geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  nav2_ros_common
  Eigen3
)
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(nav2_mppi_controller)

pluginlib_export_plugin_description_file(nav2_core mppic.xml)
pluginlib_export_plugin_description_file(nav2_mppi_controller critics.xml)
pluginlib_export_plugin_description_file(nav2_mppi_controller trajectory_validators.xml)

ament_package()
