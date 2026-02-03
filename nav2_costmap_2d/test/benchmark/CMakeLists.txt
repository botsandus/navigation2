find_package(ament_cmake_google_benchmark REQUIRED)
ament_add_google_benchmark(observation_buffer_benchmark observation_buffer_benchmark.cpp)
target_link_libraries(observation_buffer_benchmark
  nav2_costmap_2d_core
  layers
)

ament_add_google_benchmark(inflation_layer_benchmark inflation_layer_benchmark.cpp)
target_link_libraries(inflation_layer_benchmark
  nav2_costmap_2d_core
  layers
  nav2_ros_common::nav2_ros_common
  rclcpp::rclcpp
  tf2_ros::tf2_ros
  rosbag2_cpp::rosbag2_cpp
  opencv_core
  opencv_imgcodecs
)
