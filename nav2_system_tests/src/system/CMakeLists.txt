nav2_add_test(test_bt_navigator
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_system_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
  TEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}
  BT_NAVIGATOR_XML=navigate_to_pose_w_replanning_and_recovery.xml
  TESTER=nav_to_pose_tester_node.py
  ASTAR=True
  CONTROLLER=nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
  PLANNER=nav2_navfn_planner::NavfnPlanner
)

nav2_add_test(test_bt_navigator_with_wrong_init_pose
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_wrong_init_pose_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
  TEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}
  BT_NAVIGATOR_XML=navigate_to_pose_w_replanning_and_recovery.xml
  TESTER=nav_to_pose_tester_node.py
  ASTAR=True
  CONTROLLER=nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
  PLANNER=nav2_navfn_planner::NavfnPlanner
)

nav2_add_test(test_bt_navigator_with_dijkstra
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_system_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
  TEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}
  BT_NAVIGATOR_XML=navigate_to_pose_w_replanning_and_recovery.xml
  TESTER=nav_to_pose_tester_node.py
  ASTAR=False
  CONTROLLER=dwb_core::DWBLocalPlanner
  PLANNER=nav2_navfn_planner::NavfnPlanner
)

nav2_add_test(test_bt_navigator_with_groot_monitoring
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_system_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
  TEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}
  BT_NAVIGATOR_XML=navigate_to_pose_w_replanning_and_recovery.xml
  TESTER=nav_to_pose_tester_node.py
  ASTAR=False
  GROOT_MONITORING=True
  CONTROLLER=dwb_core::DWBLocalPlanner
  PLANNER=nav2_navfn_planner::NavfnPlanner
)

nav2_add_test(test_bt_navigator_with_legacy_inflation
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_system_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
  TEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}
  BT_NAVIGATOR_XML=navigate_to_pose_w_replanning_and_recovery.xml
  TESTER=nav_to_pose_tester_node.py
  ASTAR=False
  INFLATION_LAYER=nav2_costmap_2d::LegacyInflationLayer
  CONTROLLER=dwb_core::DWBLocalPlanner
  PLANNER=nav2_navfn_planner::NavfnPlanner
)

nav2_add_test(test_dynamic_obstacle
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_system_with_obstacle_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
  TEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}
  ADD_OBSTACLE=True
  BT_NAVIGATOR_XML=navigate_to_pose_w_replanning_and_recovery.xml
  TESTER=nav_to_pose_tester_node.py
  ASTAR=False
  CONTROLLER=dwb_core::DWBLocalPlanner
  PLANNER=nav2_navfn_planner::NavfnPlanner
)

nav2_add_test(test_nav_through_poses
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_system_with_obstacle_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
  TEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}
  ADD_OBSTACLE=True
  BT_NAVIGATOR_XML=navigate_through_poses_w_replanning_and_recovery.xml
  TESTER=nav_through_poses_tester_node.py
  ASTAR=False
  CONTROLLER=dwb_core::DWBLocalPlanner
  PLANNER=nav2_navfn_planner::NavfnPlanner
)

nav2_add_test(test_nav_through_poses_error_msg
  GENERATE_RESULT_FOR_RETURN_CODE_ZERO
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/test_system_with_obstacle_launch.py"
  WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
  TIMEOUT 180
  ENV
  TEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}
  ADD_OBSTACLE=True
  BT_NAVIGATOR_XML=navigate_through_poses_w_replanning_and_recovery.xml
  TESTER=nav_through_poses_tester_error_msg_node.py
  ASTAR=False
  CONTROLLER=dwb_core::DWBLocalPlanner
  PLANNER=nav2_navfn_planner::NavfnPlanner
)
