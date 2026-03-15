// Copyright (c) 2024 Open Navigation LLC
// Licensed under the Apache License, Version 2.0

#include "nav2_scenario_tester/goal_pose_tool.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace nav2_scenario_tester
{

GoalPoseTool::GoalPoseTool()
: rviz_default_plugins::tools::PoseTool()
{
  shortcut_key_ = 'g';
}

void GoalPoseTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Set Goal Pose");
  setDescription("Click and drag on the map to set a test goal pose.");
  setIcon(rviz_common::loadPixmap(
    "package://rviz_default_plugins/icons/classes/SetGoal.png"));
}

void GoalPoseTool::onPoseSet(double x, double y, double theta)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = x;
  pose.position.y = y;

  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  pose.orientation.x = quat.x();
  pose.orientation.y = quat.y();
  pose.orientation.z = quat.z();
  pose.orientation.w = quat.w();

  Q_EMIT goalPoseSet(pose);
}

}  // namespace nav2_scenario_tester

PLUGINLIB_EXPORT_CLASS(nav2_scenario_tester::GoalPoseTool, rviz_common::Tool)
