// Copyright (c) 2024 Open Navigation LLC
// Licensed under the Apache License, Version 2.0

#include "nav2_navigation_test/start_pose_tool.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace nav2_navigation_test
{

StartPoseTool::StartPoseTool()
: rviz_default_plugins::tools::PoseTool()
{
  shortcut_key_ = 's';
}

void StartPoseTool::onInitialize()
{
  PoseTool::onInitialize();
  setName("Set Start Pose");
  setDescription("Click and drag on the map to set a test start pose.");
  setIcon(rviz_common::loadPixmap(
    "package://rviz_default_plugins/icons/classes/SetInitialPose.png"));
}

void StartPoseTool::onPoseSet(double x, double y, double theta)
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

  Q_EMIT startPoseSet(pose);
}

}  // namespace nav2_navigation_test

PLUGINLIB_EXPORT_CLASS(nav2_navigation_test::StartPoseTool, rviz_common::Tool)
