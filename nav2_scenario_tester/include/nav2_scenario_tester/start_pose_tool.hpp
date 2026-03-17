// Copyright (c) 2026, Dexory (Tony Najjar)
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_SCENARIO_TESTER__START_POSE_TOOL_HPP_
#define NAV2_SCENARIO_TESTER__START_POSE_TOOL_HPP_

#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/pose.hpp>
#include <rviz_common/load_resource.hpp>
#include <rviz_default_plugins/tools/pose/pose_tool.hpp>

namespace nav2_scenario_tester
{

class StartPoseTool : public rviz_default_plugins::tools::PoseTool
{
  Q_OBJECT

public:
  StartPoseTool();
  ~StartPoseTool() override = default;
  void onInitialize() override;

protected:
  void onPoseSet(double x, double y, double theta) override;

Q_SIGNALS:
  void startPoseSet(const geometry_msgs::msg::Pose & pose);
};

}  // namespace nav2_scenario_tester

#endif  // NAV2_SCENARIO_TESTER__START_POSE_TOOL_HPP_
