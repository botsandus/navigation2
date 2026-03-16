// Copyright (c) 2026, Dexory (Tony Najjar)
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_SCENARIO_TESTER__OBSTACLE_TOOL_HPP_
#define NAV2_SCENARIO_TESTER__OBSTACLE_TOOL_HPP_

#include <memory>

#include <geometry_msgs/msg/point.hpp>
#include <rviz_common/tool.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/viewport_projection_finder.hpp>

namespace nav2_scenario_tester
{

class ObstacleTool : public rviz_common::Tool
{
  Q_OBJECT

public:
  ObstacleTool();
  ~ObstacleTool() override = default;
  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

Q_SIGNALS:
  void vertexPlaced(double x, double y);

private:
  std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;
};

}  // namespace nav2_scenario_tester

#endif  // NAV2_SCENARIO_TESTER__OBSTACLE_TOOL_HPP_
