// Copyright (c) 2024 Open Navigation LLC
// Licensed under the Apache License, Version 2.0

#include "nav2_navigation_test/obstacle_tool.hpp"

#include <rviz_common/render_panel.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace nav2_navigation_test
{

ObstacleTool::ObstacleTool()
: rviz_common::Tool()
{
  shortcut_key_ = 'o';
  projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
}

void ObstacleTool::onInitialize()
{
  setName("Draw Obstacle");
  setDescription("Click on the map to place obstacle polygon vertices.");
}

void ObstacleTool::activate() {}
void ObstacleTool::deactivate() {}

int ObstacleTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  if (event.leftUp()) {
    auto result = projection_finder_->getViewportPointProjectionOnXYPlane(
      event.panel->getRenderWindow(), event.x, event.y);
    if (result.first) {
      Q_EMIT vertexPlaced(result.second.x, result.second.y);
    }
    return Render;
  }
  return 0;
}

}  // namespace nav2_navigation_test

PLUGINLIB_EXPORT_CLASS(nav2_navigation_test::ObstacleTool, rviz_common::Tool)
