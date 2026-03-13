// Copyright (c) 2024 Open Navigation LLC
// Licensed under the Apache License, Version 2.0

#ifndef NAV2_NAVIGATION_TEST__NAV_TEST_DESIGNER_PANEL_HPP_
#define NAV2_NAVIGATION_TEST__NAV_TEST_DESIGNER_PANEL_HPP_

#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>
#include <yaml-cpp/yaml.h>

#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/tool_manager.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace nav2_navigation_test
{

/// A single polygon is a list of [x, y] vertices.
using Polygon2D = std::vector<std::pair<double, double>>;

struct TestCaseData
{
  std::string name;
  double start_x{0.0}, start_y{0.0}, start_yaw{0.0};
  double goal_x{0.0}, goal_y{0.0}, goal_yaw{0.0};
  std::vector<Polygon2D> obstacles;
  /// Full original YAML node — preserves unknown fields for round-tripping.
  YAML::Node yaml_node;
};

class NavTestDesignerPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit NavTestDesignerPanel(QWidget * parent = nullptr);
  ~NavTestDesignerPanel() override;
  void onInitialize() override;

private Q_SLOTS:
  void addTestCase();
  void removeTestCase();
  void activateStartPoseTool();
  void activateGoalPoseTool();
  void onStartPoseSet(const geometry_msgs::msg::Pose & pose);
  void onGoalPoseSet(const geometry_msgs::msg::Pose & pose);
  void onTableCellChanged(int row, int column);
  void startDrawingObstacle();
  void finishObstacle();
  void cancelObstacle();
  void removeLastObstacle();
  void onObstacleVertex(double x, double y);
  void saveYaml();
  void loadYaml();

private:
  void createLayout();
  void updateMarkers();
  void updateRowFromData(int row);
  TestCaseData readRowData(int row) const;

  // Qt widgets
  QTableWidget * table_;
  QPushButton * add_btn_;
  QPushButton * remove_btn_;
  QPushButton * start_btn_;
  QPushButton * goal_btn_;
  QPushButton * draw_obs_btn_;
  QPushButton * finish_obs_btn_;
  QPushButton * cancel_obs_btn_;
  QPushButton * remove_obs_btn_;
  QPushButton * save_btn_;
  QPushButton * load_btn_;

  // Data
  std::vector<TestCaseData> test_cases_;

  // Obstacle drawing state
  bool drawing_obstacle_{false};
  Polygon2D pending_polygon_;

  // ROS
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Column indices
  static constexpr int COL_NAME = 0;
  static constexpr int COL_START_X = 1;
  static constexpr int COL_START_Y = 2;
  static constexpr int COL_START_YAW = 3;
  static constexpr int COL_GOAL_X = 4;
  static constexpr int COL_GOAL_Y = 5;
  static constexpr int COL_GOAL_YAW = 6;
  static constexpr int NUM_COLS = 7;
};

}  // namespace nav2_navigation_test

#endif  // NAV2_NAVIGATION_TEST__NAV_TEST_DESIGNER_PANEL_HPP_
