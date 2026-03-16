// Copyright (c) 2024 Open Navigation LLC
// Licensed under the Apache License, Version 2.0

#include "nav2_scenario_tester/nav_test_designer_panel.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <QMessageBox>

#include <fstream>
#include <filesystem>
#include <sstream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "nav2_scenario_tester/goal_pose_tool.hpp"
#include "nav2_scenario_tester/obstacle_tool.hpp"
#include "nav2_scenario_tester/start_pose_tool.hpp"

namespace nav2_scenario_tester
{

NavTestDesignerPanel::NavTestDesignerPanel(QWidget * parent)
: rviz_common::Panel(parent)
{
  createLayout();
}

NavTestDesignerPanel::~NavTestDesignerPanel()
{
  executor_.cancel();
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  load_map_client_.reset();
  marker_pub_.reset();
  node_.reset();
}

void NavTestDesignerPanel::onInitialize()
{
  node_ = std::make_shared<rclcpp::Node>("nav_test_designer");
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/nav_test_designer/markers", 10);
  load_map_client_ = node_->create_client<nav2_msgs::srv::LoadMap>(
    "/map_server/load_map");

  spin_thread_ = std::thread([this]() {
        executor_.add_node(node_);
        executor_.spin();
    });

  // Connect tool signals — find tools by class name
  auto * tool_manager = getDisplayContext()->getToolManager();
  for (int i = 0; i < tool_manager->numTools(); ++i) {
    auto * tool = tool_manager->getTool(i);
    auto * start_tool = dynamic_cast<StartPoseTool *>(tool);
    if (start_tool) {
      connect(
        start_tool, &StartPoseTool::startPoseSet,
        this, &NavTestDesignerPanel::onStartPoseSet);
    }
    auto * goal_tool = dynamic_cast<GoalPoseTool *>(tool);
    if (goal_tool) {
      connect(
        goal_tool, &GoalPoseTool::goalPoseSet,
        this, &NavTestDesignerPanel::onGoalPoseSet);
    }
    auto * obs_tool = dynamic_cast<ObstacleTool *>(tool);
    if (obs_tool) {
      connect(
        obs_tool, &ObstacleTool::vertexPlaced,
        this, &NavTestDesignerPanel::onObstacleVertex);
    }
  }
}

void NavTestDesignerPanel::createLayout()
{
  auto * layout = new QVBoxLayout;

  // Table
  table_ = new QTableWidget(0, NUM_COLS, this);
  table_->setHorizontalHeaderLabels(
    {"Name", "Start X", "Start Y", "Start Yaw", "Goal X", "Goal Y", "Goal Yaw"});
  table_->horizontalHeader()->setStretchLastSection(true);
  table_->setSelectionBehavior(QAbstractItemView::SelectRows);
  table_->setSelectionMode(QAbstractItemView::SingleSelection);
  connect(table_, &QTableWidget::cellChanged, this, &NavTestDesignerPanel::onTableCellChanged);
  layout->addWidget(table_);

  // Buttons row 1: Add / Remove
  auto * row1 = new QHBoxLayout;
  add_btn_ = new QPushButton("Add Test Case", this);
  remove_btn_ = new QPushButton("Remove Selected", this);
  connect(add_btn_, &QPushButton::clicked, this, &NavTestDesignerPanel::addTestCase);
  connect(remove_btn_, &QPushButton::clicked, this, &NavTestDesignerPanel::removeTestCase);
  row1->addWidget(add_btn_);
  row1->addWidget(remove_btn_);
  layout->addLayout(row1);

  // Buttons row 2: Set Start / Set Goal
  auto * row2 = new QHBoxLayout;
  start_btn_ = new QPushButton("Set Start (click map)", this);
  goal_btn_ = new QPushButton("Set Goal (click map)", this);
  connect(start_btn_, &QPushButton::clicked, this, &NavTestDesignerPanel::activateStartPoseTool);
  connect(goal_btn_, &QPushButton::clicked, this, &NavTestDesignerPanel::activateGoalPoseTool);
  row2->addWidget(start_btn_);
  row2->addWidget(goal_btn_);
  layout->addLayout(row2);

  // Buttons row 3: Obstacle drawing
  auto * row3 = new QHBoxLayout;
  draw_obs_btn_ = new QPushButton("Draw Obstacle", this);
  finish_obs_btn_ = new QPushButton("Finish", this);
  cancel_obs_btn_ = new QPushButton("Cancel", this);
  remove_obs_btn_ = new QPushButton("Remove Last Obstacle", this);
  finish_obs_btn_->setEnabled(false);
  cancel_obs_btn_->setEnabled(false);
  connect(draw_obs_btn_, &QPushButton::clicked, this, &NavTestDesignerPanel::startDrawingObstacle);
  connect(finish_obs_btn_, &QPushButton::clicked, this, &NavTestDesignerPanel::finishObstacle);
  connect(cancel_obs_btn_, &QPushButton::clicked, this, &NavTestDesignerPanel::cancelObstacle);
  connect(
    remove_obs_btn_, &QPushButton::clicked,
    this, &NavTestDesignerPanel::removeLastObstacle);
  row3->addWidget(draw_obs_btn_);
  row3->addWidget(finish_obs_btn_);
  row3->addWidget(cancel_obs_btn_);
  row3->addWidget(remove_obs_btn_);
  layout->addLayout(row3);

  // Buttons row 4: Save / Load YAML
  auto * row4 = new QHBoxLayout;
  save_btn_ = new QPushButton("Save YAML", this);
  load_btn_ = new QPushButton("Load YAML", this);
  connect(save_btn_, &QPushButton::clicked, this, &NavTestDesignerPanel::saveYaml);
  connect(load_btn_, &QPushButton::clicked, this, &NavTestDesignerPanel::loadYaml);
  row4->addWidget(save_btn_);
  row4->addWidget(load_btn_);
  layout->addLayout(row4);

  setLayout(layout);
}

// ── Slots ──────────────────────────────────────────────────────────────

void NavTestDesignerPanel::addTestCase()
{
  TestCaseData tc;
  tc.name = "test_" + std::to_string(test_cases_.size() + 1);
  test_cases_.push_back(tc);

  int row = table_->rowCount();
  table_->blockSignals(true);
  table_->insertRow(row);
  updateRowFromData(row);
  table_->blockSignals(false);

  table_->selectRow(row);
  updateMarkers();
}

void NavTestDesignerPanel::removeTestCase()
{
  int row = table_->currentRow();
  if (row < 0) {return;}

  test_cases_.erase(test_cases_.begin() + row);
  table_->removeRow(row);
  updateMarkers();
}

void NavTestDesignerPanel::activateStartPoseTool()
{
  if (table_->currentRow() < 0) {
    QMessageBox::information(this, "Info", "Select a test case row first.");
    return;
  }
  auto * tool_manager = getDisplayContext()->getToolManager();
  for (int i = 0; i < tool_manager->numTools(); ++i) {
    if (dynamic_cast<StartPoseTool *>(tool_manager->getTool(i))) {
      tool_manager->setCurrentTool(tool_manager->getTool(i));
      return;
    }
  }
}

void NavTestDesignerPanel::activateGoalPoseTool()
{
  if (table_->currentRow() < 0) {
    QMessageBox::information(this, "Info", "Select a test case row first.");
    return;
  }
  auto * tool_manager = getDisplayContext()->getToolManager();
  for (int i = 0; i < tool_manager->numTools(); ++i) {
    if (dynamic_cast<GoalPoseTool *>(tool_manager->getTool(i))) {
      tool_manager->setCurrentTool(tool_manager->getTool(i));
      return;
    }
  }
}

void NavTestDesignerPanel::onStartPoseSet(const geometry_msgs::msg::Pose & pose)
{
  int row = table_->currentRow();
  if (row < 0 || row >= static_cast<int>(test_cases_.size())) {return;}

  tf2::Quaternion q(pose.orientation.x, pose.orientation.y,
    pose.orientation.z, pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  test_cases_[row].start_x = pose.position.x;
  test_cases_[row].start_y = pose.position.y;
  test_cases_[row].start_yaw = yaw;

  table_->blockSignals(true);
  updateRowFromData(row);
  table_->blockSignals(false);
  updateMarkers();
}

void NavTestDesignerPanel::onGoalPoseSet(const geometry_msgs::msg::Pose & pose)
{
  int row = table_->currentRow();
  if (row < 0 || row >= static_cast<int>(test_cases_.size())) {return;}

  tf2::Quaternion q(pose.orientation.x, pose.orientation.y,
    pose.orientation.z, pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  test_cases_[row].goal_x = pose.position.x;
  test_cases_[row].goal_y = pose.position.y;
  test_cases_[row].goal_yaw = yaw;

  table_->blockSignals(true);
  updateRowFromData(row);
  table_->blockSignals(false);
  updateMarkers();
}

void NavTestDesignerPanel::onTableCellChanged(int row, int /*column*/)
{
  if (row < 0 || row >= static_cast<int>(test_cases_.size())) {return;}
  auto & tc = test_cases_[row];
  auto updated = readRowData(row);
  tc.name = updated.name;
  tc.start_x = updated.start_x;
  tc.start_y = updated.start_y;
  tc.start_yaw = updated.start_yaw;
  tc.goal_x = updated.goal_x;
  tc.goal_y = updated.goal_y;
  tc.goal_yaw = updated.goal_yaw;
  updateMarkers();
}

void NavTestDesignerPanel::startDrawingObstacle()
{
  int row = table_->currentRow();
  if (row < 0) {
    QMessageBox::information(this, "Info", "Select a test case row first.");
    return;
  }
  drawing_obstacle_ = true;
  pending_polygon_.clear();
  draw_obs_btn_->setEnabled(false);
  finish_obs_btn_->setEnabled(true);
  cancel_obs_btn_->setEnabled(true);

  // Activate the obstacle tool
  auto * tool_manager = getDisplayContext()->getToolManager();
  for (int i = 0; i < tool_manager->numTools(); ++i) {
    if (dynamic_cast<ObstacleTool *>(tool_manager->getTool(i))) {
      tool_manager->setCurrentTool(tool_manager->getTool(i));
      break;
    }
  }
}

void NavTestDesignerPanel::finishObstacle()
{
  int row = table_->currentRow();
  if (row >= 0 && row < static_cast<int>(test_cases_.size()) &&
    pending_polygon_.size() >= 3)
  {
    test_cases_[row].obstacles.push_back(pending_polygon_);
  }
  pending_polygon_.clear();
  drawing_obstacle_ = false;
  draw_obs_btn_->setEnabled(true);
  finish_obs_btn_->setEnabled(false);
  cancel_obs_btn_->setEnabled(false);
  updateMarkers();
}

void NavTestDesignerPanel::cancelObstacle()
{
  pending_polygon_.clear();
  drawing_obstacle_ = false;
  draw_obs_btn_->setEnabled(true);
  finish_obs_btn_->setEnabled(false);
  cancel_obs_btn_->setEnabled(false);
  updateMarkers();
}

void NavTestDesignerPanel::removeLastObstacle()
{
  int row = table_->currentRow();
  if (row < 0 || row >= static_cast<int>(test_cases_.size())) {return;}
  if (test_cases_[row].obstacles.empty()) {return;}
  test_cases_[row].obstacles.pop_back();
  updateMarkers();
}

void NavTestDesignerPanel::onObstacleVertex(double x, double y)
{
  if (!drawing_obstacle_) {return;}
  pending_polygon_.emplace_back(x, y);
  updateMarkers();
}

void NavTestDesignerPanel::saveYaml()
{
  QString path = QFileDialog::getSaveFileName(
    this, "Save Test Cases", "", "YAML files (*.yaml *.yml)");
  if (path.isEmpty()) {return;}

  YAML::Node root;
  YAML::Node seq(YAML::NodeType::Sequence);

  // Format a double as a clean YAML scalar (avoids IEEE 754 noise like 44.700000000000003).
  // Workaround for yaml-cpp 0.8: SetDoublePrecision doesn't apply when emitting from
  // YAML::Node. Fixed upstream in https://github.com/jbeder/yaml-cpp/pull/1407.
  auto yaml_num = [](double v) -> YAML::Node {
      std::ostringstream ss;
      ss << std::fixed;
      // Use enough decimals to preserve precision, then trim trailing zeros
      ss.precision(6);
      ss << v;
      std::string s = ss.str();
      auto dot = s.find('.');
      if (dot != std::string::npos) {
        auto last = s.find_last_not_of('0');
        if (last == dot) {last++;}  // keep at least one digit after dot
        s.erase(last + 1);
      }
      return YAML::Load(s);
    };

  for (auto & tc : test_cases_) {
    // Start from the original node (preserves unknown fields), or create new
    YAML::Node node = tc.yaml_node.IsDefined() ? YAML::Clone(tc.yaml_node) : YAML::Node();

    node["name"] = tc.name;

    node["initial_pose"]["x"] = yaml_num(tc.start_x);
    node["initial_pose"]["y"] = yaml_num(tc.start_y);
    node["initial_pose"]["yaw"] = yaml_num(tc.start_yaw);
    node["initial_pose"].SetStyle(YAML::EmitterStyle::Flow);

    node["goal_pose"]["x"] = yaml_num(tc.goal_x);
    node["goal_pose"]["y"] = yaml_num(tc.goal_y);
    node["goal_pose"]["yaw"] = yaml_num(tc.goal_yaw);
    node["goal_pose"].SetStyle(YAML::EmitterStyle::Flow);

    // Write obstacles
    if (!tc.obstacles.empty()) {
      YAML::Node obs_seq(YAML::NodeType::Sequence);
      for (const auto & poly : tc.obstacles) {
        YAML::Node poly_node(YAML::NodeType::Sequence);
        for (const auto & pt : poly) {
          YAML::Node coord(YAML::NodeType::Sequence);
          coord.push_back(yaml_num(pt.first));
          coord.push_back(yaml_num(pt.second));
          coord.SetStyle(YAML::EmitterStyle::Flow);
          poly_node.push_back(coord);
        }
        poly_node.SetStyle(YAML::EmitterStyle::Flow);
        obs_seq.push_back(poly_node);
      }
      node["obstacles"] = obs_seq;
    } else {
      node.remove("obstacles");
    }

    seq.push_back(node);
  }

  root["test_cases"] = seq;
  if (!map_yaml_value_.empty()) {
    root["map"] = map_yaml_value_;
  }

  std::ofstream ofs(path.toStdString());
  if (!ofs.is_open()) {
    QMessageBox::warning(this, "Error", "Cannot open file for writing.");
    return;
  }
  YAML::Emitter emitter;
  emitter.SetDoublePrecision(6);
  emitter.SetNullFormat(YAML::LowerNull);
  emitter << root;
  ofs << emitter.c_str() << "\n";
  ofs.close();
}

void NavTestDesignerPanel::loadYaml()
{
  QString path = QFileDialog::getOpenFileName(
    this, "Load Test Cases", "", "YAML files (*.yaml *.yml)");
  if (path.isEmpty()) {return;}
  loadYamlFromPath(path.toStdString());
}

void NavTestDesignerPanel::loadYamlFromPath(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception & e) {
    QMessageBox::warning(this, "Error",
      QString("Failed to parse YAML: %1").arg(e.what()));
    return;
  }

  // Load map if specified
  if (root["map"]) {
    map_yaml_value_ = root["map"].as<std::string>();
    auto yaml_dir = std::filesystem::path(path).parent_path().string();
    auto map_path = resolveMapPath(map_yaml_value_, yaml_dir);
    if (!map_path.empty() && map_path != current_map_) {
      auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
      request->map_url = map_path;
      auto captured_path = map_path;
      load_map_client_->async_send_request(
        request,
        [this, captured_path](rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedFuture future) {
          auto result = future.get();
          if (result->result == result->RESULT_SUCCESS) {
            current_map_ = captured_path;
          } else {
            RCLCPP_WARN(node_->get_logger(), "LoadMap failed (code %d)", result->result);
          }
        });
    }
  }

  test_cases_.clear();
  table_->setRowCount(0);

  if (!root["test_cases"] || !root["test_cases"].IsSequence()) {return;}

  for (const auto & node : root["test_cases"]) {
    TestCaseData tc;
    tc.yaml_node = YAML::Clone(node);

    tc.name = node["name"].as<std::string>("test");

    if (node["initial_pose"]) {
      tc.start_x = node["initial_pose"]["x"].as<double>(0.0);
      tc.start_y = node["initial_pose"]["y"].as<double>(0.0);
      tc.start_yaw = node["initial_pose"]["yaw"].as<double>(0.0);
    }
    if (node["goal_pose"]) {
      tc.goal_x = node["goal_pose"]["x"].as<double>(0.0);
      tc.goal_y = node["goal_pose"]["y"].as<double>(0.0);
      tc.goal_yaw = node["goal_pose"]["yaw"].as<double>(0.0);
    }

    // Parse obstacles
    if (node["obstacles"] && node["obstacles"].IsSequence()) {
      for (const auto & poly_node : node["obstacles"]) {
        if (!poly_node.IsSequence()) {continue;}
        Polygon2D poly;
        for (const auto & pt_node : poly_node) {
          if (pt_node.IsSequence() && pt_node.size() >= 2) {
            poly.emplace_back(pt_node[0].as<double>(), pt_node[1].as<double>());
          }
        }
        if (poly.size() >= 3) {
          tc.obstacles.push_back(poly);
        }
      }
    }

    test_cases_.push_back(tc);
  }

  // Populate table
  table_->blockSignals(true);
  for (size_t i = 0; i < test_cases_.size(); ++i) {
    table_->insertRow(static_cast<int>(i));
    updateRowFromData(static_cast<int>(i));
  }
  table_->blockSignals(false);
  updateMarkers();
}

std::string NavTestDesignerPanel::resolveMapPath(
  const std::string & map_value, const std::string & yaml_dir)
{
  namespace fs = std::filesystem;
  // Absolute path — use directly
  if (fs::path(map_value).is_absolute()) {
    return fs::exists(map_value) ? map_value : std::string();
  }
  // Relative to the YAML file's directory
  auto candidate = fs::path(yaml_dir) / map_value;
  if (fs::exists(candidate)) {
    return fs::canonical(candidate).string();
  }
  // Relative to the package's maps/ directory
  candidate = fs::path(ament_index_cpp::get_package_share_directory(
    "nav2_scenario_tester")) / "maps" / map_value;
  if (fs::exists(candidate)) {
    return fs::canonical(candidate).string();
  }
  return std::string();
}

// ── Helpers ────────────────────────────────────────────────────────────

void NavTestDesignerPanel::updateRowFromData(int row)
{
  const auto & tc = test_cases_[row];
  auto setItem = [&](int col, const std::string & text) {
      table_->setItem(row, col, new QTableWidgetItem(QString::fromStdString(text)));
    };

  auto fmt = [](double v) {
      std::ostringstream ss;
      ss << std::fixed;
      ss.precision(2);
      ss << v;
      return ss.str();
    };

  setItem(COL_NAME, tc.name);
  setItem(COL_START_X, fmt(tc.start_x));
  setItem(COL_START_Y, fmt(tc.start_y));
  setItem(COL_START_YAW, fmt(tc.start_yaw));
  setItem(COL_GOAL_X, fmt(tc.goal_x));
  setItem(COL_GOAL_Y, fmt(tc.goal_y));
  setItem(COL_GOAL_YAW, fmt(tc.goal_yaw));
}

TestCaseData NavTestDesignerPanel::readRowData(int row) const
{
  TestCaseData tc;
  auto text = [&](int col) -> std::string {
      auto * item = table_->item(row, col);
      return item ? item->text().toStdString() : "";
    };
  auto toDouble = [](const std::string & s) -> double {
      try {
        return std::stod(s);
      } catch (...) {
        return 0.0;
      }
    };

  tc.name = text(COL_NAME);
  tc.start_x = toDouble(text(COL_START_X));
  tc.start_y = toDouble(text(COL_START_Y));
  tc.start_yaw = toDouble(text(COL_START_YAW));
  tc.goal_x = toDouble(text(COL_GOAL_X));
  tc.goal_y = toDouble(text(COL_GOAL_Y));
  tc.goal_yaw = toDouble(text(COL_GOAL_YAW));
  return tc;
}

void NavTestDesignerPanel::updateMarkers()
{
  if (!marker_pub_) {return;}

  visualization_msgs::msg::MarkerArray markers;

  // First: delete-all marker to clear previous state
  visualization_msgs::msg::Marker del;
  del.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(del);

  // Distinct color palette for cycling per test case
  static const float palette[][3] = {
    {0.12f, 0.47f, 0.71f},   // blue
    {1.00f, 0.50f, 0.05f},   // orange
    {0.17f, 0.63f, 0.17f},   // green
    {0.84f, 0.15f, 0.16f},   // red
    {0.58f, 0.40f, 0.74f},   // purple
    {0.55f, 0.34f, 0.29f},   // brown
    {0.89f, 0.47f, 0.76f},   // pink
    {0.74f, 0.74f, 0.13f},   // olive
    {0.09f, 0.75f, 0.81f},   // cyan
  };
  static const size_t palette_size = sizeof(palette) / sizeof(palette[0]);

  int id = 0;
  for (size_t i = 0; i < test_cases_.size(); ++i) {
    const auto & tc = test_cases_[i];
    const float * rgb = palette[i % palette_size];

    auto make_arrow = [&](
      double x, double y, double yaw,
      float alpha, const std::string & ns) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = node_->now();
        m.ns = ns;
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::ARROW;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.position.x = x;
        m.pose.position.y = y;
        m.pose.position.z = 0.1;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        m.pose.orientation.x = q.x();
        m.pose.orientation.y = q.y();
        m.pose.orientation.z = q.z();
        m.pose.orientation.w = q.w();

        m.scale.x = 0.8;  // arrow length
        m.scale.y = 0.15;  // arrow width
        m.scale.z = 0.15;  // arrow height
        m.color.r = rgb[0];
        m.color.g = rgb[1];
        m.color.b = rgb[2];
        m.color.a = alpha;
        markers.markers.push_back(m);
      };

    // Faded arrow for start, solid arrow for goal — same color per test
    make_arrow(tc.start_x, tc.start_y, tc.start_yaw, 0.45f, "start");
    make_arrow(tc.goal_x, tc.goal_y, tc.goal_yaw, 0.9f, "goal");

    // Text label in the same test color
    visualization_msgs::msg::Marker text;
    text.header.frame_id = "map";
    text.header.stamp = node_->now();
    text.ns = "labels";
    text.id = id++;
    text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text.action = visualization_msgs::msg::Marker::ADD;
    text.pose.position.x = (tc.start_x + tc.goal_x) / 2.0;
    text.pose.position.y = (tc.start_y + tc.goal_y) / 2.0;
    text.pose.position.z = 0.5;
    text.scale.z = 0.3;
    text.color.r = rgb[0];
    text.color.g = rgb[1];
    text.color.b = rgb[2];
    text.color.a = 1.0f;
    text.text = tc.name;
    markers.markers.push_back(text);

    // Obstacle polygons as LINE_STRIP outlines
    for (const auto & poly : tc.obstacles) {
      if (poly.size() < 3) {continue;}
      visualization_msgs::msg::Marker outline;
      outline.header.frame_id = "map";
      outline.header.stamp = node_->now();
      outline.ns = "obstacles";
      outline.id = id++;
      outline.type = visualization_msgs::msg::Marker::LINE_STRIP;
      outline.action = visualization_msgs::msg::Marker::ADD;
      outline.pose.orientation.w = 1.0;
      outline.scale.x = 0.03;  // line width
      outline.color.r = rgb[0];
      outline.color.g = rgb[1];
      outline.color.b = rgb[2];
      outline.color.a = 0.9f;
      for (const auto & pt : poly) {
        geometry_msgs::msg::Point p;
        p.x = pt.first;  p.y = pt.second;  p.z = 0.06;
        outline.points.push_back(p);
      }
      // Close the polygon
      geometry_msgs::msg::Point p_close;
      p_close.x = poly[0].first;  p_close.y = poly[0].second;  p_close.z = 0.06;
      outline.points.push_back(p_close);
      markers.markers.push_back(outline);
    }
  }

  // Show in-progress polygon being drawn (pending vertices)
  if (!pending_polygon_.empty()) {
    // Dots at each placed vertex
    for (size_t v = 0; v < pending_polygon_.size(); ++v) {
      visualization_msgs::msg::Marker dot;
      dot.header.frame_id = "map";
      dot.header.stamp = node_->now();
      dot.ns = "pending_vertices";
      dot.id = id++;
      dot.type = visualization_msgs::msg::Marker::SPHERE;
      dot.action = visualization_msgs::msg::Marker::ADD;
      dot.pose.position.x = pending_polygon_[v].first;
      dot.pose.position.y = pending_polygon_[v].second;
      dot.pose.position.z = 0.08;
      dot.pose.orientation.w = 1.0;
      dot.scale.x = 0.1;
      dot.scale.y = 0.1;
      dot.scale.z = 0.1;
      dot.color.r = 1.0f;
      dot.color.g = 0.3f;
      dot.color.b = 0.3f;
      dot.color.a = 1.0f;
      markers.markers.push_back(dot);
    }

    // Lines connecting placed vertices so far
    if (pending_polygon_.size() >= 2) {
      visualization_msgs::msg::Marker line;
      line.header.frame_id = "map";
      line.header.stamp = node_->now();
      line.ns = "pending_outline";
      line.id = id++;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.pose.orientation.w = 1.0;
      line.scale.x = 0.04;
      line.color.r = 1.0f;
      line.color.g = 0.3f;
      line.color.b = 0.3f;
      line.color.a = 0.8f;
      for (const auto & pt : pending_polygon_) {
        geometry_msgs::msg::Point p;
        p.x = pt.first;  p.y = pt.second;  p.z = 0.07;
        line.points.push_back(p);
      }
      markers.markers.push_back(line);
    }
  }

  marker_pub_->publish(markers);
}

}  // namespace nav2_scenario_tester

PLUGINLIB_EXPORT_CLASS(nav2_scenario_tester::NavTestDesignerPanel, rviz_common::Panel)
