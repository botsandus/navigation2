// Copyright (c) 2024 Open Navigation LLC
// Licensed under the Apache License, Version 2.0

#include "nav2_navigation_test/nav_test_designer_panel.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <QMessageBox>

#include <fstream>
#include <sstream>

#include <pluginlib/class_list_macros.hpp>

#include "nav2_navigation_test/goal_pose_tool.hpp"
#include "nav2_navigation_test/start_pose_tool.hpp"

namespace nav2_navigation_test
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
  marker_pub_.reset();
  node_.reset();
}

void NavTestDesignerPanel::onInitialize()
{
  node_ = std::make_shared<rclcpp::Node>("nav_test_designer");
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/nav_test_designer/markers", 10);

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

  // Buttons row 3: Save / Load YAML
  auto * row3 = new QHBoxLayout;
  save_btn_ = new QPushButton("Save YAML", this);
  load_btn_ = new QPushButton("Load YAML", this);
  connect(save_btn_, &QPushButton::clicked, this, &NavTestDesignerPanel::saveYaml);
  connect(load_btn_, &QPushButton::clicked, this, &NavTestDesignerPanel::loadYaml);
  row3->addWidget(save_btn_);
  row3->addWidget(load_btn_);
  layout->addLayout(row3);

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
  test_cases_[row] = readRowData(row);
  updateMarkers();
}

void NavTestDesignerPanel::saveYaml()
{
  QString path = QFileDialog::getSaveFileName(
    this, "Save Test Cases", "", "YAML files (*.yaml *.yml)");
  if (path.isEmpty()) {return;}

  std::ofstream ofs(path.toStdString());
  if (!ofs.is_open()) {
    QMessageBox::warning(this, "Error", "Cannot open file for writing.");
    return;
  }

  ofs << "test_cases:\n";
  for (const auto & tc : test_cases_) {
    ofs << "  - name: " << tc.name << "\n";
    ofs << "    initial_pose: {x: " << tc.start_x
        << ", y: " << tc.start_y
        << ", yaw: " << tc.start_yaw << "}\n";
    ofs << "    goal_pose: {x: " << tc.goal_x
        << ", y: " << tc.goal_y
        << ", yaw: " << tc.goal_yaw << "}\n";
    ofs << "    timeout: 90.0\n";
  }
  ofs.close();
}

void NavTestDesignerPanel::loadYaml()
{
  QString path = QFileDialog::getOpenFileName(
    this, "Load Test Cases", "", "YAML files (*.yaml *.yml)");
  if (path.isEmpty()) {return;}

  // Minimal YAML parser — reads the specific format we write
  std::ifstream ifs(path.toStdString());
  if (!ifs.is_open()) {
    QMessageBox::warning(this, "Error", "Cannot open file.");
    return;
  }

  test_cases_.clear();
  table_->setRowCount(0);

  std::string line;
  TestCaseData current;
  bool in_case = false;

  auto parse_value = [](const std::string & s, const std::string & key) -> std::string {
      auto pos = s.find(key);
      if (pos == std::string::npos) {return "";}
      pos += key.size();
      // skip whitespace
      while (pos < s.size() && (s[pos] == ' ' || s[pos] == ':')) {pos++;}
      auto end = s.find_first_of(",}", pos);
      if (end == std::string::npos) {end = s.size();}
      return s.substr(pos, end - pos);
    };

  while (std::getline(ifs, line)) {
    // Trim leading whitespace
    auto start = line.find_first_not_of(" \t");
    if (start == std::string::npos) {continue;}
    std::string trimmed = line.substr(start);

    if (trimmed.rfind("- name:", 0) == 0) {
      if (in_case) {
        test_cases_.push_back(current);
      }
      current = TestCaseData();
      current.name = trimmed.substr(8);  // after "- name: "
      // Trim trailing whitespace
      while (!current.name.empty() && current.name.back() == ' ') {
        current.name.pop_back();
      }
      in_case = true;
    } else if (trimmed.rfind("initial_pose:", 0) == 0) {
      auto x_str = parse_value(trimmed, "x:");
      auto y_str = parse_value(trimmed, "y:");
      auto yaw_str = parse_value(trimmed, "yaw:");
      if (!x_str.empty()) {current.start_x = std::stod(x_str);}
      if (!y_str.empty()) {current.start_y = std::stod(y_str);}
      if (!yaw_str.empty()) {current.start_yaw = std::stod(yaw_str);}
    } else if (trimmed.rfind("goal_pose:", 0) == 0) {
      auto x_str = parse_value(trimmed, "x:");
      auto y_str = parse_value(trimmed, "y:");
      auto yaw_str = parse_value(trimmed, "yaw:");
      if (!x_str.empty()) {current.goal_x = std::stod(x_str);}
      if (!y_str.empty()) {current.goal_y = std::stod(y_str);}
      if (!yaw_str.empty()) {current.goal_yaw = std::stod(yaw_str);}
    }
  }
  if (in_case) {
    test_cases_.push_back(current);
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
  }

  marker_pub_->publish(markers);
}

}  // namespace nav2_navigation_test

PLUGINLIB_EXPORT_CLASS(nav2_navigation_test::NavTestDesignerPanel, rviz_common::Panel)
