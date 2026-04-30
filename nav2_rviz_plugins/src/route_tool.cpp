// Copyright (c) 2024 John Chrosniak
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_rviz_plugins/route_tool.hpp"

#include <sys/types.h>

#include <algorithm>
#include <cmath>
#include <functional>

#include "rviz_common/display_context.hpp"

#include <QDesktopServices>
#include <QFileDialog>
#include <QMetaObject>
#include <QUrl>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


namespace nav2_rviz_plugins
{
RouteTool::RouteTool(QWidget * parent)
:   rviz_common::Panel(parent),
  ui_(std::make_unique<Ui::route_tool>())
{
  // Extend the widget with all attributes and children from UI file
  ui_->setupUi(this);
  node_ = std::make_shared<nav2::LifecycleNode>("route_tool_node", "", rclcpp::NodeOptions());
  node_->configure();
  graph_vis_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "route_tool_graph", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  node_->activate();
  tf_ = nav2::create_transform_buffer(node_);
  graph_loader_ = std::make_shared<nav2_route::GraphLoader>(node_, tf_, "map");
  graph_saver_ = std::make_shared<nav2_route::GraphSaver>(node_, tf_, "map");
  ui_->add_node_button->setChecked(true);
  ui_->edit_node_button->setChecked(true);
  ui_->remove_node_button->setChecked(true);
  ui_->tabWidget->setCurrentIndex(0);  // Start on Add tab
  // Hide bi-directional controls initially (Node mode is default)
  ui_->make_bidirectional_button->setVisible(false);
  ui_->reverse_edge_label->setVisible(false);
  ui_->reverse_edge_id->setVisible(false);
  ui_->remove_reverse_edge_label->setVisible(false);
  ui_->remove_reverse_edge_id->setVisible(false);
  ui_->remove_bidirectional_checkbox->setVisible(false);
  ui_->remove_bidirectional_checkbox->setEnabled(false);
  // Needed to prevent memory addresses moving from resizing
  // when adding nodes and edges
  graph_.reserve(1000);
}

void RouteTool::onInitialize(void)
{
  auto ros_node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction) {
    RCLCPP_ERROR(
      node_->get_logger(), "Unable to get ROS node abstraction");
    return;
  }
  auto node = ros_node_abstraction->get_raw_node();

  clicked_point_subscription_ = node->create_subscription<geometry_msgs::msg::PointStamped>(
    "clicked_point", 1,
    std::bind(&RouteTool::on_clicked_point, this, std::placeholders::_1));

  // Use the rviz node (already spun by rviz) so the marker server's
  // get_interactive_markers service actually answers.
  im_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
    "/route_graph_nodes", node);

  set_route_graph_client_ = node->create_client<nav2_msgs::srv::SetRouteGraph>(
    "/route_server/set_route_graph");
}

void RouteTool::on_clicked_point(const geometry_msgs::msg::PointStamped::ConstSharedPtr & msg)
{
  constexpr int kEditTabIndex = 1;
  constexpr int kRemoveTabIndex = 2;
  constexpr float kEdgeSelectionDistance = 0.3f;  // Max distance to select an edge

  // Check if we're on the Edit tab with Edge selected
  if (ui_->tabWidget->currentIndex() == kEditTabIndex && ui_->edit_edge_button->isChecked()) {
    auto edge_info = find_nearest_edge(msg->point.x, msg->point.y, kEdgeSelectionDistance);
    if (edge_info.has_value()) {
      ui_->edit_id->setText(std::to_string(edge_info->first).c_str());
      ui_->edit_field_1->setText(std::to_string(edge_info->second.first).c_str());
      ui_->edit_field_2->setText(std::to_string(edge_info->second.second).c_str());

      // Check for reverse edge and update the label and button state
      auto reverse_edge = find_reverse_edge(edge_info->second.first, edge_info->second.second);
      if (reverse_edge.has_value()) {
        ui_->reverse_edge_id->setText(std::to_string(reverse_edge.value()).c_str());
        ui_->make_bidirectional_button->setEnabled(false);
        RCLCPP_INFO(
          node_->get_logger(), "Selected edge %d (from node %d to node %d), reverse edge: %d",
          edge_info->first, edge_info->second.first, edge_info->second.second,
          reverse_edge.value());
      } else {
        ui_->reverse_edge_id->setText("None");
        ui_->make_bidirectional_button->setEnabled(true);
        RCLCPP_INFO(
          node_->get_logger(), "Selected edge %d (from node %d to node %d), no reverse edge",
          edge_info->first, edge_info->second.first, edge_info->second.second);
      }
      return;
    }
  }

  // Check if we're on the Remove tab with Edge selected
  if (ui_->tabWidget->currentIndex() == kRemoveTabIndex && ui_->remove_edge_button->isChecked()) {
    auto edge_info = find_nearest_edge(msg->point.x, msg->point.y, kEdgeSelectionDistance);
    if (edge_info.has_value()) {
      ui_->remove_id->setText(std::to_string(edge_info->first).c_str());

      // Check for reverse edge and update the label
      auto reverse_edge = find_reverse_edge(edge_info->second.first, edge_info->second.second);
      if (reverse_edge.has_value()) {
        ui_->remove_reverse_edge_id->setText(std::to_string(reverse_edge.value()).c_str());
        ui_->remove_bidirectional_checkbox->setEnabled(true);
        RCLCPP_INFO(
          node_->get_logger(), "Selected edge %d for removal, reverse edge: %d",
          edge_info->first, reverse_edge.value());
      } else {
        ui_->remove_reverse_edge_id->setText("None");
        ui_->remove_bidirectional_checkbox->setEnabled(false);
        RCLCPP_INFO(
          node_->get_logger(), "Selected edge %d for removal, no reverse edge",
          edge_info->first);
      }
      return;
    }
  }

  // Default behavior: fill position fields
  ui_->add_field_1->setText(std::to_string(msg->point.x).c_str());
  ui_->add_field_2->setText(std::to_string(msg->point.y).c_str());
  ui_->edit_field_1->setText(std::to_string(msg->point.x).c_str());
  ui_->edit_field_2->setText(std::to_string(msg->point.y).c_str());
}

float RouteTool::point_to_segment_distance(
  float px, float py,
  float x1, float y1,
  float x2, float y2) const
{
  float dx = x2 - x1;
  float dy = y2 - y1;
  float length_sq = dx * dx + dy * dy;

  if (length_sq < 1e-6f) {
    // Segment is a point
    return std::hypotf(px - x1, py - y1);
  }

  // Project point onto line, clamped to segment
  float t = std::max(0.0f, std::min(1.0f, ((px - x1) * dx + (py - y1) * dy) / length_sq));
  float proj_x = x1 + t * dx;
  float proj_y = y1 + t * dy;

  return std::hypotf(px - proj_x, py - proj_y);
}

std::optional<std::pair<unsigned int, std::pair<unsigned int, unsigned int>>>
RouteTool::find_nearest_edge(float x, float y, float max_distance) const
{
  std::optional<std::pair<unsigned int, std::pair<unsigned int, unsigned int>>> result;
  float best_score = std::numeric_limits<float>::max();

  for (const auto & node : graph_) {
    if (node.nodeid == static_cast<unsigned int>(std::numeric_limits<int>::max())) {
      continue;  // Skip deleted nodes
    }
    for (const auto & edge : node.neighbors) {
      float x1 = node.coords.x;
      float y1 = node.coords.y;
      float x2 = edge.end->coords.x;
      float y2 = edge.end->coords.y;

      float dx = x2 - x1;
      float dy = y2 - y1;
      float length_sq = dx * dx + dy * dy;

      float dist, t;
      if (length_sq < 1e-6f) {
        // Segment is a point
        dist = std::hypotf(x - x1, y - y1);
        t = 0.5f;
      } else {
        // Calculate t parameter (0=start, 1=end) and distance
        t = std::max(0.0f, std::min(1.0f, ((x - x1) * dx + (y - y1) * dy) / length_sq));
        float proj_x = x1 + t * dx;
        float proj_y = y1 + t * dy;
        dist = std::hypotf(x - proj_x, y - proj_y);
      }

      if (dist > max_distance) {
        continue;
      }

      // Score: prefer edges where click is near the arrow head (t > 0.5)
      // For bi-directional edges, this helps select the correct direction
      // Lower score is better: distance + penalty for being near the tail
      float direction_penalty = (t < 0.5f) ? (0.5f - t) * max_distance : 0.0f;
      float score = dist + direction_penalty;

      if (score < best_score) {
        best_score = score;
        result = std::make_pair(
          edge.edgeid,
          std::make_pair(node.nodeid, edge.end->nodeid));
      }
    }
  }

  return result;
}

std::optional<unsigned int> RouteTool::find_reverse_edge(
  unsigned int start_node_id, unsigned int end_node_id) const
{
  // Look for an edge going from end_node_id to start_node_id
  if (graph_to_id_map_.find(end_node_id) == graph_to_id_map_.end()) {
    return std::nullopt;
  }

  const auto & end_node = graph_[graph_to_id_map_.at(end_node_id)];
  for (const auto & edge : end_node.neighbors) {
    if (edge.end->nodeid == start_node_id) {
      return edge.edgeid;
    }
  }

  return std::nullopt;
}

void RouteTool::on_load_button_clicked(void)
{
  QString filename = QFileDialog::getOpenFileName(
    this,
    tr("Open Address Book"), "",
    tr("Address Book (*.geojson);;All Files (*)"));
  if (filename.isEmpty()) {
    RCLCPP_INFO(node_->get_logger(), "Load operation cancelled by user");
    return;
  }
  graph_to_id_map_.clear();
  edge_to_node_map_.clear();
  graph_to_incoming_edges_map_.clear();
  graph_.clear();
  graph_loader_->loadGraphFromFile(graph_, graph_to_id_map_, filename.toStdString());
  unsigned int max_node_id = 0;
  for (const auto & node : graph_) {
    max_node_id = std::max(node.nodeid, max_node_id);
    for (const auto & edge : node.neighbors) {
      max_node_id = std::max(edge.edgeid, max_node_id);
      edge_to_node_map_[edge.edgeid] = node.nodeid;
      if (graph_to_incoming_edges_map_.find(edge.end->nodeid) !=
        graph_to_incoming_edges_map_.end())
      {
        graph_to_incoming_edges_map_[edge.end->nodeid].push_back(edge.edgeid);
      } else {
        graph_to_incoming_edges_map_[edge.end->nodeid] = std::vector<unsigned int> {edge.edgeid};
      }
    }
  }
  next_node_id_ = max_node_id + 1;
  update_route_graph();
}

void RouteTool::on_save_button_clicked(void)
{
  QString filename = QFileDialog::getSaveFileName(
    this,
    tr("Open Address Book"), "",
    tr("Address Book (*.geojson);;All Files (*)"));
  if (filename.isEmpty()) {
    return;
  }
  RCLCPP_INFO(node_->get_logger(), "Save graph to: %s", filename.toStdString().c_str());
  graph_saver_->saveGraphToFile(graph_, filename.toStdString());

  // Ask the route_server to reload the graph it just had written. Async so the
  // UI never blocks; if the server isn't there we just log a warning.
  if (!set_route_graph_client_) {
    return;
  }
  if (!set_route_graph_client_->service_is_ready()) {
    RCLCPP_WARN(
      node_->get_logger(),
      "set_route_graph service not available (route_server not running?); "
      "skipping server reload");
    return;
  }
  auto request = std::make_shared<nav2_msgs::srv::SetRouteGraph::Request>();
  request->graph_filepath = filename.toStdString();
  auto logger = node_->get_logger();
  set_route_graph_client_->async_send_request(
    request,
    [logger, filename](rclcpp::Client<nav2_msgs::srv::SetRouteGraph>::SharedFuture future) {
      auto response = future.get();
      if (response->success) {
        RCLCPP_INFO(
          logger, "route_server reloaded graph from %s",
          filename.toStdString().c_str());
      } else {
        RCLCPP_WARN(
          logger, "route_server rejected reload of %s",
          filename.toStdString().c_str());
      }
    });
}

void RouteTool::on_create_button_clicked(void)
{
  if (ui_->add_field_1->toPlainText() == "" || ui_->add_field_2->toPlainText() == "") {return;}
  if (ui_->add_node_button->isChecked()) {
    auto longitude = ui_->add_field_1->toPlainText().toFloat();
    auto latitude = ui_->add_field_2->toPlainText().toFloat();
    nav2_route::Node new_node;
    new_node.nodeid = next_node_id_;
    new_node.coords.x = longitude;
    new_node.coords.y = latitude;
    graph_.push_back(new_node);
    graph_to_id_map_[next_node_id_++] = graph_.size() - 1;
    RCLCPP_INFO(node_->get_logger(), "Adding node at: (%f, %f)", longitude, latitude);
    update_route_graph();
  } else if (ui_->add_edge_button->isChecked()) {
    auto start_node = ui_->add_field_1->toPlainText().toInt();
    auto end_node = ui_->add_field_2->toPlainText().toInt();
    nav2_route::EdgeCost edge_cost;
    graph_[graph_to_id_map_[start_node]].addEdge(
      edge_cost, &(graph_[graph_to_id_map_[end_node]]),
      next_node_id_);
    if (graph_to_incoming_edges_map_.find(end_node) != graph_to_incoming_edges_map_.end()) {
      graph_to_incoming_edges_map_[end_node].push_back(next_node_id_);
    } else {
      graph_to_incoming_edges_map_[end_node] = std::vector<unsigned int> {next_node_id_};
    }
    edge_to_node_map_[next_node_id_++] = start_node;
    RCLCPP_INFO(node_->get_logger(), "Adding edge from %d to %d", start_node, end_node);
    update_route_graph();
  }
  ui_->add_field_1->setText("");
  ui_->add_field_2->setText("");
}

void RouteTool::on_confirm_button_clicked(void)
{
  if (ui_->edit_id->toPlainText() == "" || ui_->edit_field_1->toPlainText() == "" ||
    ui_->edit_field_2->toPlainText() == "") {return;}
  if (ui_->edit_node_button->isChecked()) {
    auto node_id = ui_->edit_id->toPlainText().toInt();
    auto new_longitude = ui_->edit_field_1->toPlainText().toFloat();
    auto new_latitude = ui_->edit_field_2->toPlainText().toFloat();
    if (graph_to_id_map_.find(node_id) != graph_to_id_map_.end()) {
      graph_[graph_to_id_map_[node_id]].coords.x = new_longitude;
      graph_[graph_to_id_map_[node_id]].coords.y = new_latitude;
      update_route_graph();
    }
  } else if (ui_->edit_edge_button->isChecked()) {
    auto edge_id = (unsigned int) ui_->edit_id->toPlainText().toInt();
    auto new_start = ui_->edit_field_1->toPlainText().toInt();
    auto new_end = ui_->edit_field_2->toPlainText().toInt();

    // Find and remove current edge
    auto current_start_node = &graph_[graph_to_id_map_[edge_to_node_map_[edge_id]]];
    for (auto itr = current_start_node->neighbors.begin();
      itr != current_start_node->neighbors.end(); itr++)
    {
      if (itr->edgeid == edge_id) {
        current_start_node->neighbors.erase(itr);
        break;
      }
    }

    // Create new edge with same ID using new start and stop nodes
    nav2_route::EdgeCost edge_cost;
    graph_[graph_to_id_map_[new_start]].addEdge(
      edge_cost, &(graph_[graph_to_id_map_[new_end]]),
      edge_id);
    edge_to_node_map_[edge_id] = new_start;
    if (graph_to_incoming_edges_map_.find(new_end) != graph_to_incoming_edges_map_.end()) {
      graph_to_incoming_edges_map_[new_end].push_back(edge_id);
    } else {
      graph_to_incoming_edges_map_[new_end] = std::vector<unsigned int> {edge_id};
    }

    update_route_graph();
  }
  ui_->edit_id->setText("");
  ui_->edit_field_1->setText("");
  ui_->edit_field_2->setText("");
  ui_->reverse_edge_id->setText("None");
  ui_->make_bidirectional_button->setEnabled(false);
}

void RouteTool::on_delete_button_clicked(void)
{
  if (ui_->remove_id->toPlainText() == "") {return;}
  if (ui_->remove_node_button->isChecked()) {
    unsigned int node_id = ui_->remove_id->toPlainText().toInt();
    auto map_it = graph_to_id_map_.find(node_id);
    if (map_it == graph_to_id_map_.end()) {
      return;
    }
    auto * deleted_node = &graph_[map_it->second];

    // Remove outgoing edges (FROM the deleted node TO others).
    for (const auto & edge : deleted_node->neighbors) {
      edge_to_node_map_.erase(edge.edgeid);
      auto inc_it = graph_to_incoming_edges_map_.find(edge.end->nodeid);
      if (inc_it != graph_to_incoming_edges_map_.end()) {
        auto & vec = inc_it->second;
        vec.erase(std::remove(vec.begin(), vec.end(), edge.edgeid), vec.end());
      }
      RCLCPP_INFO(node_->get_logger(), "Removed outgoing edge %d", edge.edgeid);
    }
    deleted_node->neighbors.clear();

    // Remove incoming edges (FROM others TO the deleted node).
    for (auto edge_id : graph_to_incoming_edges_map_[node_id]) {
      auto start_node = &graph_[graph_to_id_map_[edge_to_node_map_[edge_id]]];
      for (auto itr = start_node->neighbors.begin(); itr != start_node->neighbors.end(); itr++) {
        if (itr->edgeid == edge_id) {
          start_node->neighbors.erase(itr);
          edge_to_node_map_.erase(edge_id);
          RCLCPP_INFO(node_->get_logger(), "Removed incoming edge %d", edge_id);
          break;
        }
      }
    }

    // Mark node as deleted.
    deleted_node->nodeid = std::numeric_limits<int>::max();
    graph_to_id_map_.erase(node_id);
    graph_to_incoming_edges_map_.erase(node_id);
    RCLCPP_INFO(node_->get_logger(), "Removed node %d", node_id);
    update_route_graph();
  } else if (ui_->remove_edge_button->isChecked()) {
    auto edge_id = (unsigned int) ui_->remove_id->toPlainText().toInt();

    // Find edge endpoints before removing
    auto edge_start_node_id = edge_to_node_map_[edge_id];
    unsigned int edge_end_node_id = 0;
    auto start_node = &graph_[graph_to_id_map_[edge_start_node_id]];
    for (const auto & edge : start_node->neighbors) {
      if (edge.edgeid == edge_id) {
        edge_end_node_id = edge.end->nodeid;
        break;
      }
    }

    // Find reverse edge if checkbox is checked
    std::optional<unsigned int> reverse_edge_id;
    if (ui_->remove_bidirectional_checkbox->isChecked()) {
      reverse_edge_id = find_reverse_edge(edge_start_node_id, edge_end_node_id);
    }

    // Remove the main edge
    for (auto itr = start_node->neighbors.begin(); itr != start_node->neighbors.end(); itr++) {
      if (itr->edgeid == edge_id) {
        start_node->neighbors.erase(itr);
        edge_to_node_map_.erase(edge_id);
        RCLCPP_INFO(node_->get_logger(), "Removed edge %d", edge_id);
        break;
      }
    }

    // Remove reverse edge if found and checkbox is checked
    if (reverse_edge_id.has_value()) {
      auto rev_edge_id = reverse_edge_id.value();
      auto rev_start_node = &graph_[graph_to_id_map_[edge_to_node_map_[rev_edge_id]]];
      for (auto itr = rev_start_node->neighbors.begin();
        itr != rev_start_node->neighbors.end(); itr++)
      {
        if (itr->edgeid == rev_edge_id) {
          rev_start_node->neighbors.erase(itr);
          edge_to_node_map_.erase(rev_edge_id);
          RCLCPP_INFO(node_->get_logger(), "Removed reverse edge %d", rev_edge_id);
          break;
        }
      }
    }

    update_route_graph();
  }
  ui_->remove_id->setText("");
  ui_->remove_reverse_edge_id->setText("None");
  ui_->remove_bidirectional_checkbox->setEnabled(false);
}

void RouteTool::on_add_node_button_toggled(void)
{
  if (ui_->add_node_button->isChecked()) {
    ui_->add_text->setText("Position:");
    ui_->add_label_1->setText("X:");
    ui_->add_label_2->setText("Y:");
  } else {
    ui_->add_text->setText("Connections:");
    ui_->add_label_1->setText("Start Node ID:");
    ui_->add_label_2->setText("End Node ID:");
  }
}

void RouteTool::on_edit_node_button_toggled(void)
{
  if (ui_->edit_node_button->isChecked()) {
    ui_->edit_text->setText("Position:");
    ui_->edit_label_1->setText("X:");
    ui_->edit_label_2->setText("Y:");
    ui_->make_bidirectional_button->setVisible(false);
    ui_->reverse_edge_label->setVisible(false);
    ui_->reverse_edge_id->setVisible(false);
  } else {
    ui_->edit_text->setText("Connections:");
    ui_->edit_label_1->setText("Start Node ID:");
    ui_->edit_label_2->setText("End Node ID:");
    ui_->make_bidirectional_button->setVisible(true);
    ui_->reverse_edge_label->setVisible(true);
    ui_->reverse_edge_id->setVisible(true);
  }
}

void RouteTool::on_remove_node_button_toggled(void)
{
  if (ui_->remove_node_button->isChecked()) {
    ui_->remove_reverse_edge_label->setVisible(false);
    ui_->remove_reverse_edge_id->setVisible(false);
    ui_->remove_bidirectional_checkbox->setVisible(false);
  } else {
    ui_->remove_reverse_edge_label->setVisible(true);
    ui_->remove_reverse_edge_id->setVisible(true);
    ui_->remove_bidirectional_checkbox->setVisible(true);
  }
}

void RouteTool::on_make_bidirectional_button_clicked(void)
{
  if (ui_->edit_id->toPlainText() == "" || ui_->edit_field_1->toPlainText() == "" ||
    ui_->edit_field_2->toPlainText() == "")
  {
    return;
  }

  auto start_node = ui_->edit_field_1->toPlainText().toInt();
  auto end_node = ui_->edit_field_2->toPlainText().toInt();

  // Check if reverse edge already exists
  auto reverse_edge = find_reverse_edge(start_node, end_node);
  if (reverse_edge.has_value()) {
    RCLCPP_INFO(
      node_->get_logger(), "Reverse edge already exists with ID %d", reverse_edge.value());
    return;
  }

  // Create the reverse edge with the next available ID
  nav2_route::EdgeCost edge_cost;
  graph_[graph_to_id_map_[end_node]].addEdge(
    edge_cost, &(graph_[graph_to_id_map_[start_node]]),
    next_node_id_);

  if (graph_to_incoming_edges_map_.find(start_node) != graph_to_incoming_edges_map_.end()) {
    graph_to_incoming_edges_map_[start_node].push_back(next_node_id_);
  } else {
    graph_to_incoming_edges_map_[start_node] = std::vector<unsigned int>{next_node_id_};
  }
  edge_to_node_map_[next_node_id_] = end_node;

  RCLCPP_INFO(
    node_->get_logger(), "Created reverse edge %d from node %d to node %d",
    next_node_id_, end_node, start_node);

  // Update the UI to show the new reverse edge ID
  ui_->reverse_edge_id->setText(std::to_string(next_node_id_).c_str());

  next_node_id_++;
  update_route_graph();
}

void RouteTool::update_route_graph(void)
{
  graph_vis_publisher_->publish(nav2_route::utils::toMsg(graph_, "map", node_->now()));
  rebuild_interactive_markers();
}

void RouteTool::rebuild_interactive_markers(void)
{
  if (!im_server_) {
    return;
  }
  im_server_->clear();
  for (const auto & node : graph_) {
    if (node.nodeid == static_cast<unsigned int>(std::numeric_limits<int>::max())) {
      continue;  // Skip deleted nodes
    }

    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "map";
    int_marker.header.stamp = node_->now();
    int_marker.name = std::to_string(node.nodeid);
    int_marker.description = "";
    int_marker.scale = 0.6f;
    int_marker.pose.position.x = node.coords.x;
    int_marker.pose.position.y = node.coords.y;
    int_marker.pose.position.z = 0.0;
    int_marker.pose.orientation.w = 1.0;

    visualization_msgs::msg::InteractiveMarkerControl control;
    // Quaternion for a control whose normal points along +Z (drag in XY plane).
    tf2::Quaternion q;
    q.setRPY(0.0, M_PI_2, 0.0);
    q.normalize();
    control.orientation.w = q.w();
    control.orientation.x = q.x();
    control.orientation.y = q.y();
    control.orientation.z = q.z();
    control.name = "move_xy";
    control.interaction_mode =
      visualization_msgs::msg::InteractiveMarkerControl::MOVE_PLANE;
    control.always_visible = true;

    // Visible disc handle so the user has something to grab.
    visualization_msgs::msg::Marker handle;
    handle.type = visualization_msgs::msg::Marker::CYLINDER;
    handle.scale.x = 0.4;
    handle.scale.y = 0.4;
    handle.scale.z = 0.05;
    handle.color.r = 0.2f;
    handle.color.g = 0.6f;
    handle.color.b = 1.0f;
    handle.color.a = 0.6f;
    control.markers.push_back(handle);

    int_marker.controls.push_back(control);

    im_server_->insert(
      int_marker,
      std::bind(&RouteTool::on_marker_feedback, this, std::placeholders::_1));
  }
  im_server_->applyChanges();
}

void RouteTool::on_marker_feedback(
  const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr & feedback)
{
  using Feedback = visualization_msgs::msg::InteractiveMarkerFeedback;
  if (feedback->event_type != Feedback::POSE_UPDATE &&
    feedback->event_type != Feedback::MOUSE_UP)
  {
    return;
  }
  unsigned int node_id;
  try {
    node_id = static_cast<unsigned int>(std::stoul(feedback->marker_name));
  } catch (const std::exception &) {
    return;
  }
  const float x = static_cast<float>(feedback->pose.position.x);
  const float y = static_cast<float>(feedback->pose.position.y);
  const bool commit = (feedback->event_type == Feedback::MOUSE_UP);

  // Feedback fires on an executor thread; bounce to the Qt/UI thread before
  // touching graph_ and the panel widgets.
  QMetaObject::invokeMethod(
    this,
    [this, node_id, x, y, commit]() {apply_marker_drag(node_id, x, y, commit);},
    Qt::QueuedConnection);
}

void RouteTool::apply_marker_drag(unsigned int node_id, float x, float y, bool commit)
{
  auto it = graph_to_id_map_.find(node_id);
  if (it == graph_to_id_map_.end()) {
    return;
  }
  graph_[it->second].coords.x = x;
  graph_[it->second].coords.y = y;

  // Live preview during drag: just republish the visualization. Avoid calling
  // update_route_graph(), which would clear and reinsert the marker the user
  // is currently dragging and cancel the gesture.
  graph_vis_publisher_->publish(nav2_route::utils::toMsg(graph_, "map", node_->now()));

  if (commit) {
    // Click / drag-release selects the node in the current tab.
    constexpr int kEditTabIndex = 1;
    constexpr int kRemoveTabIndex = 2;
    const int tab = ui_->tabWidget->currentIndex();
    if (tab == kEditTabIndex && ui_->edit_node_button->isChecked()) {
      ui_->edit_id->setText(QString::number(node_id));
      ui_->edit_field_1->setText(QString::number(x));
      ui_->edit_field_2->setText(QString::number(y));
    } else if (tab == kRemoveTabIndex && ui_->remove_node_button->isChecked()) {
      ui_->remove_id->setText(QString::number(node_id));
    }
  }
}

void RouteTool::save(rviz_common::Config config) const
{
  rviz_common::Panel::save(config);
}

void RouteTool::load(const rviz_common::Config & config)
{
  rviz_common::Panel::load(config);
}
}  // namespace nav2_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(nav2_rviz_plugins::RouteTool, rviz_common::Panel)
