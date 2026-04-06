// Copyright (c) 2026 Dexory
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

#include "nav2_costmap_2d/topic_polygon_layer.hpp"

#include <tf2/exceptions.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_ros_common/qos_profiles.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::TopicPolygonLayer, nav2_costmap_2d::Layer)

namespace nav2_costmap_2d
{

void TopicPolygonLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("TopicPolygonLayer: failed to lock parent node");
  }

  enabled_ = node->declare_or_get_parameter(name_ + "." + "enabled", true);

  setDefaultValue(nav2_costmap_2d::NO_INFORMATION);

  auto polygons_topic = node->declare_or_get_parameter(
    name_ + "." + "polygons_topic", std::string{"vo_polygons"});
  polygons_topic = joinWithParentNamespace(polygons_topic);

  data_frame_ = node->declare_or_get_parameter(
    name_ + "." + "polygons_frame", std::string{"map"});

  fill_polygons_ = node->declare_or_get_parameter(
    name_ + "." + "fill_polygons", true);

  rclcpp::QoS map_qos = nav2::qos::LatchedSubscriptionQoS(1);

  RCLCPP_INFO(
    logger_,
    "TopicPolygonLayer(%s): subscribing to '%s' in frame '%s'",
    name_.c_str(), polygons_topic.c_str(), data_frame_.c_str());

  polygons_sub_ = node->create_subscription<nav2_msgs::msg::PolygonObjects>(
    polygons_topic,
    std::bind(&TopicPolygonLayer::polygonsCallback, this, std::placeholders::_1),
    map_qos);

  setCurrent(true);
  matchSize();
}

void TopicPolygonLayer::activate()
{
  auto node = node_.lock();
  post_set_params_handler_ = node->add_post_set_parameters_callback(
    std::bind(
      &TopicPolygonLayer::updateParameterUpdatesCallback,
      this, std::placeholders::_1));
  on_set_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &TopicPolygonLayer::validateParameterUpdatesCallback,
      this, std::placeholders::_1));
}

void TopicPolygonLayer::deactivate()
{
  auto node = node_.lock();
  if (post_set_params_handler_ && node) {
    node->remove_post_set_parameters_callback(post_set_params_handler_.get());
  }
  post_set_params_handler_.reset();
  if (on_set_params_handler_ && node) {
    node->remove_on_set_parameters_callback(on_set_params_handler_.get());
  }
  on_set_params_handler_.reset();
}

void TopicPolygonLayer::polygonsCallback(
  const nav2_msgs::msg::PolygonObjects::ConstSharedPtr & msg)
{
  std::vector<geometry_msgs::msg::Polygon> polygons;
  polygons.reserve(msg->polygons.size());
  for (const auto & obj : msg->polygons) {
    geometry_msgs::msg::Polygon poly;
    poly.points = obj.points;
    polygons.push_back(std::move(poly));
  }

  if (!msg->header.frame_id.empty()) {
    data_frame_ = msg->header.frame_id;
  }

  // Thread-safe polygon update
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (arePolygonVectorsEqual(polygons, current_polygons_)) {
    return;
  }
  current_polygons_ = std::move(polygons);
  has_new_data_ = true;
  buffer_valid_ = false;
  setCurrent(false);
}

rcl_interfaces::msg::SetParametersResult TopicPolygonLayer::validateParameterUpdatesCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    const auto & param_name = param.get_name();
    if (param_name.find(name_ + ".") != 0) {
      continue;
    }

    if (param_name == name_ + "." + "polygons_topic" ||
      param_name == name_ + "." + "polygons_frame")
    {
      RCLCPP_WARN(
        logger_, "%s is not a dynamic parameter "
        "cannot be changed while running. Rejecting parameter update.", param_name.c_str());
      result.successful = false;
    }
  }
  return result;
}

void TopicPolygonLayer::updateParameterUpdatesCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  for (const auto & param : parameters) {
    const auto & param_type = param.get_type();
    const auto & param_name = param.get_name();
    if (param_name.find(name_ + ".") != 0) {
      continue;
    }

    if (param_type == rclcpp::ParameterType::PARAMETER_BOOL) {
      if (param_name == name_ + "." + "enabled" && enabled_ != param.as_bool()) {
        enabled_ = param.as_bool();
        if (enabled_) {
          has_new_data_ = true;
          buffer_valid_ = false;
        }
        setCurrent(false);
      } else if (param_name == name_ + "." + "fill_polygons") {
        fill_polygons_ = param.as_bool();
        has_new_data_ = true;
        buffer_valid_ = false;
        setCurrent(false);
      }
    }
  }
}

void TopicPolygonLayer::matchSize()
{
  auto * master = layered_costmap_->getCostmap();

  if (getSizeInCellsX() == master->getSizeInCellsX() &&
    getSizeInCellsY() == master->getSizeInCellsY() &&
    std::abs(getResolution() - master->getResolution()) < 1e-9 &&
    std::abs(getOriginX() - master->getOriginX()) < 1e-9 &&
    std::abs(getOriginY() - master->getOriginY()) < 1e-9)
  {
    return;
  }

  resizeMap(
    master->getSizeInCellsX(),
    master->getSizeInCellsY(),
    master->getResolution(),
    master->getOriginX(),
    master->getOriginY());
  buffer_valid_ = false;
}

bool TopicPolygonLayer::arePolygonVectorsEqual(
  const std::vector<geometry_msgs::msg::Polygon> & a,
  const std::vector<geometry_msgs::msg::Polygon> & b)
{
  if (a.size() != b.size()) {
    return false;
  }
  for (size_t i = 0; i < a.size(); ++i) {
    if (a[i].points.size() != b[i].points.size()) {
      return false;
    }
    for (size_t j = 0; j < a[i].points.size(); ++j) {
      if (std::abs(a[i].points[j].x - b[i].points[j].x) > 1e-6f ||
        std::abs(a[i].points[j].y - b[i].points[j].y) > 1e-6f)
      {
        return false;
      }
    }
  }
  return true;
}

std::vector<geometry_msgs::msg::Polygon> TopicPolygonLayer::transformPolygons(
  const std::vector<geometry_msgs::msg::Polygon> & polygons,
  const geometry_msgs::msg::TransformStamped & tf_stamped)
{
  const auto & t = tf_stamped.transform.translation;
  const auto & q = tf_stamped.transform.rotation;
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  const double tx = t.x;
  const double ty = t.y;

  std::vector<geometry_msgs::msg::Polygon> result;
  result.reserve(polygons.size());
  for (const auto & poly : polygons) {
    geometry_msgs::msg::Polygon out;
    out.points.reserve(poly.points.size());
    for (const auto & pt : poly.points) {
      geometry_msgs::msg::Point32 p;
      p.x = static_cast<float>(cos_yaw * pt.x - sin_yaw * pt.y + tx);
      p.y = static_cast<float>(sin_yaw * pt.x + cos_yaw * pt.y + ty);
      p.z = 0.0f;
      out.points.push_back(p);
    }
    result.push_back(std::move(out));
  }
  return result;
}

void TopicPolygonLayer::rasterisePolygons(
  const std::vector<geometry_msgs::msg::Polygon> & polygons)
{
  resetMaps();

  const unsigned int size_x = getSizeInCellsX();
  const unsigned int size_y = getSizeInCellsY();
  if (size_x == 0 || size_y == 0) {
    return;
  }

  const double origin_x = getOriginX();
  const double origin_y = getOriginY();
  const double inv_res = 1.0 / getResolution();
  unsigned char * charmap = getCharMap();

  cv::Mat grid(static_cast<int>(size_y), static_cast<int>(size_x), CV_8UC1, charmap);

  std::vector<cv::Point> pts;

  for (const auto & poly : polygons) {
    if (poly.points.size() < 3) {
      continue;
    }

    pts.clear();
    pts.reserve(poly.points.size());
    for (const auto & pt : poly.points) {
      int cx = static_cast<int>(
        std::round((static_cast<double>(pt.x) - origin_x) * inv_res - 0.5));
      int cy = static_cast<int>(
        std::round((static_cast<double>(pt.y) - origin_y) * inv_res - 0.5));
      pts.emplace_back(cx, cy);
    }

    if (fill_polygons_) {
      const cv::Point * ppt[1] = {pts.data()};
      int npt[1] = {static_cast<int>(pts.size())};
      cv::fillPoly(grid, ppt, npt, 1, cv::Scalar(nav2_costmap_2d::LETHAL_OBSTACLE));
    } else {
      cv::polylines(
        grid, pts, /*isClosed=*/true,
        cv::Scalar(nav2_costmap_2d::LETHAL_OBSTACLE));
    }
  }
}

void TopicPolygonLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  matchSize();
  useExtraBounds(min_x, min_y, max_x, max_y);

  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  if (current_polygons_.empty() && !has_new_data_) {
    return;
  }

  if (!has_new_data_ && buffer_valid_ && !layered_costmap_->isRolling()) {
    return;
  }

  std::string global_frame = layered_costmap_->getGlobalFrameID();
  bool needs_transform = (data_frame_ != global_frame);
  geometry_msgs::msg::TransformStamped tf_stamped;

  if (needs_transform) {
    try {
      tf_stamped = tf_->lookupTransform(global_frame, data_frame_, tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000,
        "TopicPolygonLayer(%s): could not transform '%s' -> '%s': %s",
        name_.c_str(), data_frame_.c_str(), global_frame.c_str(), ex.what());
      return;
    }
  }

  auto transformed = needs_transform ?
    transformPolygons(current_polygons_, tf_stamped) :
    std::vector<geometry_msgs::msg::Polygon>{};
  const auto & costmap_polygons = needs_transform ? transformed : current_polygons_;

  double report_min_x = std::numeric_limits<double>::max();
  double report_min_y = std::numeric_limits<double>::max();
  double report_max_x = std::numeric_limits<double>::lowest();
  double report_max_y = std::numeric_limits<double>::lowest();
  for (const auto & poly : costmap_polygons) {
    for (const auto & pt : poly.points) {
      report_min_x = std::min(report_min_x, static_cast<double>(pt.x));
      report_min_y = std::min(report_min_y, static_cast<double>(pt.y));
      report_max_x = std::max(report_max_x, static_cast<double>(pt.x));
      report_max_y = std::max(report_max_y, static_cast<double>(pt.y));
    }
  }

  if (has_new_data_) {
    *min_x = std::min(*min_x, prev_min_x_);
    *min_y = std::min(*min_y, prev_min_y_);
    *max_x = std::max(*max_x, prev_max_x_);
    *max_y = std::max(*max_y, prev_max_y_);

    prev_min_x_ = report_min_x;
    prev_min_y_ = report_min_y;
    prev_max_x_ = report_max_x;
    prev_max_y_ = report_max_y;
    has_new_data_ = false;
  }

  if (!buffer_valid_ || layered_costmap_->isRolling()) {
    *min_x = std::min(*min_x, report_min_x);
    *min_y = std::min(*min_y, report_min_y);
    *max_x = std::max(*max_x, report_max_x);
    *max_y = std::max(*max_y, report_max_y);

    rasterisePolygons(costmap_polygons);
    buffer_valid_ = true;
  }
}

void TopicPolygonLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  setCurrent(true);
}

void TopicPolygonLayer::reset()
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  has_new_data_ = true;
  buffer_valid_ = false;
  setCurrent(false);
  resetMaps();
}

bool TopicPolygonLayer::isClearable()
{
  return true;
}

}  // namespace nav2_costmap_2d
