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

#include "nav2_costmap_2d/vector_object_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/polygon_fill_2d.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::VectorObjectLayer, nav2_costmap_2d::Layer)

namespace nav2_costmap_2d
{

namespace
{

struct Transform2D
{
  double cos_yaw;
  double sin_yaw;
  double tx;
  double ty;

  explicit Transform2D(const geometry_msgs::msg::TransformStamped & tf_stamped)
  {
    const auto & q = tf_stamped.transform.rotation;
    const double yaw = std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    cos_yaw = std::cos(yaw);
    sin_yaw = std::sin(yaw);
    tx = tf_stamped.transform.translation.x;
    ty = tf_stamped.transform.translation.y;
  }

  inline void apply(double x, double y, double & out_x, double & out_y) const
  {
    out_x = cos_yaw * x - sin_yaw * y + tx;
    out_y = sin_yaw * x + cos_yaw * y + ty;
  }
};

geometry_msgs::msg::TransformStamped identityTransform()
{
  geometry_msgs::msg::TransformStamped tf_stamped;
  tf_stamped.transform.rotation.w = 1.0;
  return tf_stamped;
}

}  // namespace

void VectorObjectLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("VectorObjectLayer: failed to lock parent node");
  }

  enabled_ = node->declare_or_get_parameter(getFullName("enabled"), true);
  const auto shapes_topic = node->declare_or_get_parameter(
    getFullName("shapes_topic"), std::string{});
  const bool transient_local = node->declare_or_get_parameter(
    getFullName("shapes_topic_transient_local"), true);
  combination_method_ = combination_method_from_int(
    node->declare_or_get_parameter(
      getFullName("combination_method"),
      static_cast<int>(CombinationMethod::Max)));

  param_callback_handle_ = node->add_on_set_parameters_callback(
    std::bind(&VectorObjectLayer::dynamicParametersCallback, this, std::placeholders::_1));

  if (!shapes_topic.empty()) {
    rclcpp::QoS qos(1);
    qos.reliable();
    if (transient_local) {
      qos.transient_local();
    }
    RCLCPP_INFO(
      logger_, "VectorObjectLayer(%s): subscribing to shapes topic '%s'",
      name_.c_str(), shapes_topic.c_str());
    shapes_sub_ = node->create_subscription<nav2_msgs::msg::VectorObjects>(
      shapes_topic,
      std::bind(&VectorObjectLayer::shapesCallback, this, std::placeholders::_1),
      qos);
  } else {
    RCLCPP_WARN(
      logger_,
      "VectorObjectLayer(%s): no 'shapes_topic' configured — the layer stays "
      "empty unless a subclass feeds it via setVectorObjects()",
      name_.c_str());
  }

  // Non-shape cells must be NO_INFORMATION so that updateWithMax skips them,
  // keeping the layer transparent where no shapes exist.
  setDefaultValue(NO_INFORMATION);

  current_ = true;
  matchSize();
}

void VectorObjectLayer::shapesCallback(const nav2_msgs::msg::VectorObjects::SharedPtr msg)
{
  setVectorObjects(msg->polygons, msg->circles);
}

void VectorObjectLayer::setVectorObjects(
  std::vector<nav2_msgs::msg::PolygonObject> polygons,
  std::vector<nav2_msgs::msg::CircleObject> circles)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  if (polygons == polygons_ && circles == circles_) {
    return;
  }

  polygons_ = std::move(polygons);
  circles_ = std::move(circles);
  has_new_data_ = true;
  buffer_valid_ = false;
  current_ = false;
}

rcl_interfaces::msg::SetParametersResult VectorObjectLayer::dynamicParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & param : parameters) {
    if (param.get_name() == getFullName("enabled")) {
      enabled_ = param.as_bool();
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (enabled_) {
        has_new_data_ = true;
        buffer_valid_ = false;
        pending_disable_clear_ = false;
      } else {
        pending_disable_clear_ = true;
      }
      current_ = false;
    } else if (param.get_name() == getFullName("combination_method")) {
      combination_method_ = combination_method_from_int(param.as_int());
    }
  }

  onDynamicParametersChange(parameters);

  return result;
}

void VectorObjectLayer::matchSize()
{
  auto * master = layered_costmap_->getCostmap();

  // Only resize (which wipes the buffer) when geometry actually changed
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

unsigned char VectorObjectLayer::valueToCost(int8_t value)
{
  if (value < 0) {
    return NO_INFORMATION;
  }
  const int clamped = std::min<int>(value, 100);
  if (clamped == 100) {
    return LETHAL_OBSTACLE;
  }
  return static_cast<unsigned char>(
    std::lround(static_cast<double>(clamped) / 100.0 * LETHAL_OBSTACLE));
}

bool VectorObjectLayer::lookupShapeTransforms(
  std::vector<geometry_msgs::msg::TransformStamped> & polygon_tfs,
  std::vector<geometry_msgs::msg::TransformStamped> & circle_tfs)
{
  const std::string global_frame = layered_costmap_->getGlobalFrameID();

  auto resolve = [&](const std::string & frame,
    geometry_msgs::msg::TransformStamped & tf_stamped) -> bool {
      if (frame.empty() || frame == global_frame) {
        tf_stamped = identityTransform();
        return true;
      }
      try {
        tf_stamped = tf_->lookupTransform(global_frame, frame, tf2::TimePointZero);
      } catch (tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          logger_, *clock_, 5000,
          "VectorObjectLayer(%s): could not transform '%s' -> '%s': %s",
          name_.c_str(), frame.c_str(), global_frame.c_str(), ex.what());
        return false;
      }
      return true;
    };

  polygon_tfs.resize(polygons_.size());
  for (size_t i = 0; i < polygons_.size(); ++i) {
    if (!resolve(polygons_[i].header.frame_id, polygon_tfs[i])) {
      return false;
    }
  }
  circle_tfs.resize(circles_.size());
  for (size_t i = 0; i < circles_.size(); ++i) {
    if (!resolve(circles_[i].header.frame_id, circle_tfs[i])) {
      return false;
    }
  }
  return true;
}

void VectorObjectLayer::rasterisePolygonFilled(
  const std::vector<double> & vx, const std::vector<double> & vy, unsigned char cost)
{
  unsigned char * charmap = getCharMap();
  const unsigned int size_x = getSizeInCellsX();
  nav2_util::fillPolygon(
    vx, vy, size_x, getSizeInCellsY(),
    [charmap, size_x, cost](unsigned int y, unsigned int x_start, unsigned int x_end) {
      unsigned char * row = charmap + static_cast<size_t>(y) * size_x;
      for (unsigned int x = x_start; x <= x_end; ++x) {
        row[x] = std::max(row[x] == NO_INFORMATION ? FREE_SPACE : row[x], cost);
      }
    });
}

void VectorObjectLayer::rasterisePolygonOutline(
  const std::vector<double> & vx, const std::vector<double> & vy, unsigned char cost)
{
  // 1-cell-wide polygonal chain; to close an outline, the caller repeats
  // the first vertex.
  unsigned char * charmap = getCharMap();
  const unsigned int size_x = getSizeInCellsX();
  const unsigned int size_y = getSizeInCellsY();
  auto write_cell = [charmap, size_x, cost](unsigned int y, unsigned int x) {
      unsigned char & cell = charmap[static_cast<size_t>(y) * size_x + x];
      cell = std::max(cell == NO_INFORMATION ? FREE_SPACE : cell, cost);
    };
  for (std::size_t i = 0; i + 1 < vx.size(); ++i) {
    nav2_util::forEachLineCell(vx[i], vy[i], vx[i + 1], vy[i + 1], size_x, size_y, write_cell);
  }
}

void VectorObjectLayer::rasteriseCircle(
  double cx, double cy, double radius_cells, bool fill, unsigned char cost)
{
  unsigned char * charmap = getCharMap();
  const unsigned int size_x = getSizeInCellsX();
  const unsigned int size_y = getSizeInCellsY();

  if (fill) {
    nav2_util::fillCircle(
      cx, cy, radius_cells, size_x, size_y,
      [charmap, size_x, cost](unsigned int y, unsigned int x_start, unsigned int x_end) {
        unsigned char * row = charmap + static_cast<size_t>(y) * size_x;
        for (unsigned int x = x_start; x <= x_end; ++x) {
          row[x] = std::max(row[x] == NO_INFORMATION ? FREE_SPACE : row[x], cost);
        }
      });
    return;
  }

  // Outline: draw the circle as a polygonal chain with ~1-cell-long segments
  auto write_cell = [charmap, size_x, cost](unsigned int y, unsigned int x) {
      unsigned char & cell = charmap[static_cast<size_t>(y) * size_x + x];
      cell = std::max(cell == NO_INFORMATION ? FREE_SPACE : cell, cost);
    };
  const int segments = std::max(16, static_cast<int>(std::ceil(2.0 * M_PI * radius_cells)));
  double px = cx + radius_cells;
  double py = cy;
  for (int s = 1; s <= segments; ++s) {
    const double angle = 2.0 * M_PI * s / segments;
    const double nx = cx + radius_cells * std::cos(angle);
    const double ny = cy + radius_cells * std::sin(angle);
    nav2_util::forEachLineCell(px, py, nx, ny, size_x, size_y, write_cell);
    px = nx;
    py = ny;
  }
}

void VectorObjectLayer::rasteriseShapes(
  const std::vector<geometry_msgs::msg::TransformStamped> & polygon_tfs,
  const std::vector<geometry_msgs::msg::TransformStamped> & circle_tfs,
  double & bbox_min_x, double & bbox_min_y, double & bbox_max_x, double & bbox_max_y)
{
  resetMaps();

  const unsigned int size_x = getSizeInCellsX();
  const unsigned int size_y = getSizeInCellsY();
  if (size_x == 0 || size_y == 0) {
    return;
  }

  const double origin_x = getOriginX();
  const double origin_y = getOriginY();
  const double res = getResolution();

  std::vector<double> vx, vy;

  for (size_t i = 0; i < polygons_.size(); ++i) {
    const auto & poly = polygons_[i];
    const unsigned char cost = valueToCost(poly.value);
    if (cost == NO_INFORMATION || poly.points.size() < 2) {
      continue;
    }

    const Transform2D tf2d(polygon_tfs[i]);
    vx.resize(poly.points.size());
    vy.resize(poly.points.size());
    bool finite = true;
    for (size_t p = 0; p < poly.points.size(); ++p) {
      double wx, wy;
      tf2d.apply(poly.points[p].x, poly.points[p].y, wx, wy);
      if (!std::isfinite(wx) || !std::isfinite(wy)) {
        finite = false;
        break;
      }
      bbox_min_x = std::min(bbox_min_x, wx);
      bbox_min_y = std::min(bbox_min_y, wy);
      bbox_max_x = std::max(bbox_max_x, wx);
      bbox_max_y = std::max(bbox_max_y, wy);
      // Continuous cell coordinates: matches the VO server's rasterisation
      vx[p] = (wx - origin_x) / res - 0.5;
      vy[p] = (wy - origin_y) / res - 0.5;
    }
    if (!finite) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000,
        "VectorObjectLayer(%s): polygon has non-finite vertex after TF, skipping",
        name_.c_str());
      continue;
    }

    if (poly.closed && poly.points.size() >= 3) {
      rasterisePolygonFilled(vx, vy, cost);
    } else {
      rasterisePolygonOutline(vx, vy, cost);
    }
  }

  for (size_t i = 0; i < circles_.size(); ++i) {
    const auto & circle = circles_[i];
    const unsigned char cost = valueToCost(circle.value);
    if (cost == NO_INFORMATION || circle.radius <= 0.0F) {
      continue;
    }

    const Transform2D tf2d(circle_tfs[i]);
    double wx, wy;
    tf2d.apply(circle.center.x, circle.center.y, wx, wy);
    if (!std::isfinite(wx) || !std::isfinite(wy)) {
      continue;
    }
    const double r = circle.radius;
    bbox_min_x = std::min(bbox_min_x, wx - r);
    bbox_min_y = std::min(bbox_min_y, wy - r);
    bbox_max_x = std::max(bbox_max_x, wx + r);
    bbox_max_y = std::max(bbox_max_y, wy + r);

    rasteriseCircle(
      (wx - origin_x) / res - 0.5,
      (wy - origin_y) / res - 0.5,
      r / res, circle.fill, cost);
  }
}

void VectorObjectLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    // Report the previous bbox once so cells written before disabling are
    // cleared from the master (which only resets within the update region)
    if (pending_disable_clear_) {
      *min_x = std::min(*min_x, prev_min_x_);
      *min_y = std::min(*min_y, prev_min_y_);
      *max_x = std::max(*max_x, prev_max_x_);
      *max_y = std::max(*max_y, prev_max_y_);
      prev_min_x_ = std::numeric_limits<double>::max();
      prev_min_y_ = std::numeric_limits<double>::max();
      prev_max_x_ = std::numeric_limits<double>::lowest();
      prev_max_y_ = std::numeric_limits<double>::lowest();
      pending_disable_clear_ = false;
    }
    return;
  }

  // Sync geometry with the master: required for rolling costmaps (origin
  // shifts every cycle) and on resize events (e.g. when the map arrives)
  matchSize();

  useExtraBounds(min_x, min_y, max_x, max_y);

  std::lock_guard<std::mutex> lock(data_mutex_);

  if (polygons_.empty() && circles_.empty() && !has_new_data_) {
    return;
  }

  // On a non-rolling costmap with a valid buffer and no new data, cells
  // previously written via updateWithMax persist in the master (it only
  // resets within the combined update region). Skip all work.
  if (!has_new_data_ && buffer_valid_ && !layered_costmap_->isRolling()) {
    return;
  }

  std::vector<geometry_msgs::msg::TransformStamped> polygon_tfs, circle_tfs;
  if (!lookupShapeTransforms(polygon_tfs, circle_tfs)) {
    return;  // retry next cycle: has_new_data_/buffer_valid_ unchanged
  }

  double bbox_min_x = std::numeric_limits<double>::max();
  double bbox_min_y = std::numeric_limits<double>::max();
  double bbox_max_x = std::numeric_limits<double>::lowest();
  double bbox_max_y = std::numeric_limits<double>::lowest();

  // Re-rasterise when invalidated (new data, geometry change, reset) or on
  // rolling costmaps where the origin shifts and cells never persist.
  // Reporting only the shapes' bbox keeps the combined update region (and
  // therefore downstream layers like InflationLayer) small.
  if (!buffer_valid_ || layered_costmap_->isRolling()) {
    rasteriseShapes(polygon_tfs, circle_tfs, bbox_min_x, bbox_min_y, bbox_max_x, bbox_max_y);
    buffer_valid_ = true;

    *min_x = std::min(*min_x, bbox_min_x);
    *min_y = std::min(*min_y, bbox_min_y);
    *max_x = std::max(*max_x, bbox_max_x);
    *max_y = std::max(*max_y, bbox_max_y);
  }

  // On new data, also report the previous bbox so stale cells are cleared
  if (has_new_data_) {
    *min_x = std::min(*min_x, prev_min_x_);
    *min_y = std::min(*min_y, prev_min_y_);
    *max_x = std::max(*max_x, prev_max_x_);
    *max_y = std::max(*max_y, prev_max_y_);

    prev_min_x_ = bbox_min_x;
    prev_min_y_ = bbox_min_y;
    prev_max_x_ = bbox_max_x;
    prev_max_y_ = bbox_max_y;
    has_new_data_ = false;
  }
}

void VectorObjectLayer::updateCosts(
  Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }

  switch (combination_method_) {
    case CombinationMethod::Overwrite:
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case CombinationMethod::Max:
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    case CombinationMethod::MaxWithoutUnknownOverwrite:
      updateWithMaxWithoutUnknownOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:  // Nothing
      break;
  }
  current_ = true;
}

void VectorObjectLayer::reset()
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  // Keep the shape set intact: only invalidate the rasterised buffer so
  // shapes are re-drawn on the next updateBounds() cycle
  has_new_data_ = true;
  buffer_valid_ = false;
  current_ = false;
  resetMaps();
}

bool VectorObjectLayer::isClearable()
{
  return true;
}

}  // namespace nav2_costmap_2d
