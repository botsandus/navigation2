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
#include "nav2_util/raytrace_line_2d.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::VectorObjectLayer, nav2_costmap_2d::Layer)

namespace nav2_costmap_2d
{

namespace
{

/// @brief Write a cost into a cell: overwrite NO_INFORMATION, otherwise keep the max.
inline void writeCost(unsigned char & cell, unsigned char cost)
{
  if (cell == NO_INFORMATION || cost > cell) {
    cell = cost;
  }
}

/// @brief Functor for nav2_util::raytraceLine applying writeCost per cell.
class WriteCostAction
{
public:
  WriteCostAction(unsigned char * costmap, unsigned char cost)
  : costmap_(costmap), cost_(cost) {}
  inline void operator()(unsigned int offset) {writeCost(costmap_[offset], cost_);}

private:
  unsigned char * costmap_;
  unsigned char cost_;
};

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
      if (enabled_) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        has_new_data_ = true;
        buffer_valid_ = false;
      }
      current_ = false;
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
  // Scanline fill in continuous cell coordinates, matching the Vector Object
  // server's rasterisation semantics (half-open Y interval, multiply-then-
  // divide intersection, ceil-based span bounds).
  const std::size_t n = vx.size();
  const int map_w = static_cast<int>(getSizeInCellsX());
  const int map_h = static_cast<int>(getSizeInCellsY());
  const double map_w_d = static_cast<double>(map_w - 1);
  const double map_h_d = static_cast<double>(map_h - 1);
  unsigned char * charmap = getCharMap();

  double y_min_d = std::ceil(*std::min_element(vy.begin(), vy.end()));
  double y_max_d = std::floor(*std::max_element(vy.begin(), vy.end()));
  y_min_d = std::clamp(y_min_d, 0.0, map_h_d);
  y_max_d = std::clamp(y_max_d, 0.0, map_h_d);
  const int y_min = static_cast<int>(y_min_d);
  const int y_max = static_cast<int>(y_max_d);
  if (y_min > y_max) {
    return;  // no visible extent in Y
  }

  struct EdgeInfo
  {
    double y_lo, y_hi, xi, yi, dx, dy;
  };
  std::vector<EdgeInfo> edges;
  edges.reserve(n);
  for (std::size_t i = 0; i < n; i++) {
    const std::size_t j = (i + 1) % n;
    const double y0 = vy[i], y1 = vy[j];
    const double x0 = vx[i], x1 = vx[j];
    if (y0 == y1) {
      continue;  // horizontal edge never contributes an intersection
    }
    EdgeInfo e;
    if (y0 < y1) {
      e.y_lo = y0; e.y_hi = y1; e.xi = x0; e.yi = y0; e.dx = x1 - x0; e.dy = y1 - y0;
    } else {
      e.y_lo = y1; e.y_hi = y0; e.xi = x1; e.yi = y1; e.dx = x0 - x1; e.dy = y0 - y1;
    }
    edges.push_back(e);
  }

  std::vector<double> xs;
  xs.reserve(edges.size());

  for (int y = y_min; y <= y_max; y++) {
    xs.clear();
    for (const auto & e : edges) {
      if (y <= e.y_lo || y > e.y_hi) {
        continue;
      }
      xs.push_back(e.xi + (y - e.yi) * e.dx / e.dy);
    }
    std::sort(xs.begin(), xs.end());

    for (std::size_t k = 0; k + 1 < xs.size(); k += 2) {
      const double a = std::ceil(xs[k]);
      const double b = std::ceil(xs[k + 1]) - 1.0;
      if (b < 0.0 || a > map_w_d) {
        continue;
      }
      const int x_start = static_cast<int>(std::max(a, 0.0));
      const int x_end = static_cast<int>(std::min(b, map_w_d));
      if (x_start > x_end) {
        continue;
      }
      const unsigned int row_offset = static_cast<unsigned int>(y) * getSizeInCellsX();
      for (int x = x_start; x <= x_end; x++) {
        writeCost(charmap[row_offset + static_cast<unsigned int>(x)], cost);
      }
    }
  }
}

void VectorObjectLayer::rasterisePolygonOutline(
  const std::vector<double> & vx, const std::vector<double> & vy, unsigned char cost)
{
  // Draw the polygonal chain segment by segment (vertices clamped to the
  // map). To close an outline, the caller repeats the first vertex.
  const int map_w = static_cast<int>(getSizeInCellsX());
  const int map_h = static_cast<int>(getSizeInCellsY());
  unsigned char * charmap = getCharMap();
  WriteCostAction action(charmap, cost);

  auto to_cell = [](double v, int max_cell) -> unsigned int {
      return static_cast<unsigned int>(
        std::clamp(std::lround(v), 0L, static_cast<int64_t>(max_cell)));
    };

  for (std::size_t i = 0; i + 1 < vx.size(); i++) {
    nav2_util::raytraceLine(
      action,
      to_cell(vx[i], map_w - 1), to_cell(vy[i], map_h - 1),
      to_cell(vx[i + 1], map_w - 1), to_cell(vy[i + 1], map_h - 1),
      getSizeInCellsX());
  }
}

void VectorObjectLayer::rasteriseCircle(
  double cx, double cy, double radius_cells, bool fill, unsigned char cost)
{
  const int map_w = static_cast<int>(getSizeInCellsX());
  const int map_h = static_cast<int>(getSizeInCellsY());
  unsigned char * charmap = getCharMap();

  if (fill) {
    // Row-span fill in continuous cell coordinates
    const int y0 = std::max(static_cast<int>(std::ceil(cy - radius_cells)), 0);
    const int y1 = std::min(static_cast<int>(std::floor(cy + radius_cells)), map_h - 1);
    for (int y = y0; y <= y1; y++) {
      const double t = radius_cells * radius_cells - (y - cy) * (y - cy);
      if (t < 0.0) {
        continue;
      }
      const double dx = std::sqrt(t);
      const int x0 = std::max(static_cast<int>(std::ceil(cx - dx)), 0);
      const int x1 = std::min(static_cast<int>(std::floor(cx + dx)), map_w - 1);
      const unsigned int row_offset = static_cast<unsigned int>(y) * getSizeInCellsX();
      for (int x = x0; x <= x1; x++) {
        writeCost(charmap[row_offset + static_cast<unsigned int>(x)], cost);
      }
    }
  } else {
    // Approximate the outline with a 36-segment closed chain
    constexpr int kSegments = 36;
    std::vector<double> vx(kSegments + 1), vy(kSegments + 1);
    for (int s = 0; s <= kSegments; s++) {
      const double angle = 2.0 * M_PI * s / kSegments;
      vx[s] = cx + radius_cells * std::cos(angle);
      vy[s] = cy + radius_cells * std::sin(angle);
    }
    rasterisePolygonOutline(vx, vy, cost);
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

  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
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
