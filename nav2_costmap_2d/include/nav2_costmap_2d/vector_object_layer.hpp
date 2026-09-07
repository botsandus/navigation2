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

#ifndef NAV2_COSTMAP_2D__VECTOR_OBJECT_LAYER_HPP_
#define NAV2_COSTMAP_2D__VECTOR_OBJECT_LAYER_HPP_

#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_msgs/msg/vector_objects.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_costmap_2d
{

/**
 * @class VectorObjectLayer
 * @brief Costmap layer that rasterises vector objects (polygons and circles)
 * directly into the costmap, in-process.
 *
 * Unlike consuming rasterised vector objects through a StaticLayer or a
 * KeepoutFilter mask, this layer:
 *   - reports only the shapes' bounding box (plus the previous bounding box
 *     for stale-cell clearing) as its update region, so downstream layers
 *     (notably InflationLayer) do not reprocess the whole map on changes
 *   - composes with InflationLayer like any other obstacle-producing plugin
 *   - costs nothing at steady state on non-rolling costmaps
 *
 * Shapes are fed either:
 *   - via the 'shapes_topic' parameter (nav2_msgs/VectorObjects, typically
 *     transient-local), or
 *   - by a subclass calling setVectorObjects() from its own data source.
 *
 * Shape semantics match the Vector Object server: a polygon with
 * closed = true is filled; closed = false is drawn as a polygonal chain
 * (repeat the first vertex to close an outline). Shape values are
 * OccupancyGrid values: -1 maps to NO_INFORMATION (transparent), 0..100
 * scale linearly up to LETHAL_OBSTACLE. An empty shape frame_id means the
 * costmap's global frame.
 */
class VectorObjectLayer : public CostmapLayer
{
public:
  VectorObjectLayer() = default;
  ~VectorObjectLayer() override = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override;
  void matchSize() override;

protected:
  /// @brief Replace the current shape set. Thread-safe; for subclasses with
  /// their own data sources. No-op when the shape set is unchanged.
  void setVectorObjects(
    std::vector<nav2_msgs::msg::PolygonObject> polygons,
    std::vector<nav2_msgs::msg::CircleObject> circles);

  /// @brief Hook for subclass-specific dynamic parameter changes, called
  /// after the base has handled 'enabled'.
  virtual void onDynamicParametersChange(
    const std::vector<rclcpp::Parameter> & /*parameters*/) {}

private:
  void shapesCallback(const nav2_msgs::msg::VectorObjects::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  /// @brief Convert an OccupancyGrid shape value to a costmap cost.
  static unsigned char valueToCost(int8_t value);

  /// @brief Look up transforms for every distinct shape frame.
  /// @return false if any lookup failed (caller should retry next cycle).
  bool lookupShapeTransforms(
    std::vector<geometry_msgs::msg::TransformStamped> & polygon_tfs,
    std::vector<geometry_msgs::msg::TransformStamped> & circle_tfs);

  /// @brief Rasterise all shapes into the internal buffer and grow the given
  /// bounding box to the shapes' extent (in the costmap global frame).
  /// Called with data_mutex_ held and transforms resolved.
  void rasteriseShapes(
    const std::vector<geometry_msgs::msg::TransformStamped> & polygon_tfs,
    const std::vector<geometry_msgs::msg::TransformStamped> & circle_tfs,
    double & bbox_min_x, double & bbox_min_y, double & bbox_max_x, double & bbox_max_y);

  void rasterisePolygonFilled(const std::vector<double> & vx, const std::vector<double> & vy,
    unsigned char cost);
  void rasterisePolygonOutline(const std::vector<double> & vx, const std::vector<double> & vy,
    unsigned char cost);
  void rasteriseCircle(double cx, double cy, double radius_cells, bool fill, unsigned char cost);

  nav2::Subscription<nav2_msgs::msg::VectorObjects>::SharedPtr shapes_sub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Shape state (protected by data_mutex_)
  std::mutex data_mutex_;
  std::vector<nav2_msgs::msg::PolygonObject> polygons_;
  std::vector<nav2_msgs::msg::CircleObject> circles_;
  bool has_new_data_{false};
  bool buffer_valid_{false};

  // Previous reported bounding box (costmap frame) for stale-cell clearing.
  // Degenerate initial bounds so the first data does not include (0,0).
  double prev_min_x_{std::numeric_limits<double>::max()};
  double prev_min_y_{std::numeric_limits<double>::max()};
  double prev_max_x_{std::numeric_limits<double>::lowest()};
  double prev_max_y_{std::numeric_limits<double>::lowest()};
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__VECTOR_OBJECT_LAYER_HPP_
