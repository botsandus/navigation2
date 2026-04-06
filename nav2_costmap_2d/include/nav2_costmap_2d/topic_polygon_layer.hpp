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

#ifndef NAV2_COSTMAP_2D__TOPIC_POLYGON_LAYER_HPP_
#define NAV2_COSTMAP_2D__TOPIC_POLYGON_LAYER_HPP_

#include <limits>
#include <string>
#include <vector>

#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_msgs/msg/polygon_objects.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_costmap_2d
{

/**
 * @class TopicPolygonLayer
 * @brief Costmap layer that subscribes to a PolygonObjects topic and
 * rasterises the polygons directly into the costmap buffer using
 * OpenCV's cv::fillPoly / cv::polylines.
 */
class TopicPolygonLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  TopicPolygonLayer() = default;
  ~TopicPolygonLayer() override = default;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override;
  void matchSize() override;

private:
  void polygonsCallback(const nav2_msgs::msg::PolygonObjects::ConstSharedPtr & msg);

  /**
   * @brief Validate incoming parameter updates before applying them.
   */
  rcl_interfaces::msg::SetParametersResult validateParameterUpdatesCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  /**
   * @brief Apply validated parameter updates.
   */
  void updateParameterUpdatesCallback(const std::vector<rclcpp::Parameter> & parameters);

  /// @brief Rasterise already-transformed polygons into the internal costmap buffer.
  void rasterisePolygons(const std::vector<geometry_msgs::msg::Polygon> & polygons);

  /// @brief Transform a set of polygons using the given TransformStamped.
  static std::vector<geometry_msgs::msg::Polygon> transformPolygons(
    const std::vector<geometry_msgs::msg::Polygon> & polygons,
    const geometry_msgs::msg::TransformStamped & tf_stamped);

  /// @brief Check geometric equality of two polygon vectors.
  static bool arePolygonVectorsEqual(
    const std::vector<geometry_msgs::msg::Polygon> & a,
    const std::vector<geometry_msgs::msg::Polygon> & b);

  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_params_handler_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handler_;
  nav2::Subscription<nav2_msgs::msg::PolygonObjects>::SharedPtr polygons_sub_;

  bool fill_polygons_{true};
  std::string data_frame_{"map"};

  // Polygon state (protected by getMutex())
  std::vector<geometry_msgs::msg::Polygon> current_polygons_;
  bool has_new_data_{false};
  bool buffer_valid_{false};

  // Previous bounding box for stale-cell clearing.
  double prev_min_x_{std::numeric_limits<double>::max()};
  double prev_min_y_{std::numeric_limits<double>::max()};
  double prev_max_x_{std::numeric_limits<double>::lowest()};
  double prev_max_y_{std::numeric_limits<double>::lowest()};
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__TOPIC_POLYGON_LAYER_HPP_
