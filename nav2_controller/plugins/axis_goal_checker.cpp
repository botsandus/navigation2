// Copyright (c) 2025 Dexory
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

#include <memory>
#include <string>
#include <limits>
#include <vector>

#include "angles/angles.h"
#include "nav2_controller/plugins/axis_goal_checker.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace nav2_controller
{

AxisGoalChecker::AxisGoalChecker()
: along_path_tolerance_(0.25), cross_track_tolerance_(0.25),
  path_length_tolerance_(1.0), direction_estimation_distance_(0.15), is_overshoot_valid_(false)
{
}

AxisGoalChecker::~AxisGoalChecker()
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

void AxisGoalChecker::initialize(
  const nav2::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>/*costmap_ros*/)
{
  plugin_name_ = plugin_name;
  node_ = parent;
  auto node = node_.lock();
  logger_ = node->get_logger();

  along_path_tolerance_ = node->declare_or_get_parameter(
    plugin_name + ".along_path_tolerance", 0.25);
  cross_track_tolerance_ = node->declare_or_get_parameter(
    plugin_name + ".cross_track_tolerance", 0.25);
  path_length_tolerance_ = node->declare_or_get_parameter(
    plugin_name + ".path_length_tolerance", 1.0);
  direction_estimation_distance_ = node->declare_or_get_parameter(
    plugin_name + ".direction_estimation_distance", 0.15);
  is_overshoot_valid_ = node->declare_or_get_parameter(
    plugin_name + ".is_overshoot_valid", false);

  // Add callback for dynamic parameters
  post_set_params_handler_ = node->add_post_set_parameters_callback(
    std::bind(
      &AxisGoalChecker::updateParametersCallback,
      this, std::placeholders::_1));
  on_set_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &AxisGoalChecker::validateParameterUpdatesCallback,
      this, std::placeholders::_1));
}

void AxisGoalChecker::reset()
{
}

bool AxisGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity,
  const nav_msgs::msg::Path & transformed_global_plan)
{
  // Since we do not consider orientation in this goal checker
  // we can directly check if the XY position is reached
  return isGoalXYReached(query_pose, goal_pose, velocity, transformed_global_plan);
}

bool AxisGoalChecker::isGoalXYReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist &,
  const nav_msgs::msg::Path & transformed_global_plan)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  // If the local plan length is longer than the tolerance, we skip the check
  const double path_length =
    nav2_util::geometry_utils::calculate_path_length(transformed_global_plan);
  if (path_length > path_length_tolerance_) {
    RCLCPP_INFO(
      logger_,
      "[%s] Not reached: remaining path length %.3f m > path_length_tolerance %.3f m",
      plugin_name_.c_str(), path_length, path_length_tolerance_);
    return false;
  }

  // Check if we have at least 2 poses to determine path direction
  if (transformed_global_plan.poses.size() >= 2) {
    const geometry_msgs::msg::Pose * before_goal_pose_ptr = nullptr;
    double dx = 0.0;
    double dy = 0.0;

    // Walk back from the goal until a pose is at least direction_estimation_distance_ away
    for (int i = transformed_global_plan.poses.size() - 2; i >= 0; --i) {
      const auto & candidate_pose = transformed_global_plan.poses[i].pose;
      dx = goal_pose.position.x - candidate_pose.position.x;
      dy = goal_pose.position.y - candidate_pose.position.y;
      double pose_distance = std::hypot(dx, dy);

      if (pose_distance >= direction_estimation_distance_) {
        before_goal_pose_ptr = &candidate_pose;
        break;
      }
    }

    // If no pose is far enough back to estimate a direction, fall back to simple distance check
    if (!before_goal_pose_ptr) {
      double distance_to_goal = std::hypot(
        goal_pose.position.x - query_pose.position.x,
        goal_pose.position.y - query_pose.position.y);
      double tolerance = std::min(along_path_tolerance_, cross_track_tolerance_);
      RCLCPP_INFO(
        logger_,
        "[%s] No plan pose >= %.3f m from goal to estimate direction, "
        "falling back to simple distance check: distance_to_goal %.3f m vs tolerance %.3f m -> %s",
        plugin_name_.c_str(), direction_estimation_distance_, distance_to_goal, tolerance,
        distance_to_goal < tolerance ? "REACHED" : "not reached");
      return distance_to_goal < tolerance;
    }

    // end of path direction
    double end_of_path_yaw = atan2(dy, dx);

    // Check if robot is already at goal (would cause atan2(0,0))
    double robot_to_goal_dx = goal_pose.position.x - query_pose.position.x;
    double robot_to_goal_dy = goal_pose.position.y - query_pose.position.y;
    double distance_to_goal = std::hypot(robot_to_goal_dx, robot_to_goal_dy);

    if (distance_to_goal < 1e-6) {
      RCLCPP_INFO(logger_, "[%s] REACHED: robot exactly at goal", plugin_name_.c_str());
      return true;  // Robot is at goal
    }

    double robot_to_goal_yaw = atan2(robot_to_goal_dy, robot_to_goal_dx);
    double projection_angle = angles::shortest_angular_distance(
      robot_to_goal_yaw, end_of_path_yaw);
    double along_path_distance = distance_to_goal * cos(projection_angle);
    double cross_track_distance = distance_to_goal * sin(projection_angle);

    const double effective_along = is_overshoot_valid_ ?
      along_path_distance : fabs(along_path_distance);
    const bool along_ok = effective_along < along_path_tolerance_;
    const bool cross_ok = fabs(cross_track_distance) < cross_track_tolerance_;
    RCLCPP_INFO(
      logger_,
      "[%s] distance_to_goal %.3f m, path_yaw %.3f rad, projection_angle %.3f rad | "
      "along_path %.3f m (%stolerance %.3f, overshoot_valid=%d) -> %s | "
      "cross_track %.3f m (tolerance %.3f) -> %s | %s",
      plugin_name_.c_str(), distance_to_goal, end_of_path_yaw, projection_angle,
      along_path_distance, is_overshoot_valid_ ? "signed, " : "abs, ", along_path_tolerance_,
      is_overshoot_valid_, along_ok ? "OK" : "FAIL",
      cross_track_distance, cross_track_tolerance_, cross_ok ? "OK" : "FAIL",
      along_ok && cross_ok ? "REACHED" : "not reached");
    return along_ok && cross_ok;
  } else {
    // Fallback: path has only 1 point, use simple distance check
    double distance_to_goal = std::hypot(
      goal_pose.position.x - query_pose.position.x,
      goal_pose.position.y - query_pose.position.y);
    double tolerance = std::min(along_path_tolerance_, cross_track_tolerance_);
    RCLCPP_INFO(
      logger_,
      "[%s] Path has fewer than 2 poses, falling back to simple distance check: "
      "distance_to_goal %.3f m vs tolerance %.3f m -> %s",
      plugin_name_.c_str(), distance_to_goal, tolerance,
      distance_to_goal < tolerance ? "REACHED" : "not reached");
    return distance_to_goal < tolerance;
  }
}

bool AxisGoalChecker::getTolerances(
  geometry_msgs::msg::Pose & pose_tolerance,
  geometry_msgs::msg::Twist & vel_tolerance,
  double & path_length_tolerance)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  double invalid_field = std::numeric_limits<double>::lowest();

  pose_tolerance.position.x = std::min(along_path_tolerance_, cross_track_tolerance_);
  pose_tolerance.position.y = std::min(along_path_tolerance_, cross_track_tolerance_);
  pose_tolerance.position.z = invalid_field;
  pose_tolerance.orientation =
    nav2_util::geometry_utils::orientationAroundZAxis(M_PI_2);

  vel_tolerance.linear.x = invalid_field;
  vel_tolerance.linear.y = invalid_field;
  vel_tolerance.linear.z = invalid_field;

  vel_tolerance.angular.x = invalid_field;
  vel_tolerance.angular.y = invalid_field;
  vel_tolerance.angular.z = invalid_field;

  path_length_tolerance = path_length_tolerance_;

  return true;
}

rcl_interfaces::msg::SetParametersResult
AxisGoalChecker::validateParameterUpdatesCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(plugin_name_ + ".") != 0) {
      continue;
    }
    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (parameter.as_double() < 0.0) {
        RCLCPP_WARN(
        logger_, "The value of parameter '%s' is incorrectly set to %f, "
        "it should be >=0. Ignoring parameter update.",
        param_name.c_str(), parameter.as_double());
        result.successful = false;
      }
      if (param_name == plugin_name_ + ".direction_estimation_distance") {
        const double value = parameter.as_double();
        if (value <= 0.0 || value >= path_length_tolerance_) {
          RCLCPP_WARN(
            logger_, "The value of parameter '%s' is set to %f, it should be >0 and "
            "<path_length_tolerance (%f). Ignoring parameter update.",
            param_name.c_str(), value, path_length_tolerance_);
          result.successful = false;
        }
      }
    }
  }
  return result;
}

void
AxisGoalChecker::updateParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);
  for (const auto & parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();
    if (name.find(plugin_name_ + ".") != 0) {
      continue;
    }
    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == plugin_name_ + ".along_path_tolerance") {
        along_path_tolerance_ = parameter.as_double();
      } else if (name == plugin_name_ + ".cross_track_tolerance") {
        cross_track_tolerance_ = parameter.as_double();
      } else if (name == plugin_name_ + ".path_length_tolerance") {
        path_length_tolerance_ = parameter.as_double();
      } else if (name == plugin_name_ + ".direction_estimation_distance") {
        direction_estimation_distance_ = parameter.as_double();
      }
    } else if (type == ParameterType::PARAMETER_BOOL) {
      if (name == plugin_name_ + ".is_overshoot_valid") {
        is_overshoot_valid_ = parameter.as_bool();
      }
    }
  }
}

}  // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav2_controller::AxisGoalChecker, nav2_core::GoalChecker)
