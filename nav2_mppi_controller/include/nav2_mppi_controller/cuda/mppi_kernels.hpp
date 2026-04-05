// Copyright (c) 2026 GPU acceleration proof of concept
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

#ifndef NAV2_MPPI_CONTROLLER__CUDA__MPPI_KERNELS_HPP_
#define NAV2_MPPI_CONTROLLER__CUDA__MPPI_KERNELS_HPP_

/**
 * @file mppi_kernels.hpp
 * @brief C++ interface to CUDA kernels for GPU-accelerated MPPI optimization.
 *
 * This header is pure C++ (no CUDA syntax) so it can be included from
 * regular .cpp files compiled with a standard C++ compiler. The actual
 * kernel implementations are in mppi_kernels.cu.
 */

namespace mppi::cuda
{

/**
 * @brief Parameters for costmap scoring on GPU.
 *
 * Mirrors the subset of costmap metadata needed by GPU kernels.
 */
struct CostmapParams
{
  float origin_x;
  float origin_y;
  float resolution;
  unsigned int size_x;
  unsigned int size_y;
};

/**
 * @brief Parameters for the obstacle/cost critic scoring.
 */
struct ScoringParams
{
  float collision_cost;                ///< Cost assigned to colliding trajectories
  float critical_cost;                 ///< Extra cost for near-collision points
  float near_collision_cost;           ///< Threshold above which critical cost is added
  float weight;                        ///< Critic weight (pre-divided by 254 for cost critic)
  float repulsion_weight;              ///< Weight for repulsive cost (obstacles critic)
  float critical_weight;               ///< Weight for critical cost (obstacles critic)
  float collision_margin_distance;     ///< Distance margin for near-collision
  float inflation_radius;              ///< Inflation radius from inflation layer
  float inflation_scale_factor;        ///< Scale factor from inflation layer
  float inscribed_radius;              ///< Inscribed radius of robot
  bool near_goal;                      ///< Whether we're near the goal
  bool is_tracking_unknown;            ///< Whether unknown cells are tracked
  unsigned int trajectory_point_step;  ///< Step size for trajectory point sampling
};

/**
 * @brief Opaque handle to GPU device memory for MPPI acceleration.
 *
 * Created once, reused across optimization cycles. Manages all CUDA
 * allocations and the CUDA stream.
 */
struct GpuContext;

/**
 * @brief Allocate GPU context for given problem dimensions.
 * @param batch_size Number of sampled trajectories (e.g. 1000)
 * @param time_steps Horizon length (e.g. 56)
 * @param max_costmap_size Maximum costmap cells (size_x * size_y)
 * @return Opaque pointer to GPU context (caller owns, free with destroyGpuContext)
 */
GpuContext * createGpuContext(
  unsigned int batch_size, unsigned int time_steps, unsigned int max_costmap_size);

/**
 * @brief Free GPU context and all device memory.
 */
void destroyGpuContext(GpuContext * ctx);

/**
 * @brief Upload costmap data to GPU.
 *
 * Async copy to device — returns quickly, actual transfer overlaps with CPU work.
 *
 * @param ctx GPU context
 * @param costmap_data Host pointer to costmap unsigned char array
 * @param params Costmap metadata (origin, resolution, size)
 */
void uploadCostmap(
  GpuContext * ctx, const unsigned char * costmap_data, const CostmapParams & params);

/**
 * @brief Score all trajectories on GPU using costmap cost lookups.
 *
 * This replaces the inner double loop of CostCritic::score() — checking
 * batch_size * time_steps points against the costmap in parallel.
 *
 * @param ctx GPU context (must have costmap uploaded)
 * @param traj_x Host pointer to trajectories.x data [batch_size x time_steps, col-major]
 * @param traj_y Host pointer to trajectories.y data
 * @param traj_yaws Host pointer to trajectories.yaws data
 * @param batch_size Number of trajectories
 * @param time_steps Number of timesteps per trajectory
 * @param params Scoring parameters
 * @param[out] costs_out Host pointer to output costs [batch_size] — ADDED to existing values
 * @param[out] all_collide Set to true if every trajectory is in collision
 */
void scoreTrajectories(
  GpuContext * ctx,
  const float * traj_x, const float * traj_y, const float * traj_yaws,
  unsigned int batch_size, unsigned int time_steps,
  const ScoringParams & params,
  float * costs_out, bool * all_collide);

/**
 * @brief Compute softmax-weighted control update on GPU.
 *
 * Given per-trajectory costs and noised control sequences, compute:
 *   softmax = exp(-costs / temperature) / sum(exp(-costs / temperature))
 *   control_out = softmax^T * controls_matrix
 *
 * @param ctx GPU context
 * @param costs Host pointer to per-trajectory costs [batch_size]
 * @param cvx Host pointer to state.cvx data [batch_size x time_steps, col-major]
 * @param cwz Host pointer to state.cwz data
 * @param cvy Host pointer to state.cvy data (nullptr if non-holonomic)
 * @param batch_size Number of trajectories
 * @param time_steps Number of timesteps
 * @param temperature Softmax temperature
 * @param[out] out_vx Result control vx [time_steps]
 * @param[out] out_wz Result control wz [time_steps]
 * @param[out] out_vy Result control vy [time_steps] (unused if cvy == nullptr)
 */
void softmaxControlUpdate(
  GpuContext * ctx,
  const float * costs,
  const float * cvx, const float * cwz, const float * cvy,
  unsigned int batch_size, unsigned int time_steps,
  float temperature,
  float * out_vx, float * out_wz, float * out_vy);

}  // namespace mppi::cuda

#endif  // NAV2_MPPI_CONTROLLER__CUDA__MPPI_KERNELS_HPP_
