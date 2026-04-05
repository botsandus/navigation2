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

#ifndef NAV2_MPPI_CONTROLLER__CUDA__GPU_SCORER_HPP_
#define NAV2_MPPI_CONTROLLER__CUDA__GPU_SCORER_HPP_

/**
 * @file gpu_scorer.hpp
 * @brief GPU-accelerated batch trajectory scorer for MPPI.
 *
 * Wraps the CUDA kernel interface to provide a simple API that can be
 * called from the Optimizer. Manages GPU context lifetime and provides
 * methods matching the existing CPU scoring flow.
 *
 * Usage in Optimizer:
 *   - Call initialize() once during on_configure
 *   - Call uploadCostmap() once per optimize() cycle
 *   - Call scoreTrajectoriesBatch() to replace CostCritic::score()
 *   - Call softmaxUpdate() to replace the softmax portion of updateControlSequence()
 */

#include <memory>

#include "nav2_mppi_controller/cuda/mppi_kernels.hpp"
#include "nav2_mppi_controller/critic_data.hpp"
#include "nav2_mppi_controller/models/state.hpp"
#include "nav2_mppi_controller/models/trajectories.hpp"
#include "nav2_mppi_controller/models/control_sequence.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"

namespace mppi::cuda
{

/**
 * @class GpuScorer
 * @brief Manages GPU resources and provides batch scoring for MPPI trajectories.
 *
 * This class owns the GpuContext and translates between Nav2/Eigen data
 * structures and the raw pointers expected by the CUDA kernels.
 */
class GpuScorer
{
public:
  GpuScorer() = default;
  ~GpuScorer() {shutdown();}

  // Non-copyable, movable
  GpuScorer(const GpuScorer &) = delete;
  GpuScorer & operator=(const GpuScorer &) = delete;
  GpuScorer(GpuScorer && other) noexcept
  : ctx_(other.ctx_) {other.ctx_ = nullptr;}
  GpuScorer & operator=(GpuScorer && other) noexcept
  {
    if (this != &other) {shutdown(); ctx_ = other.ctx_; other.ctx_ = nullptr;}
    return *this;
  }

  /**
   * @brief Initialize GPU context with problem dimensions.
   * @param batch_size Number of trajectories (e.g. 1000)
   * @param time_steps Horizon length (e.g. 56)
   * @param max_costmap_cells Maximum expected costmap size (size_x * size_y)
   */
  void initialize(unsigned int batch_size, unsigned int time_steps, unsigned int max_costmap_cells)
  {
    shutdown();
    ctx_ = createGpuContext(batch_size, time_steps, max_costmap_cells);
  }

  /**
   * @brief Release GPU resources.
   */
  void shutdown()
  {
    if (ctx_) {
      destroyGpuContext(ctx_);
      ctx_ = nullptr;
    }
  }

  /**
   * @brief Check if GPU context is initialized.
   */
  bool isInitialized() const {return ctx_ != nullptr;}

  /**
   * @brief Upload the current costmap to GPU memory.
   * @param costmap Pointer to nav2 costmap
   */
  void uploadCostmap(const nav2_costmap_2d::Costmap2D * costmap)
  {
    CostmapParams params;
    params.origin_x = static_cast<float>(costmap->getOriginX());
    params.origin_y = static_cast<float>(costmap->getOriginY());
    params.resolution = static_cast<float>(costmap->getResolution());
    params.size_x = costmap->getSizeInCellsX();
    params.size_y = costmap->getSizeInCellsY();

    mppi::cuda::uploadCostmap(ctx_, costmap->getCharMap(), params);
  }

  /**
   * @brief Score all trajectories against the costmap on GPU.
   *
   * Replaces the inner loop of CostCritic::score(). The costmap must
   * have been uploaded first via uploadCostmap().
   *
   * @param data CriticData containing trajectories and costs
   * @param params Scoring parameters
   * @return true if all trajectories are in collision
   */
  bool scoreTrajectoriesBatch(
    CriticData & data,
    const ScoringParams & params)
  {
    const unsigned int batch_size = data.trajectories.x.rows();
    const unsigned int time_steps = data.trajectories.x.cols();

    bool all_collide = false;

    scoreTrajectories(
      ctx_,
      data.trajectories.x.data(),
      data.trajectories.y.data(),
      data.trajectories.yaws.data(),
      batch_size, time_steps,
      params,
      data.costs.data(),
      &all_collide);

    return all_collide;
  }

  /**
   * @brief Compute softmax-weighted control sequence update on GPU.
   *
   * @param costs Per-trajectory cost array
   * @param state State with noised controls (cvx, cwz, cvy)
   * @param control_sequence Output control sequence to update
   * @param temperature Softmax temperature
   * @param is_holonomic Whether to process vy
   */
  void softmaxUpdate(
    const Eigen::ArrayXf & costs,
    const models::State & state,
    models::ControlSequence & control_sequence,
    float temperature,
    bool is_holonomic)
  {
    const unsigned int batch_size = state.cvx.rows();
    const unsigned int time_steps = state.cvx.cols();

    softmaxControlUpdate(
      ctx_,
      costs.data(),
      state.cvx.data(),
      state.cwz.data(),
      is_holonomic ? state.cvy.data() : nullptr,
      batch_size, time_steps,
      temperature,
      control_sequence.vx.data(),
      control_sequence.wz.data(),
      is_holonomic ? control_sequence.vy.data() : nullptr);
  }

private:
  GpuContext * ctx_{nullptr};
};

}  // namespace mppi::cuda

#endif  // NAV2_MPPI_CONTROLLER__CUDA__GPU_SCORER_HPP_
