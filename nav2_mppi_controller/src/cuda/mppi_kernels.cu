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

/**
 * @file mppi_kernels.cu
 * @brief CUDA kernel implementations for GPU-accelerated MPPI scoring.
 *
 * Key design decisions:
 * - One CUDA thread per trajectory for scoring (batch_size threads).
 *   Each thread iterates over its time_steps — this avoids cross-thread
 *   reduction and matches the early-exit-on-collision pattern.
 * - Costmap stored as a CUDA texture for cached 2D lookups.
 * - Softmax uses shared memory reduction for numerical stability.
 * - All host↔device transfers use a dedicated CUDA stream for async overlap.
 */

#include "nav2_mppi_controller/cuda/mppi_kernels.hpp"

#include <cuda_runtime.h>
#include <cstdio>
#include <cfloat>
#include <cmath>

// Macro for checking CUDA errors in PoC — logs and continues
#define CUDA_CHECK(call) \
  do { \
    cudaError_t err = (call); \
    if (err != cudaSuccess) { \
      fprintf(stderr, "CUDA error at %s:%d: %s\n", __FILE__, __LINE__, \
              cudaGetErrorString(err)); \
    } \
  } while (0)

namespace mppi::cuda
{

// ──────────────────────────────────────────────────────────────────────
// Device‑side constants copied once per costmap upload
// ──────────────────────────────────────────────────────────────────────
__constant__ CostmapParams d_costmap_params;

// ──────────────────────────────────────────────────────────────────────
// GPU context — holds all device allocations and the CUDA stream
// ──────────────────────────────────────────────────────────────────────
struct GpuContext
{
  // Problem dimensions
  unsigned int batch_size;
  unsigned int time_steps;
  unsigned int max_costmap_size;

  // CUDA stream for async operations
  cudaStream_t stream;

  // Device buffers — trajectories
  float * d_traj_x;
  float * d_traj_y;
  float * d_traj_yaws;

  // Device buffers — costmap
  unsigned char * d_costmap;

  // Device buffers — per‑trajectory costs (output)
  float * d_costs;

  // Device buffers — controls for softmax
  float * d_cvx;
  float * d_cwz;
  float * d_cvy;

  // Device buffers — softmax output controls
  float * d_out_vx;
  float * d_out_wz;
  float * d_out_vy;

  // Device buffer — single bool for all_collide reduction
  bool * d_all_collide;
};

// ──────────────────────────────────────────────────────────────────────
// Context lifecycle
// ──────────────────────────────────────────────────────────────────────
GpuContext * createGpuContext(
  unsigned int batch_size, unsigned int time_steps, unsigned int max_costmap_size)
{
  auto * ctx = new GpuContext();
  ctx->batch_size = batch_size;
  ctx->time_steps = time_steps;
  ctx->max_costmap_size = max_costmap_size;

  CUDA_CHECK(cudaStreamCreate(&ctx->stream));

  const size_t traj_bytes = batch_size * time_steps * sizeof(float);
  const size_t cost_bytes = batch_size * sizeof(float);

  // Trajectory buffers
  CUDA_CHECK(cudaMalloc(&ctx->d_traj_x, traj_bytes));
  CUDA_CHECK(cudaMalloc(&ctx->d_traj_y, traj_bytes));
  CUDA_CHECK(cudaMalloc(&ctx->d_traj_yaws, traj_bytes));

  // Costmap buffer
  CUDA_CHECK(cudaMalloc(&ctx->d_costmap, max_costmap_size * sizeof(unsigned char)));

  // Cost output buffer
  CUDA_CHECK(cudaMalloc(&ctx->d_costs, cost_bytes));

  // Control buffers for softmax
  CUDA_CHECK(cudaMalloc(&ctx->d_cvx, traj_bytes));
  CUDA_CHECK(cudaMalloc(&ctx->d_cwz, traj_bytes));
  CUDA_CHECK(cudaMalloc(&ctx->d_cvy, traj_bytes));

  // Output control sequences
  CUDA_CHECK(cudaMalloc(&ctx->d_out_vx, time_steps * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&ctx->d_out_wz, time_steps * sizeof(float)));
  CUDA_CHECK(cudaMalloc(&ctx->d_out_vy, time_steps * sizeof(float)));

  // All-collide flag
  CUDA_CHECK(cudaMalloc(&ctx->d_all_collide, sizeof(bool)));

  return ctx;
}

void destroyGpuContext(GpuContext * ctx)
{
  if (!ctx) {return;}

  cudaStreamSynchronize(ctx->stream);
  cudaStreamDestroy(ctx->stream);

  cudaFree(ctx->d_traj_x);
  cudaFree(ctx->d_traj_y);
  cudaFree(ctx->d_traj_yaws);
  cudaFree(ctx->d_costmap);
  cudaFree(ctx->d_costs);
  cudaFree(ctx->d_cvx);
  cudaFree(ctx->d_cwz);
  cudaFree(ctx->d_cvy);
  cudaFree(ctx->d_out_vx);
  cudaFree(ctx->d_out_wz);
  cudaFree(ctx->d_out_vy);
  cudaFree(ctx->d_all_collide);

  delete ctx;
}

// ──────────────────────────────────────────────────────────────────────
// Costmap upload
// ──────────────────────────────────────────────────────────────────────
void uploadCostmap(
  GpuContext * ctx, const unsigned char * costmap_data, const CostmapParams & params)
{
  const size_t costmap_bytes = params.size_x * params.size_y * sizeof(unsigned char);

  CUDA_CHECK(cudaMemcpyToSymbolAsync(
    d_costmap_params, &params, sizeof(CostmapParams), 0,
    cudaMemcpyHostToDevice, ctx->stream));

  CUDA_CHECK(cudaMemcpyAsync(
    ctx->d_costmap, costmap_data, costmap_bytes,
    cudaMemcpyHostToDevice, ctx->stream));
}

// ──────────────────────────────────────────────────────────────────────
// Trajectory scoring kernel
// ──────────────────────────────────────────────────────────────────────

/**
 * GPU kernel: one thread per trajectory.
 *
 * For each trajectory, iterate over time steps, look up costmap cost,
 * and accumulate per-trajectory scoring. Mirrors the CostCritic::score()
 * inner loop but runs batch_size threads in parallel.
 *
 * Eigen stores ArrayXXf in column-major order:
 *   element (row=i, col=j) is at index j * batch_size + i
 * So for thread i processing timestep j: idx = j * batch_size + i
 */
__global__ void scoreTrajectories_kernel(
  const float * __restrict__ traj_x,
  const float * __restrict__ traj_y,
  const float * __restrict__ traj_yaws,
  const unsigned char * __restrict__ costmap,
  unsigned int batch_size,
  unsigned int time_steps,
  ScoringParams params,
  float * __restrict__ costs,
  bool * __restrict__ all_collide_flag)
{
  const unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= batch_size) {return;}

  const CostmapParams & cm = d_costmap_params;
  const float inv_resolution = 1.0f / cm.resolution;
  const unsigned int step = params.trajectory_point_step;

  float traj_cost = 0.0f;
  float repulsive_cost = 0.0f;
  bool trajectory_collide = false;

  // Number of strided timesteps
  unsigned int strided_steps = (time_steps - 1) / step + 1;

  for (unsigned int js = 0; js < strided_steps && !trajectory_collide; ++js) {
    unsigned int j = js * step;
    // Eigen col-major: (i, j) -> j * batch_size + i
    unsigned int idx = j * batch_size + i;
    float x = traj_x[idx];
    float y = traj_y[idx];

    // World to map
    float mx_f = (x - cm.origin_x) * inv_resolution;
    float my_f = (y - cm.origin_y) * inv_resolution;

    if (mx_f < 0.0f || my_f < 0.0f) {
      // Out of map — treat as collision
      traj_cost = params.collision_cost;
      trajectory_collide = true;
      break;
    }

    unsigned int mx = static_cast<unsigned int>(mx_f);
    unsigned int my = static_cast<unsigned int>(my_f);

    if (mx >= cm.size_x || my >= cm.size_y) {
      traj_cost = params.collision_cost;
      trajectory_collide = true;
      break;
    }

    float pose_cost = static_cast<float>(costmap[my * cm.size_x + mx]);

    if (pose_cost < 1.0f) {
      continue;  // Free space
    }

    // Collision check (simplified — center-point only, no footprint on GPU)
    unsigned char cost_byte = static_cast<unsigned char>(pose_cost);
    if (cost_byte == 254u) {  // LETHAL_OBSTACLE
      traj_cost = params.collision_cost;
      trajectory_collide = true;
      break;
    }
    if (cost_byte == 253u) {  // INSCRIBED_INFLATED_OBSTACLE
      traj_cost = params.collision_cost;
      trajectory_collide = true;
      break;
    }
    if (cost_byte == 255u) {  // NO_INFORMATION
      if (!params.is_tracking_unknown) {
        traj_cost = params.collision_cost;
        trajectory_collide = true;
        break;
      }
      continue;
    }

    // Near-collision
    if (pose_cost >= params.near_collision_cost) {
      traj_cost += params.critical_cost;
    } else if (!params.near_goal) {
      traj_cost += pose_cost;
    }

    // Repulsive cost (for ObstaclesCritic-style scoring)
    if (params.inflation_radius > 0.0f && params.inflation_scale_factor > 0.0f) {
      float dist_to_obj = (params.inflation_scale_factor * params.inscribed_radius -
        logf(pose_cost) + logf(253.0f)) / params.inflation_scale_factor;
      dist_to_obj -= params.inscribed_radius;  // center-point

      if (dist_to_obj < params.collision_margin_distance) {
        traj_cost += (params.collision_margin_distance - dist_to_obj);
      }
      if (!params.near_goal) {
        repulsive_cost += params.inflation_radius - dist_to_obj;
      }
    }
  }

  // Write per-trajectory cost (add to existing costs)
  if (trajectory_collide) {
    costs[i] += params.collision_cost * params.weight;
  } else {
    costs[i] += traj_cost * (params.weight / static_cast<float>(strided_steps));
  }

  // Atomic AND for all_collide detection:
  // Start with true; any non-colliding trajectory sets it to false
  if (!trajectory_collide) {
    *all_collide_flag = false;
  }
}

void scoreTrajectories(
  GpuContext * ctx,
  const float * traj_x, const float * traj_y, const float * traj_yaws,
  unsigned int batch_size, unsigned int time_steps,
  const ScoringParams & params,
  float * costs_out, bool * all_collide)
{
  const size_t traj_bytes = batch_size * time_steps * sizeof(float);
  const size_t cost_bytes = batch_size * sizeof(float);

  // Upload trajectories
  CUDA_CHECK(cudaMemcpyAsync(ctx->d_traj_x, traj_x, traj_bytes,
    cudaMemcpyHostToDevice, ctx->stream));
  CUDA_CHECK(cudaMemcpyAsync(ctx->d_traj_y, traj_y, traj_bytes,
    cudaMemcpyHostToDevice, ctx->stream));
  CUDA_CHECK(cudaMemcpyAsync(ctx->d_traj_yaws, traj_yaws, traj_bytes,
    cudaMemcpyHostToDevice, ctx->stream));

  // Upload current costs (we add to them)
  CUDA_CHECK(cudaMemcpyAsync(ctx->d_costs, costs_out, cost_bytes,
    cudaMemcpyHostToDevice, ctx->stream));

  // Initialize all_collide to true
  bool init_collide = true;
  CUDA_CHECK(cudaMemcpyAsync(ctx->d_all_collide, &init_collide, sizeof(bool),
    cudaMemcpyHostToDevice, ctx->stream));

  // Launch kernel — 256 threads per block
  const unsigned int block_size = 256;
  const unsigned int grid_size = (batch_size + block_size - 1) / block_size;

  scoreTrajectories_kernel<<<grid_size, block_size, 0, ctx->stream>>>(
    ctx->d_traj_x, ctx->d_traj_y, ctx->d_traj_yaws,
    ctx->d_costmap,
    batch_size, time_steps,
    params,
    ctx->d_costs,
    ctx->d_all_collide);

  // Download results
  CUDA_CHECK(cudaMemcpyAsync(costs_out, ctx->d_costs, cost_bytes,
    cudaMemcpyDeviceToHost, ctx->stream));
  CUDA_CHECK(cudaMemcpyAsync(all_collide, ctx->d_all_collide, sizeof(bool),
    cudaMemcpyDeviceToHost, ctx->stream));

  // Wait for results
  CUDA_CHECK(cudaStreamSynchronize(ctx->stream));
}

// ──────────────────────────────────────────────────────────────────────
// Softmax weighted control update kernel
// ──────────────────────────────────────────────────────────────────────

/**
 * Kernel 1: compute softmax weights from costs.
 * One block does parallel reduction for min, then computes exp(-normalized/T).
 */
__global__ void computeSoftmax_kernel(
  const float * __restrict__ costs,
  float * __restrict__ softmax_weights,
  unsigned int batch_size,
  float inv_temperature)
{
  extern __shared__ float shared[];

  const unsigned int tid = threadIdx.x;
  const unsigned int i = blockIdx.x * blockDim.x + tid;

  // Step 1: find min cost (parallel reduction)
  float val = (i < batch_size) ? costs[i] : FLT_MAX;
  shared[tid] = val;
  __syncthreads();

  for (unsigned int s = blockDim.x / 2; s > 0; s >>= 1) {
    if (tid < s && shared[tid + s] < shared[tid]) {
      shared[tid] = shared[tid + s];
    }
    __syncthreads();
  }
  float min_cost = shared[0];

  // Step 2: compute unnormalized exp
  if (i < batch_size) {
    softmax_weights[i] = expf(-inv_temperature * (costs[i] - min_cost));
  }
}

/**
 * Kernel 2: weighted sum of controls by softmax weights.
 * One thread per time step — each does a dot product over batch_size.
 * This is O(time_steps * batch_size) total work, fine for time_steps ≈ 56.
 */
__global__ void weightedControlSum_kernel(
  const float * __restrict__ softmax_weights,
  const float * __restrict__ cvx,
  const float * __restrict__ cwz,
  const float * __restrict__ cvy,
  unsigned int batch_size,
  unsigned int time_steps,
  float sum_weights,
  bool has_vy,
  float * __restrict__ out_vx,
  float * __restrict__ out_wz,
  float * __restrict__ out_vy)
{
  const unsigned int j = blockIdx.x * blockDim.x + threadIdx.x;  // time step index
  if (j >= time_steps) {return;}

  float vx_sum = 0.0f;
  float wz_sum = 0.0f;
  float vy_sum = 0.0f;
  const float inv_sum = 1.0f / sum_weights;

  for (unsigned int i = 0; i < batch_size; ++i) {
    float w = softmax_weights[i];
    // Eigen col-major: (row=i, col=j) = j * batch_size + i
    unsigned int idx = j * batch_size + i;
    vx_sum += w * cvx[idx];
    wz_sum += w * cwz[idx];
    if (has_vy) {
      vy_sum += w * cvy[idx];
    }
  }

  out_vx[j] = vx_sum * inv_sum;
  out_wz[j] = wz_sum * inv_sum;
  if (has_vy) {
    out_vy[j] = vy_sum * inv_sum;
  }
}

void softmaxControlUpdate(
  GpuContext * ctx,
  const float * costs,
  const float * cvx, const float * cwz, const float * cvy,
  unsigned int batch_size, unsigned int time_steps,
  float temperature,
  float * out_vx, float * out_wz, float * out_vy)
{
  const size_t traj_bytes = batch_size * time_steps * sizeof(float);
  const size_t cost_bytes = batch_size * sizeof(float);
  const float inv_temp = 1.0f / temperature;

  // Upload costs and controls
  CUDA_CHECK(cudaMemcpyAsync(ctx->d_costs, costs, cost_bytes,
    cudaMemcpyHostToDevice, ctx->stream));
  CUDA_CHECK(cudaMemcpyAsync(ctx->d_cvx, cvx, traj_bytes,
    cudaMemcpyHostToDevice, ctx->stream));
  CUDA_CHECK(cudaMemcpyAsync(ctx->d_cwz, cwz, traj_bytes,
    cudaMemcpyHostToDevice, ctx->stream));

  bool has_vy = (cvy != nullptr);
  if (has_vy) {
    CUDA_CHECK(cudaMemcpyAsync(ctx->d_cvy, cvy, traj_bytes,
      cudaMemcpyHostToDevice, ctx->stream));
  }

  // Step 1: compute softmax weights
  // For simplicity in this PoC, use a single block for small batch sizes.
  // For batch_size=1000, one block of 1024 threads handles it.
  unsigned int block_size = 1024;
  unsigned int smem = block_size * sizeof(float);
  computeSoftmax_kernel<<<1, block_size, smem, ctx->stream>>>(
    ctx->d_costs, ctx->d_costs,  // reuse costs buffer for weights
    batch_size, inv_temp);

  // Download weights to compute sum on CPU (simpler than another reduction kernel)
  // In production, this would be a device-side reduction.
  float weights[batch_size];  // VLA, fine for PoC
  CUDA_CHECK(cudaMemcpyAsync(weights, ctx->d_costs, cost_bytes,
    cudaMemcpyDeviceToHost, ctx->stream));
  CUDA_CHECK(cudaStreamSynchronize(ctx->stream));

  float sum_weights = 0.0f;
  for (unsigned int i = 0; i < batch_size; ++i) {
    sum_weights += weights[i];
  }

  if (sum_weights < 1e-10f) {
    sum_weights = 1e-10f;
  }

  // Step 2: weighted sum — one thread per timestep
  unsigned int ts_block = 64;
  unsigned int ts_grid = (time_steps + ts_block - 1) / ts_block;
  weightedControlSum_kernel<<<ts_grid, ts_block, 0, ctx->stream>>>(
    ctx->d_costs,  // contains softmax weights now
    ctx->d_cvx, ctx->d_cwz, ctx->d_cvy,
    batch_size, time_steps, sum_weights, has_vy,
    ctx->d_out_vx, ctx->d_out_wz, ctx->d_out_vy);

  // Download results
  CUDA_CHECK(cudaMemcpyAsync(out_vx, ctx->d_out_vx, time_steps * sizeof(float),
    cudaMemcpyDeviceToHost, ctx->stream));
  CUDA_CHECK(cudaMemcpyAsync(out_wz, ctx->d_out_wz, time_steps * sizeof(float),
    cudaMemcpyDeviceToHost, ctx->stream));
  if (has_vy) {
    CUDA_CHECK(cudaMemcpyAsync(out_vy, ctx->d_out_vy, time_steps * sizeof(float),
      cudaMemcpyDeviceToHost, ctx->stream));
  }

  CUDA_CHECK(cudaStreamSynchronize(ctx->stream));
}

}  // namespace mppi::cuda
