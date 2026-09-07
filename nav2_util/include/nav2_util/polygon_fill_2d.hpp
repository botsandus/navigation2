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

#ifndef NAV2_UTIL__POLYGON_FILL_2D_HPP_
#define NAV2_UTIL__POLYGON_FILL_2D_HPP_

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <vector>

namespace nav2_util
{

/// @brief Outcome of a 2D fill operation
enum class FillResult
{
  FILLED,             ///< Spans were produced (possibly clipped to the grid)
  EMPTY_EXTENT,       ///< The shape has no visible extent on the grid
  NON_FINITE_VERTEX,  ///< A vertex/center/radius was NaN or Inf (e.g. bad TF)
};

/**
 * @brief Rasterise a filled polygon on a grid using a scanline algorithm,
 * applying an action for each horizontal span of covered cells.
 *
 * Vertices are given in continuous cell coordinates: for a grid with a world
 * origin and resolution, cell_x = (world_x - origin_x) / resolution - 0.5.
 * Using continuous coordinates makes the covered-cell decision exactly match
 * nav2_util::geometry_utils::isPointInsidePolygon() evaluated at cell centres:
 * edge intersections use the same half-open interval (y_lo, y_hi] and the
 * same multiply-then-divide formula.
 *
 * The polygon is implicitly closed (an edge connects the last vertex to the
 * first). Spans are clipped to [0, size_x) x [0, size_y); shapes partially or
 * fully outside the grid are safe.
 *
 * @param vx Polygon vertex X coordinates (continuous cell coordinates)
 * @param vy Polygon vertex Y coordinates (continuous cell coordinates)
 * @param size_x Grid width in cells
 * @param size_y Grid height in cells
 * @param apply_span Functor called as apply_span(y, x_start, x_end) with
 * unsigned int cell indices, x range inclusive
 * @return FillResult describing the outcome
 */
template<class SpanActionType>
inline FillResult fillPolygon(
  const std::vector<double> & vx, const std::vector<double> & vy,
  unsigned int size_x, unsigned int size_y,
  SpanActionType apply_span)
{
  const std::size_t n = std::min(vx.size(), vy.size());
  if (n < 3 || size_x == 0 || size_y == 0) {
    return FillResult::EMPTY_EXTENT;
  }

  // Guard against NaN vertices (e.g. from a bad TF result). A NaN violates
  // std::sort's strict-weak-ordering and can walk off the buffer.
  for (std::size_t i = 0; i < n; i++) {
    if (!std::isfinite(vx[i]) || !std::isfinite(vy[i])) {
      return FillResult::NON_FINITE_VERTEX;
    }
  }

  // Clamp in double space before casting to int to avoid UB when the
  // polygon extends far outside the grid.
  const double grid_h_d = static_cast<double>(size_y - 1);
  const double grid_w_d = static_cast<double>(size_x - 1);

  double y_min_d = std::ceil(*std::min_element(vy.begin(), vy.begin() + n));
  double y_max_d = std::floor(*std::max_element(vy.begin(), vy.begin() + n));
  y_min_d = std::clamp(y_min_d, 0.0, grid_h_d);
  y_max_d = std::clamp(y_max_d, 0.0, grid_h_d);
  const int y_min = static_cast<int>(y_min_d);
  const int y_max = static_cast<int>(y_max_d);

  if (y_min > y_max) {
    return FillResult::EMPTY_EXTENT;
  }

  // Precompute per-edge information. The intersection at scanline y is
  // computed as xi + (y - yi) * dx / dy, preserving the isPointInsidePolygon
  // multiply-then-divide order for bit-identical output on boundary cases.
  struct EdgeInfo
  {
    double y_lo;  // lower (exclusive) Y bound — matches isPointInsidePolygon (y_lo, y_hi]
    double y_hi;  // upper (inclusive) Y bound
    double xi;    // X at the lower-Y endpoint
    double yi;    // Y at the lower-Y endpoint
    double dx;    // xj - xi
    double dy;    // yj - yi (always > 0 after orientation normalisation)
  };
  std::vector<EdgeInfo> edges;
  edges.reserve(n);
  for (std::size_t i = 0; i < n; i++) {
    const std::size_t j = (i + 1) % n;
    const double y0 = vy[i], y1 = vy[j];
    const double x0 = vx[i], x1 = vx[j];
    if (y0 == y1) {
      continue;  // horizontal edge — never contributes an intersection
    }
    EdgeInfo e;
    // Normalise so dy > 0 (low-to-high) to keep xi/yi at the lower endpoint.
    if (y0 < y1) {
      e.y_lo = y0;  e.y_hi = y1;  e.xi = x0;  e.yi = y0;
      e.dx = x1 - x0;  e.dy = y1 - y0;
    } else {
      e.y_lo = y1;  e.y_hi = y0;  e.xi = x1;  e.yi = y1;
      e.dx = x0 - x1;  e.dy = y0 - y1;
    }
    edges.push_back(e);
  }

  if (edges.empty()) {
    return FillResult::EMPTY_EXTENT;  // degenerate (all edges horizontal)
  }

  // Allocate the intersection vector once outside the loop; one entry per
  // edge is the maximum, so no heap allocation occurs during the sweep.
  std::vector<double> xs;
  xs.reserve(edges.size());

  for (int y = y_min; y <= y_max; y++) {
    xs.clear();
    for (const auto & e : edges) {
      // Half-open interval (y_lo, y_hi] — lower exclusive, upper inclusive —
      // matching nav2_util::geometry_utils::isPointInsidePolygon().
      if (y <= e.y_lo || y > e.y_hi) {
        continue;
      }
      xs.push_back(e.xi + (y - e.yi) * e.dx / e.dy);
    }

    std::sort(xs.begin(), xs.end());

    for (std::size_t k = 0; k + 1 < xs.size(); k += 2) {
      const double a = std::ceil(xs[k]);
      const double b = std::ceil(xs[k + 1]) - 1.0;
      if (b < 0.0 || a > grid_w_d) {
        continue;
      }
      const int x_start = static_cast<int>(std::max(a, 0.0));
      const int x_end = static_cast<int>(std::min(b, grid_w_d));
      if (x_start > x_end) {
        continue;
      }
      apply_span(
        static_cast<unsigned int>(y),
        static_cast<unsigned int>(x_start),
        static_cast<unsigned int>(x_end));
    }
  }

  return FillResult::FILLED;
}

/**
 * @brief Rasterise a filled circle on a grid, applying an action for each
 * horizontal span of covered cells.
 *
 * The centre is given in continuous cell coordinates (see fillPolygon) so
 * sub-cell precision is preserved; the radius is in cells. A circle whose
 * centre is off-grid but whose body overlaps the grid still draws. Work is
 * O(radius) rows with a single span per row.
 *
 * @param cx Circle centre X (continuous cell coordinates)
 * @param cy Circle centre Y (continuous cell coordinates)
 * @param radius Circle radius in cells
 * @param size_x Grid width in cells
 * @param size_y Grid height in cells
 * @param apply_span Functor called as apply_span(y, x_start, x_end) with
 * unsigned int cell indices, x range inclusive
 * @return FillResult describing the outcome
 */
template<class SpanActionType>
inline FillResult fillCircle(
  double cx, double cy, double radius,
  unsigned int size_x, unsigned int size_y,
  SpanActionType apply_span)
{
  if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(radius)) {
    return FillResult::NON_FINITE_VERTEX;
  }
  if (radius <= 0.0 || size_x == 0 || size_y == 0) {
    return FillResult::EMPTY_EXTENT;
  }

  const int grid_w = static_cast<int>(size_x);
  const int grid_h = static_cast<int>(size_y);
  const double grid_w_d = static_cast<double>(grid_w);
  const double grid_h_d = static_cast<double>(grid_h);

  // Check against the circle's extent, not just its centre, so a circle
  // whose centre is off-grid but whose body overlaps still draws.
  const double y0_check_d = std::clamp(std::ceil(cy - radius), -1.0, grid_h_d);
  const double y1_check_d = std::clamp(std::floor(cy + radius), -1.0, grid_h_d);
  const double x0_check_d = std::clamp(std::ceil(cx - radius), -1.0, grid_w_d);
  const double x1_check_d = std::clamp(std::floor(cx + radius), -1.0, grid_w_d);

  const int y0_check = static_cast<int>(y0_check_d);
  const int y1_check = static_cast<int>(y1_check_d);
  const int x0_check = static_cast<int>(x0_check_d);
  const int x1_check = static_cast<int>(x1_check_d);

  if (y0_check >= grid_h || y1_check < 0 || x0_check >= grid_w || x1_check < 0) {
    return FillResult::EMPTY_EXTENT;
  }

  const int y0 = std::max(y0_check, 0);
  const int y1 = std::min(y1_check, grid_h - 1);

  for (int y = y0; y <= y1; y++) {
    const double t = radius * radius - (y - cy) * (y - cy);
    if (t < 0.0) {
      continue;
    }
    const double dx = std::sqrt(t);
    const double a = std::ceil(cx - dx);
    const double b = std::floor(cx + dx);
    if (b < 0.0 || a > grid_w_d - 1.0) {
      continue;
    }
    const int x_lo = static_cast<int>(std::max(a, 0.0));
    const int x_hi = static_cast<int>(std::min(b, grid_w_d - 1.0));
    if (x_lo > x_hi) {
      continue;
    }
    apply_span(
      static_cast<unsigned int>(y),
      static_cast<unsigned int>(x_lo),
      static_cast<unsigned int>(x_hi));
  }

  return FillResult::FILLED;
}

/**
 * @brief Apply an action to every cell on a line segment, clipped to the grid.
 *
 * Unlike raytraceLine (which requires in-bounds unsigned endpoints), this
 * walks the full segment in signed integer space with Bresenham's algorithm
 * and applies the action only to cells inside [0, size_x) x [0, size_y),
 * so segments partially or fully outside the grid are safe. Endpoints are
 * given in continuous cell coordinates and rounded to the nearest cell.
 *
 * @param apply_cell Functor called as apply_cell(y, x) with unsigned int
 * cell indices
 */
template<class CellActionType>
inline void forEachLineCell(
  double x0d, double y0d, double x1d, double y1d,
  unsigned int size_x, unsigned int size_y,
  CellActionType apply_cell)
{
  if (!std::isfinite(x0d) || !std::isfinite(y0d) ||
    !std::isfinite(x1d) || !std::isfinite(y1d))
  {
    return;
  }

  int x0 = static_cast<int>(std::lround(x0d));
  int y0 = static_cast<int>(std::lround(y0d));
  const int x1 = static_cast<int>(std::lround(x1d));
  const int y1 = static_cast<int>(std::lround(y1d));

  const int dx = std::abs(x1 - x0);
  const int dy = -std::abs(y1 - y0);
  const int sx = x0 < x1 ? 1 : -1;
  const int sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;

  const int w = static_cast<int>(size_x);
  const int h = static_cast<int>(size_y);

  while (true) {
    if (x0 >= 0 && x0 < w && y0 >= 0 && y0 < h) {
      apply_cell(static_cast<unsigned int>(y0), static_cast<unsigned int>(x0));
    }
    if (x0 == x1 && y0 == y1) {
      break;
    }
    const int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }
}

}  // namespace nav2_util

#endif  // NAV2_UTIL__POLYGON_FILL_2D_HPP_
