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

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "tf2_ros/buffer.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/vector_object_layer.hpp"
#include "nav2_msgs/msg/circle_object.hpp"
#include "nav2_msgs/msg/polygon_object.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"

using nav2_msgs::msg::CircleObject;
using nav2_msgs::msg::PolygonObject;

// Expose the protected shape setter for direct-injection testing
class TestableLayer : public nav2_costmap_2d::VectorObjectLayer
{
public:
  using VectorObjectLayer::setVectorObjects;
};

static unsigned int countValues(
  nav2_costmap_2d::Costmap2D & costmap,
  unsigned char value)
{
  unsigned int count = 0;
  for (unsigned int y = 0; y < costmap.getSizeInCellsY(); ++y) {
    for (unsigned int x = 0; x < costmap.getSizeInCellsX(); ++x) {
      if (costmap.getCost(x, y) == value) {
        ++count;
      }
    }
  }
  return count;
}

/// @brief Build a PolygonObject from vertex pairs (open ring, no duplicate endpoint)
static PolygonObject makePolygon(
  const std::vector<std::pair<float, float>> & pts,
  bool closed = true, int8_t value = 100, const std::string & frame = "")
{
  PolygonObject poly;
  poly.header.frame_id = frame;
  poly.closed = closed;
  poly.value = value;
  poly.points.resize(pts.size());
  for (size_t i = 0; i < pts.size(); ++i) {
    poly.points[i].x = pts[i].first;
    poly.points[i].y = pts[i].second;
  }
  return poly;
}

class VectorObjectLayerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Unique node names prevent parameter collisions between tests
    static int node_id = 0;
    node_ = std::make_shared<nav2::LifecycleNode>(
      "test_vo_layer_" + std::to_string(node_id++));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());

    layers_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    layer_ = std::make_shared<TestableLayer>();

    layers_->addPlugin(std::shared_ptr<nav2_costmap_2d::Layer>(layer_));
    layer_->initialize(
      layers_.get(), "test_layer", tf_buffer_.get(), node_, nullptr);

    // 100x100 cells at 0.05 m/cell -> 5 m x 5 m, origin at (0,0)
    layers_->resizeMap(100, 100, 0.05, 0.0, 0.0);
  }

  /// Run a full updateBounds + updateCosts cycle
  void updateCycle()
  {
    double minx = 1e9, miny = 1e9, maxx = -1e9, maxy = -1e9;
    layer_->updateBounds(0.0, 0.0, 0.0, &minx, &miny, &maxx, &maxy);

    auto * master = layers_->getCostmap();
    layer_->updateCosts(
      *master, 0, 0,
      static_cast<int>(master->getSizeInCellsX()),
      static_cast<int>(master->getSizeInCellsY()));
  }

  nav2::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::LayeredCostmap> layers_;
  std::shared_ptr<TestableLayer> layer_;
};

TEST_F(VectorObjectLayerTest, InitialStateIsClean)
{
  updateCycle();
  auto * master = layers_->getCostmap();
  EXPECT_EQ(countValues(*master, nav2_costmap_2d::LETHAL_OBSTACLE), 0u);
}

TEST_F(VectorObjectLayerTest, IsClearable)
{
  EXPECT_TRUE(layer_->isClearable());
}

TEST_F(VectorObjectLayerTest, SingleRectangleProducesLethalCells)
{
  // A 1m x 1m rectangle at (1,1)-(2,2) on a 5m x 5m costmap at 0.05 res
  auto poly = makePolygon({{1.0f, 1.0f}, {2.0f, 1.0f}, {2.0f, 2.0f}, {1.0f, 2.0f}});
  layer_->setVectorObjects({poly}, {});
  updateCycle();

  auto * master = layers_->getCostmap();
  unsigned int lethal = countValues(*master, nav2_costmap_2d::LETHAL_OBSTACLE);
  // 1m / 0.05 = 20 cells per side -> ~20x20 = ~400 interior cells
  EXPECT_GT(lethal, 300u);
  EXPECT_LT(lethal, 500u);

  unsigned int mx, my;
  ASSERT_TRUE(master->worldToMap(1.5, 1.5, mx, my));
  EXPECT_EQ(master->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);

  ASSERT_TRUE(master->worldToMap(0.5, 0.5, mx, my));
  EXPECT_NE(master->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(VectorObjectLayerTest, MultiplePolygonsRasterised)
{
  auto poly_a = makePolygon({{0.5f, 0.5f}, {1.0f, 0.5f}, {1.0f, 1.0f}, {0.5f, 1.0f}});
  auto poly_b = makePolygon({{3.0f, 3.0f}, {3.5f, 3.0f}, {3.5f, 3.5f}, {3.0f, 3.5f}});
  layer_->setVectorObjects({poly_a, poly_b}, {});
  updateCycle();

  auto * master = layers_->getCostmap();
  unsigned int mx, my;

  ASSERT_TRUE(master->worldToMap(0.75, 0.75, mx, my));
  EXPECT_EQ(master->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);

  ASSERT_TRUE(master->worldToMap(3.25, 3.25, mx, my));
  EXPECT_EQ(master->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);

  ASSERT_TRUE(master->worldToMap(2.0, 2.0, mx, my));
  EXPECT_NE(master->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(VectorObjectLayerTest, EmptyShapesClearsLayer)
{
  auto poly = makePolygon({{1.0f, 1.0f}, {2.0f, 1.0f}, {2.0f, 2.0f}, {1.0f, 2.0f}});
  layer_->setVectorObjects({poly}, {});
  updateCycle();

  auto * master = layers_->getCostmap();
  ASSERT_GT(countValues(*master, nav2_costmap_2d::LETHAL_OBSTACLE), 0u);

  layer_->setVectorObjects({}, {});
  updateCycle();

  // Verify the layer's own buffer was cleared (the master merges with max)
  EXPECT_EQ(countValues(*layer_, nav2_costmap_2d::LETHAL_OBSTACLE), 0u);
}

TEST_F(VectorObjectLayerTest, IdenticalShapesSkipRerasterisation)
{
  auto poly = makePolygon({{1.0f, 1.0f}, {2.0f, 1.0f}, {2.0f, 2.0f}, {1.0f, 2.0f}});

  layer_->setVectorObjects({poly}, {});
  updateCycle();

  // Setting an identical shape set must not drop currency
  layer_->setVectorObjects({poly}, {});
  EXPECT_TRUE(layer_->isCurrent());
}

TEST_F(VectorObjectLayerTest, ShapeReplacementClearsOldCells)
{
  auto poly_old = makePolygon({{0.5f, 0.5f}, {1.5f, 0.5f}, {1.5f, 1.5f}, {0.5f, 1.5f}});
  layer_->setVectorObjects({poly_old}, {});
  updateCycle();

  auto poly_new = makePolygon({{3.0f, 3.0f}, {4.0f, 3.0f}, {4.0f, 4.0f}, {3.0f, 4.0f}});
  layer_->setVectorObjects({poly_new}, {});
  updateCycle();

  unsigned int mx, my;
  ASSERT_TRUE(layer_->worldToMap(1.0, 1.0, mx, my));
  EXPECT_NE(layer_->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);

  ASSERT_TRUE(layer_->worldToMap(3.5, 3.5, mx, my));
  EXPECT_EQ(layer_->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(VectorObjectLayerTest, ResetInvalidatesBufferButKeepsShapes)
{
  auto poly = makePolygon({{1.0f, 1.0f}, {2.0f, 1.0f}, {2.0f, 2.0f}, {1.0f, 2.0f}});
  layer_->setVectorObjects({poly}, {});
  updateCycle();

  unsigned int mx, my;
  ASSERT_TRUE(layer_->worldToMap(1.5, 1.5, mx, my));
  EXPECT_EQ(layer_->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);

  layer_->reset();
  EXPECT_NE(layer_->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);

  // Shapes are retained: they re-appear on the next cycle
  updateCycle();
  EXPECT_EQ(layer_->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(VectorObjectLayerTest, DisabledLayerDoesNotRasterise)
{
  auto poly = makePolygon({{1.0f, 1.0f}, {2.0f, 1.0f}, {2.0f, 2.0f}, {1.0f, 2.0f}});
  layer_->setVectorObjects({poly}, {});

  node_->set_parameter(rclcpp::Parameter("test_layer.enabled", false));
  updateCycle();

  auto * master = layers_->getCostmap();
  EXPECT_EQ(countValues(*master, nav2_costmap_2d::LETHAL_OBSTACLE), 0u);
}

TEST_F(VectorObjectLayerTest, ReenablingLayer)
{
  auto poly = makePolygon({{1.0f, 1.0f}, {2.0f, 1.0f}, {2.0f, 2.0f}, {1.0f, 2.0f}});
  layer_->setVectorObjects({poly}, {});

  node_->set_parameter(rclcpp::Parameter("test_layer.enabled", false));
  updateCycle();
  auto * master = layers_->getCostmap();
  EXPECT_EQ(countValues(*master, nav2_costmap_2d::LETHAL_OBSTACLE), 0u);

  node_->set_parameter(rclcpp::Parameter("test_layer.enabled", true));
  updateCycle();
  EXPECT_GT(countValues(*master, nav2_costmap_2d::LETHAL_OBSTACLE), 0u);
}

TEST_F(VectorObjectLayerTest, TrianglePolygon)
{
  auto poly = makePolygon({{1.0f, 1.0f}, {3.0f, 1.0f}, {2.0f, 3.0f}});
  layer_->setVectorObjects({poly}, {});
  updateCycle();

  auto * master = layers_->getCostmap();
  EXPECT_GT(countValues(*master, nav2_costmap_2d::LETHAL_OBSTACLE), 0u);

  unsigned int mx, my;
  ASSERT_TRUE(master->worldToMap(2.0, 1.5, mx, my));
  EXPECT_EQ(master->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(VectorObjectLayerTest, PolygonOutsideCostmapBounds)
{
  // Costmap is 0..5m; polygon is entirely outside
  auto poly = makePolygon({{10.0f, 10.0f}, {12.0f, 10.0f}, {12.0f, 12.0f}, {10.0f, 12.0f}});
  layer_->setVectorObjects({poly}, {});
  updateCycle();

  auto * master = layers_->getCostmap();
  EXPECT_EQ(countValues(*master, nav2_costmap_2d::LETHAL_OBSTACLE), 0u);
}

TEST_F(VectorObjectLayerTest, OutlinePolygonDrawsBorderOnly)
{
  // Open chain with the first vertex repeated: 1-cell-wide closed outline
  auto poly = makePolygon(
    {{1.0f, 1.0f}, {3.0f, 1.0f}, {3.0f, 3.0f}, {1.0f, 3.0f}, {1.0f, 1.0f}},
    /*closed=*/false);
  layer_->setVectorObjects({poly}, {});
  updateCycle();

  auto * master = layers_->getCostmap();
  unsigned int mx, my;

  // On the border
  ASSERT_TRUE(master->worldToMap(2.0, 1.0, mx, my));
  EXPECT_EQ(master->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);

  // Interior stays free
  ASSERT_TRUE(master->worldToMap(2.0, 2.0, mx, my));
  EXPECT_NE(master->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);

  // Outline of a 2m square at 0.05 res: perimeter ~160 cells, far below fill (~1600)
  unsigned int lethal = countValues(*master, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_GT(lethal, 100u);
  EXPECT_LT(lethal, 400u);
}

TEST_F(VectorObjectLayerTest, PerShapeValueMapsToCost)
{
  auto poly = makePolygon(
    {{1.0f, 1.0f}, {2.0f, 1.0f}, {2.0f, 2.0f}, {1.0f, 2.0f}},
    /*closed=*/true, /*value=*/50);
  layer_->setVectorObjects({poly}, {});
  updateCycle();

  // 50/100 * LETHAL_OBSTACLE(254) = 127
  auto * master = layers_->getCostmap();
  unsigned int mx, my;
  ASSERT_TRUE(master->worldToMap(1.5, 1.5, mx, my));
  EXPECT_EQ(master->getCost(mx, my), 127);
  EXPECT_EQ(countValues(*master, nav2_costmap_2d::LETHAL_OBSTACLE), 0u);
}

TEST_F(VectorObjectLayerTest, FilledCircle)
{
  CircleObject circle;
  circle.center.x = 2.5F;
  circle.center.y = 2.5F;
  circle.radius = 1.0F;
  circle.fill = true;
  circle.value = 100;
  layer_->setVectorObjects({}, {circle});
  updateCycle();

  auto * master = layers_->getCostmap();
  unsigned int mx, my;

  ASSERT_TRUE(master->worldToMap(2.5, 2.5, mx, my));
  EXPECT_EQ(master->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);

  // Point outside the radius
  ASSERT_TRUE(master->worldToMap(4.0, 4.0, mx, my));
  EXPECT_NE(master->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);

  // Area of r=1m circle at 0.05 res: pi * 20^2 ~ 1257 cells
  unsigned int lethal = countValues(*master, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_GT(lethal, 1100u);
  EXPECT_LT(lethal, 1400u);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
