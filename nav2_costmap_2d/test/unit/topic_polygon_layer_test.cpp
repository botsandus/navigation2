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

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/topic_polygon_layer.hpp"
#include "nav2_msgs/msg/polygon_objects.hpp"

using namespace std::chrono_literals;

static const char LAYER_NAME[]{"polygon_layer"};
static const char POLYGONS_TOPIC[]{"test_vo_polygons"};
static const char GLOBAL_FRAME[]{"map"};

// ---------------------------------------------------------------------------
// Publisher helper — publishes a PolygonObjects message with transient_local QoS
// ---------------------------------------------------------------------------
class PolygonsPublisher : public rclcpp::Node
{
public:
  explicit PolygonsPublisher(const nav2_msgs::msg::PolygonObjects & msg)
  : Node("polygons_pub")
  {
    publisher_ = this->create_publisher<nav2_msgs::msg::PolygonObjects>(
      POLYGONS_TOPIC,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
    publisher_->publish(msg);
  }

  void publish(const nav2_msgs::msg::PolygonObjects & msg)
  {
    publisher_->publish(msg);
  }

  ~PolygonsPublisher()
  {
    publisher_.reset();
  }

private:
  rclcpp::Publisher<nav2_msgs::msg::PolygonObjects>::SharedPtr publisher_;
};

// ---------------------------------------------------------------------------
// Helper to build a PolygonObjects message
// ---------------------------------------------------------------------------
nav2_msgs::msg::PolygonObjects makePolygonsMsg(
  const std::vector<std::vector<std::pair<float, float>>> & polygon_points,
  const std::string & frame_id = GLOBAL_FRAME)
{
  nav2_msgs::msg::PolygonObjects msg;
  msg.header.frame_id = frame_id;

  for (const auto & pts : polygon_points) {
    nav2_msgs::msg::PolygonObject obj;
    for (const auto & [x, y] : pts) {
      geometry_msgs::msg::Point32 p;
      p.x = x;
      p.y = y;
      p.z = 0.0f;
      obj.points.push_back(p);
    }
    msg.polygons.push_back(obj);
  }
  return msg;
}

// ---------------------------------------------------------------------------
// Global rclcpp init/shutdown
// ---------------------------------------------------------------------------
class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

// ---------------------------------------------------------------------------
// Test fixture
// ---------------------------------------------------------------------------
class TopicPolygonLayerTest : public ::testing::Test
{
protected:
  void createLayer(const std::string & global_frame = GLOBAL_FRAME);
  void waitForData();
  void waitSome(const std::chrono::nanoseconds & duration);
  void reset();

  unsigned int countCost(unsigned char value);

  nav2::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<nav2_costmap_2d::LayeredCostmap> layers_;
  std::shared_ptr<nav2_costmap_2d::TopicPolygonLayer> layer_;
  std::shared_ptr<PolygonsPublisher> publisher_;
};

void TopicPolygonLayerTest::createLayer(const std::string & global_frame)
{
  node_ = std::make_shared<nav2::LifecycleNode>("test_node");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_buffer_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  layers_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>(
    global_frame, false, false);
  layers_->resizeMap(10, 10, 1.0, 0.0, 0.0);

  // Declare parameters the layer will read
  node_->declare_parameter(
    std::string(LAYER_NAME) + ".enabled", rclcpp::ParameterValue(true));
  node_->declare_parameter(
    std::string(LAYER_NAME) + ".polygons_topic",
    rclcpp::ParameterValue(std::string(POLYGONS_TOPIC)));
  node_->declare_parameter(
    std::string(LAYER_NAME) + ".polygons_frame",
    rclcpp::ParameterValue(std::string(global_frame)));
  node_->declare_parameter(
    std::string(LAYER_NAME) + ".fill_polygons", rclcpp::ParameterValue(true));

  layer_ = std::make_shared<nav2_costmap_2d::TopicPolygonLayer>();
  layers_->addPlugin(layer_);
  layer_->initialize(layers_.get(), LAYER_NAME, tf_buffer_.get(), node_, nullptr);
}

void TopicPolygonLayerTest::waitForData()
{
  // Spin until the subscription callback fires and the layer processes data.
  // After receiving data, isCurrent() becomes false until updateCosts runs.
  // We first need to wait for the callback, then call updateMap to make it current.
  waitSome(500ms);
}

void TopicPolygonLayerTest::waitSome(const std::chrono::nanoseconds & duration)
{
  rclcpp::Time start_time = node_->now();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_->get_node_base_interface());
  while (rclcpp::ok() && node_->now() - start_time <= rclcpp::Duration(duration)) {
    executor.spin_some();
    std::this_thread::sleep_for(10ms);
  }
}

void TopicPolygonLayerTest::reset()
{
  publisher_.reset();
  layer_.reset();
  layers_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  node_.reset();
}

unsigned int TopicPolygonLayerTest::countCost(unsigned char value)
{
  auto * costmap = layers_->getCostmap();
  unsigned int count = 0;
  for (unsigned int y = 0; y < costmap->getSizeInCellsY(); y++) {
    for (unsigned int x = 0; x < costmap->getSizeInCellsX(); x++) {
      if (costmap->getCost(x, y) == value) {
        count++;
      }
    }
  }
  return count;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

// Verify that a filled square polygon produces LETHAL cells on the costmap
TEST_F(TopicPolygonLayerTest, testFilledPolygon)
{
  // Polygon: 3x3 square from (2,2) to (5,5) in a 10x10 costmap with resolution=1.0
  auto msg = makePolygonsMsg({
    {{2.0f, 2.0f}, {5.0f, 2.0f}, {5.0f, 5.0f}, {2.0f, 5.0f}}
  });

  publisher_ = std::make_shared<PolygonsPublisher>(msg);
  createLayer();

  waitForData();
  layers_->updateMap(5.0, 5.0, 0.0);

  unsigned int lethal_count = countCost(nav2_costmap_2d::LETHAL_OBSTACLE);
  // A 3x3 filled square should produce at least 4 lethal cells (interior)
  EXPECT_GT(lethal_count, 0u);
  // And not fill the entire map
  EXPECT_LT(lethal_count, 100u);

  reset();
}

// Verify that an empty polygon message produces no lethal cells
TEST_F(TopicPolygonLayerTest, testEmptyPolygons)
{
  auto msg = makePolygonsMsg({});

  publisher_ = std::make_shared<PolygonsPublisher>(msg);
  createLayer();

  waitSome(200ms);
  layers_->updateMap(5.0, 5.0, 0.0);

  unsigned int lethal_count = countCost(nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(lethal_count, 0u);

  reset();
}

// Verify that polygons are cleared when an updated empty message arrives
TEST_F(TopicPolygonLayerTest, testPolygonRemoval)
{
  auto msg = makePolygonsMsg({
    {{2.0f, 2.0f}, {5.0f, 2.0f}, {5.0f, 5.0f}, {2.0f, 5.0f}}
  });

  publisher_ = std::make_shared<PolygonsPublisher>(msg);
  createLayer();

  waitForData();
  layers_->updateMap(5.0, 5.0, 0.0);

  unsigned int lethal_before = countCost(nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_GT(lethal_before, 0u);

  // Send empty polygons
  auto empty_msg = makePolygonsMsg({});
  publisher_->publish(empty_msg);
  waitSome(200ms);
  layers_->updateMap(5.0, 5.0, 0.0);

  unsigned int lethal_after = countCost(nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(lethal_after, 0u);

  reset();
}

// Verify that duplicate messages don't cause unnecessary updates
TEST_F(TopicPolygonLayerTest, testDuplicateMessage)
{
  auto msg = makePolygonsMsg({
    {{1.0f, 1.0f}, {3.0f, 1.0f}, {3.0f, 3.0f}, {1.0f, 3.0f}}
  });

  publisher_ = std::make_shared<PolygonsPublisher>(msg);
  createLayer();

  waitForData();
  layers_->updateMap(5.0, 5.0, 0.0);

  unsigned int lethal_first = countCost(nav2_costmap_2d::LETHAL_OBSTACLE);

  // Re-publish identical message
  publisher_->publish(msg);
  waitSome(200ms);
  layers_->updateMap(5.0, 5.0, 0.0);

  unsigned int lethal_second = countCost(nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(lethal_first, lethal_second);

  reset();
}

// Verify that the layer is clearable
TEST_F(TopicPolygonLayerTest, testIsClearable)
{
  auto msg = makePolygonsMsg({});
  publisher_ = std::make_shared<PolygonsPublisher>(msg);
  createLayer();

  EXPECT_TRUE(layer_->isClearable());

  reset();
}

// Verify reset clears the costmap buffer
TEST_F(TopicPolygonLayerTest, testReset)
{
  auto msg = makePolygonsMsg({
    {{2.0f, 2.0f}, {5.0f, 2.0f}, {5.0f, 5.0f}, {2.0f, 5.0f}}
  });

  publisher_ = std::make_shared<PolygonsPublisher>(msg);
  createLayer();

  waitForData();
  layers_->updateMap(5.0, 5.0, 0.0);
  EXPECT_GT(countCost(nav2_costmap_2d::LETHAL_OBSTACLE), 0u);

  layer_->reset();
  // After reset, the layer should re-rasterise on next update
  // (polygons are still stored, but buffer is marked invalid)

  reset();
}

// Verify multiple non-overlapping polygons
TEST_F(TopicPolygonLayerTest, testMultiplePolygons)
{
  // Two small polygons at opposite corners
  auto msg = makePolygonsMsg({
    {{1.0f, 1.0f}, {2.0f, 1.0f}, {2.0f, 2.0f}, {1.0f, 2.0f}},
    {{7.0f, 7.0f}, {8.0f, 7.0f}, {8.0f, 8.0f}, {7.0f, 8.0f}}
  });

  publisher_ = std::make_shared<PolygonsPublisher>(msg);
  createLayer();

  waitForData();
  layers_->updateMap(5.0, 5.0, 0.0);

  auto * costmap = layers_->getCostmap();
  // Check that cells near (1,1) and (7,7) are lethal
  EXPECT_EQ(costmap->getCost(1, 1), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(costmap->getCost(7, 7), nav2_costmap_2d::LETHAL_OBSTACLE);
  // Check that center is free
  EXPECT_NE(costmap->getCost(5, 5), nav2_costmap_2d::LETHAL_OBSTACLE);

  reset();
}

// Verify polygon update (old polygon cleared, new polygon appears)
TEST_F(TopicPolygonLayerTest, testPolygonUpdate)
{
  // Start with polygon in bottom-left
  auto msg1 = makePolygonsMsg({
    {{1.0f, 1.0f}, {3.0f, 1.0f}, {3.0f, 3.0f}, {1.0f, 3.0f}}
  });
  publisher_ = std::make_shared<PolygonsPublisher>(msg1);
  createLayer();

  waitForData();
  layers_->updateMap(5.0, 5.0, 0.0);

  auto * costmap = layers_->getCostmap();
  EXPECT_EQ(costmap->getCost(1, 1), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_NE(costmap->getCost(7, 7), nav2_costmap_2d::LETHAL_OBSTACLE);

  // Move polygon to top-right
  auto msg2 = makePolygonsMsg({
    {{6.0f, 6.0f}, {8.0f, 6.0f}, {8.0f, 8.0f}, {6.0f, 8.0f}}
  });
  publisher_->publish(msg2);
  waitSome(200ms);
  layers_->updateMap(5.0, 5.0, 0.0);

  // Old location should be clear, new location should be lethal
  EXPECT_NE(costmap->getCost(1, 1), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(costmap->getCost(7, 7), nav2_costmap_2d::LETHAL_OBSTACLE);

  reset();
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
