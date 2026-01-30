// Copyright (c) 2026 Auto
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

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <array>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "tf2_ros/buffer.hpp"

#include <opencv2/imgcodecs.hpp>

namespace
{
static constexpr const char * global_frame{"map"};

// Save costmap to a PNG file for visualization
bool saveCostmapToPng(
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::string & path)
{
  unsigned int size_x = costmap.getSizeInCellsX();
  unsigned int size_y = costmap.getSizeInCellsY();

  cv::Mat image(size_y, size_x, CV_8UC1);

  for (unsigned int y = 0; y < size_y; ++y) {
    for (unsigned int x = 0; x < size_x; ++x) {
      unsigned char cost = costmap.getCost(x, y);
      image.at<unsigned char>(y, x) = 255 - cost;
    }
  }

  return cv::imwrite(path, image);
}

// Save raw costmap data for comparison
bool saveCostmapRaw(
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::string & path)
{
  std::ofstream file(path, std::ios::binary);
  if (!file) {
    return false;
  }

  unsigned int size_x = costmap.getSizeInCellsX();
  unsigned int size_y = costmap.getSizeInCellsY();

  file.write(reinterpret_cast<const char *>(&size_x), sizeof(size_x));
  file.write(reinterpret_cast<const char *>(&size_y), sizeof(size_y));
  file.write(reinterpret_cast<const char *>(costmap.getCharMap()), size_x * size_y);

  return file.good();
}

// Load raw costmap data for comparison
bool loadCostmapRaw(
  const std::string & path,
  std::vector<unsigned char> & data,
  unsigned int & width,
  unsigned int & height)
{
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    return false;
  }

  file.read(reinterpret_cast<char *>(&width), sizeof(width));
  file.read(reinterpret_cast<char *>(&height), sizeof(height));
  data.resize(width * height);
  file.read(reinterpret_cast<char *>(data.data()), data.size());

  return file.good();
}

// Compare costmap against reference and report differences
void compareCostmaps(
  const nav2_costmap_2d::Costmap2D & costmap,
  const std::vector<unsigned char> & reference,
  unsigned int ref_width,
  unsigned int ref_height)
{
  unsigned int size_x = costmap.getSizeInCellsX();
  unsigned int size_y = costmap.getSizeInCellsY();

  std::cout << "\n=== Comparison ===" << std::endl;

  if (size_x != ref_width || size_y != ref_height) {
    std::cout << "  ERROR: Size mismatch! Current: " << size_x << "x" << size_y
              << ", Reference: " << ref_width << "x" << ref_height << std::endl;
    return;
  }

  const unsigned char * current = costmap.getCharMap();
  size_t total = size_x * size_y;
  size_t diff_count = 0;
  size_t max_diff = 0;
  unsigned int min_x = size_x, min_y = size_y, max_x = 0, max_y = 0;

  size_t diff_ref_free = 0;
  size_t diff_ref_inscribed = 0;
  size_t diff_ref_lethal = 0;
  size_t diff_ref_unknown = 0;
  size_t diff_ref_other = 0;

  size_t diff_up = 0;
  size_t diff_down = 0;
  size_t diff_eq = 0;

  std::array<size_t, 7> diff_bins{};  // 0:1, 1:2-5, 2:6-10, 3:11-20, 4:21-40, 5:41-80, 6:81+

  // Track cost distributions for different cells
  std::map<unsigned char, size_t> ref_cost_counts;  // reference cost -> count

  for (size_t i = 0; i < total; ++i) {
    const unsigned char cur = current[i];
    const unsigned char ref = reference[i];
    if (cur != ref) {
      diff_count++;
      const size_t diff = std::abs(static_cast<int>(cur) - static_cast<int>(ref));
      max_diff = std::max(max_diff, diff);

      const unsigned int x = static_cast<unsigned int>(i % size_x);
      const unsigned int y = static_cast<unsigned int>(i / size_x);
      min_x = std::min(min_x, x);
      min_y = std::min(min_y, y);
      max_x = std::max(max_x, x);
      max_y = std::max(max_y, y);

      // Count reference costs
      ref_cost_counts[ref]++;

      if (ref == nav2_costmap_2d::FREE_SPACE) {
        diff_ref_free++;
      } else if (ref == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        diff_ref_inscribed++;
      } else if (ref == nav2_costmap_2d::LETHAL_OBSTACLE) {
        diff_ref_lethal++;
      } else if (ref == nav2_costmap_2d::NO_INFORMATION) {
        diff_ref_unknown++;
      } else {
        diff_ref_other++;
      }


      if (cur > ref) {
        diff_up++;
      } else if (cur < ref) {
        diff_down++;
      } else {
        diff_eq++;
      }

      if (diff == 1) {
        diff_bins[0]++;
      } else if (diff <= 5) {
        diff_bins[1]++;
      } else if (diff <= 10) {
        diff_bins[2]++;
      } else if (diff <= 20) {
        diff_bins[3]++;
      } else if (diff <= 40) {
        diff_bins[4]++;
      } else if (diff <= 80) {
        diff_bins[5]++;
      } else {
        diff_bins[6]++;
      }
    }
  }

  double diff_percent = 100.0 * diff_count / total;

  std::cout << "  Total cells: " << total << std::endl;
  std::cout << "  Different: " << diff_count << " (" << std::fixed << std::setprecision(4)
            << diff_percent << "%)" << std::endl;
  std::cout << "  Max difference: " << max_diff << std::endl;

  if (diff_count > 0) {
    std::cout << "  Diff bbox: [" << min_x << "," << min_y << "] - ["
              << max_x << "," << max_y << "]" << std::endl;
    std::cout << "  Diff by reference cost: FREE=" << diff_ref_free
              << " INSCRIBED=" << diff_ref_inscribed
              << " LETHAL=" << diff_ref_lethal
              << " UNKNOWN=" << diff_ref_unknown
              << " OTHER=" << diff_ref_other << std::endl;
    std::cout << "  Diff direction: UP=" << diff_up
              << " DOWN=" << diff_down
              << " EQUAL=" << diff_eq << std::endl;
    std::cout << "  Diff magnitude bins:"
              << " 1=" << diff_bins[0]
              << " 2-5=" << diff_bins[1]
              << " 6-10=" << diff_bins[2]
              << " 11-20=" << diff_bins[3]
              << " 21-40=" << diff_bins[4]
              << " 41-80=" << diff_bins[5]
              << " 81+=" << diff_bins[6]
              << std::endl;

    // Show distribution of reference costs for different cells
    std::cout << "  Reference cost distribution (different cells):" << std::endl;

    // Sort by count descending
    std::vector<std::pair<size_t, unsigned char>> sorted_costs;
    for (const auto & [cost, count] : ref_cost_counts) {
      sorted_costs.emplace_back(count, cost);
    }
    std::sort(sorted_costs.rbegin(), sorted_costs.rend());

    // Show top 15
    std::cout << "    ";
    for (size_t i = 0; i < std::min(size_t{15}, sorted_costs.size()); ++i) {
      if (i > 0) {std::cout << ", ";}
      std::cout << "cost_" << static_cast<int>(sorted_costs[i].second)
                << ":" << sorted_costs[i].first;
    }
    std::cout << std::endl;
  }

  if (diff_count == 0) {
    std::cout << "  Result: IDENTICAL" << std::endl;
  } else {
    std::cout << "  Result: DIFFERENT" << std::endl;
  }
}

// Read first OccupancyGrid message from MCAP file
bool readOccupancyGridFromMcap(
  const std::string & mcap_path,
  const std::string & topic,
  nav_msgs::msg::OccupancyGrid & msg)
{
  rosbag2_cpp::Reader reader;
  reader.open(mcap_path);

  rclcpp::Serialization<nav_msgs::msg::OccupancyGrid> serializer;

  while (reader.has_next()) {
    auto bag_msg = reader.read_next();

    if (bag_msg->topic_name == topic) {
      rclcpp::SerializedMessage serialized_msg(*bag_msg->serialized_data);
      serializer.deserialize_message(&serialized_msg, &msg);
      return true;
    }
  }

  return false;
}

}  // namespace

void printUsage(const char * prog_name)
{
  std::cout << "Usage: " << prog_name << " --mcap <path> [options]" << std::endl;
  std::cout << std::endl;
  std::cout << "Reads OccupancyGrid from MCAP and benchmarks inflation layer." << std::endl;
  std::cout << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "  --mcap <path>         MCAP file path (required)" << std::endl;
  std::cout << "  --topic <name>        OccupancyGrid topic in MCAP (default: /map)" << std::endl;
  std::cout << "  --inflation <radius>  Inflation radius in meters (default: 2.0)" << std::endl;
  std::cout << "  --scaling <factor>    Cost scaling factor (default: 3.0)" << std::endl;
  std::cout << "  --output <path>       Save inflated costmap PNG" << std::endl;
  std::cout << "  --save-raw <path>     Save raw costmap data for comparison" << std::endl;
  std::cout << "  --compare <path>      Compare against reference raw file" << std::endl;
  std::cout << "  --iterations <n>      Number of benchmark iterations (default: 10)" << std::endl;
  std::cout << "  --roi-size <cells>    Square ROI size from origin (default: full map)" <<
    std::endl;
  std::cout << "  --help                Show this help message" << std::endl;
}

int main(int argc, char ** argv)
{
  std::string mcap_path;
  std::string map_topic = "/map";
  double inflation_radius = 2.0;
  double cost_scaling_factor = 3.0;
  std::string output_path;
  std::string save_raw_path;
  std::string compare_path;
  int iterations = 10;
  int roi_size = -1;  // Negative means use full map

  // Parse arguments
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];

    if (arg == "--mcap" && i + 1 < argc) {
      mcap_path = argv[++i];
    } else if (arg == "--topic" && i + 1 < argc) {
      map_topic = argv[++i];
    } else if (arg == "--inflation" && i + 1 < argc) {
      inflation_radius = std::stod(argv[++i]);
    } else if (arg == "--scaling" && i + 1 < argc) {
      cost_scaling_factor = std::stod(argv[++i]);
    } else if (arg == "--output" && i + 1 < argc) {
      output_path = argv[++i];
    } else if (arg == "--save-raw" && i + 1 < argc) {
      save_raw_path = argv[++i];
    } else if (arg == "--compare" && i + 1 < argc) {
      compare_path = argv[++i];
    } else if (arg == "--iterations" && i + 1 < argc) {
      iterations = std::stoi(argv[++i]);
    } else if (arg == "--roi-size" && i + 1 < argc) {
      roi_size = std::stoi(argv[++i]);
    } else if (arg == "--help" || arg == "-h") {
      printUsage(argv[0]);
      return 0;
    }
  }

  if (mcap_path.empty()) {
    std::cerr << "Error: --mcap <path> is required" << std::endl;
    printUsage(argv[0]);
    return 1;
  }

  // Read OccupancyGrid from MCAP
  std::cout << "Reading OccupancyGrid from: " << mcap_path << std::endl;
  std::cout << "Topic: " << map_topic << std::endl;

  nav_msgs::msg::OccupancyGrid map_msg;
  if (!readOccupancyGridFromMcap(mcap_path, map_topic, map_msg)) {
    std::cerr << "Failed to find OccupancyGrid on topic " << map_topic << " in " << mcap_path
              << std::endl;
    return 1;
  }

  // Extract map info
  unsigned int width = map_msg.info.width;
  unsigned int height = map_msg.info.height;
  double resolution = map_msg.info.resolution;

  // Convert OccupancyGrid data to costmap format
  std::vector<unsigned char> map_data(width * height);
  size_t obstacle_count = 0;

  for (size_t i = 0; i < map_msg.data.size(); ++i) {
    int8_t value = map_msg.data[i];
    if (value < 0) {
      map_data[i] = nav2_costmap_2d::NO_INFORMATION;
    } else if (value >= 65) {
      map_data[i] = nav2_costmap_2d::LETHAL_OBSTACLE;
      obstacle_count++;
    } else {
      map_data[i] = nav2_costmap_2d::FREE_SPACE;
    }
  }

  std::cout << "Map: " << width << "x" << height
            << " (" << width * height << " cells, "
            << obstacle_count << " obstacles, "
            << "res=" << resolution << "m/cell)" << std::endl;

  // Initialize ROS
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions();
  std::vector<rclcpp::Parameter> parameters;
  parameters.push_back(rclcpp::Parameter("inflation.cost_scaling_factor", cost_scaling_factor));
  parameters.push_back(rclcpp::Parameter("inflation.inflation_radius", inflation_radius));
  options.parameter_overrides(parameters);

  auto node = std::make_shared<nav2::LifecycleNode>("inflation_benchmark_node", "", options);
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());

  // Create layered costmap
  nav2_costmap_2d::LayeredCostmap layers(global_frame, false, false);
  layers.resizeMap(width, height, resolution, 0.0, 0.0);

  // Copy map data
  nav2_costmap_2d::Costmap2D * costmap = layers.getCostmap();
  unsigned char * costmap_data = costmap->getCharMap();
  std::copy(map_data.begin(), map_data.end(), costmap_data);

  // Initialize inflation layer
  auto ilayer = std::make_shared<nav2_costmap_2d::InflationLayer>();
  ilayer->initialize(&layers, "inflation", tf_buffer.get(), node, nullptr);
  layers.addPlugin(std::shared_ptr<nav2_costmap_2d::Layer>(ilayer));

  // Compute ROI bounds
  int roi_min_i = 0;
  int roi_min_j = 0;
  int roi_max_i = width;
  int roi_max_j = height;

  if (roi_size > 0) {
    // Square ROI from origin
    roi_max_i = std::min(static_cast<int>(width), roi_size);
    roi_max_j = std::min(static_cast<int>(height), roi_size);
  }

  const int actual_roi_width = roi_max_i - roi_min_i;
  const int actual_roi_height = roi_max_j - roi_min_j;
  const double actual_roi_percent = (100.0 * actual_roi_width * actual_roi_height) /
    (width * height);

  if (roi_size > 0) {
    std::cout << "ROI: " << actual_roi_width << "x" << actual_roi_height
              << " from origin (" << (actual_roi_width * actual_roi_height) << " cells, "
              << std::fixed << std::setprecision(2) << actual_roi_percent << "%)" << std::endl;
  } else {
    std::cout << "ROI: Full map (" << (width * height) << " cells)" << std::endl;
  }

  // Benchmark
  std::vector<double> times;
  times.reserve(iterations);

  std::cout << "Running " << iterations << " iterations..." << std::endl;

  for (int i = 0; i < iterations; ++i) {
    std::copy(map_data.begin(), map_data.end(), costmap_data);

    auto start = std::chrono::high_resolution_clock::now();
    ilayer->updateCosts(*costmap, roi_min_i, roi_min_j, roi_max_i, roi_max_j);
    auto end = std::chrono::high_resolution_clock::now();

    double ms = std::chrono::duration<double, std::milli>(end - start).count();
    times.push_back(ms);
    std::cout << "  Run " << (i + 1) << ": " << std::fixed << std::setprecision(2) << ms << " ms"
              << std::endl;
  }

  // Statistics
  double sum = 0.0, min_time = times[0], max_time = times[0];
  for (double t : times) {
    sum += t;
    min_time = std::min(min_time, t);
    max_time = std::max(max_time, t);
  }
  double mean = sum / times.size();

  double variance = 0.0;
  for (double t : times) {
    variance += (t - mean) * (t - mean);
  }
  double stddev = std::sqrt(variance / times.size());

  std::cout << "\n=== Results ===" << std::endl;
  std::cout << "  Map size: " << width << " x " << height << " (" << (width * height) << " cells)"
            << std::endl;
  std::cout << "  ROI size: " << actual_roi_width << " x " << actual_roi_height
            << " (" << (actual_roi_width * actual_roi_height) << " cells, "
            << std::fixed << std::setprecision(2) << actual_roi_percent << "%)" << std::endl;
  std::cout << "  Mean: " << std::fixed << std::setprecision(2) << mean << " ms" << std::endl;
  std::cout << "  Std dev: " << std::fixed << std::setprecision(2) << stddev << " ms" << std::endl;
  std::cout << "  Min: " << std::fixed << std::setprecision(2) << min_time << " ms" << std::endl;
  std::cout << "  Max: " << std::fixed << std::setprecision(2) << max_time << " ms" << std::endl;
  std::cout << "  Throughput: " << std::fixed << std::setprecision(2)
            << (width * height / mean / 1000.0) << " M cells/ms" << std::endl;

  // Save output if requested
  if (!output_path.empty()) {
    if (saveCostmapToPng(*costmap, output_path)) {
      std::cout << "\nSaved inflated costmap to: " << output_path << std::endl;
    }
  }

  // Save raw data if requested
  if (!save_raw_path.empty()) {
    if (saveCostmapRaw(*costmap, save_raw_path)) {
      std::cout << "Saved raw costmap to: " << save_raw_path << std::endl;
    }
  }

  // Compare against reference if requested
  if (!compare_path.empty()) {
    std::vector<unsigned char> reference;
    unsigned int ref_width = 0, ref_height = 0;
    if (loadCostmapRaw(compare_path, reference, ref_width, ref_height)) {
      compareCostmaps(*costmap, reference, ref_width, ref_height);
    } else {
      std::cerr << "Failed to load reference file: " << compare_path << std::endl;
    }
  }

  rclcpp::shutdown();
  return 0;
}
