# Nav2 Costmap 2D Benchmarks

This directory contains performance benchmarks for various components of the nav2_costmap_2d package.

## Available Benchmarks

### 1. Observation Buffer Benchmark
Tests the performance of the ObservationBuffer class for handling point cloud observations.

**File:** `observation_buffer_benchmark.cpp`

### 2. Inflation Layer UpdateCosts Benchmark
Tests the performance of the InflationLayer's `updateCosts` function with various map sizes and obstacle densities.

**File:** `inflation_layer_updatecosts_benchmark.cpp`

## Building the Benchmarks

From your workspace root:

```bash
colcon build --packages-select nav2_costmap_2d --cmake-args -DBUILD_TESTING=ON
```

The benchmarks will be built in the `build/nav2_costmap_2d/test/benchmark/` directory.

## Running the Inflation Layer UpdateCosts Benchmark

### Predefined Scenarios

Run all predefined scenarios with various map sizes and occupancy levels:

```bash
./build/nav2_costmap_2d/test/benchmark/inflation_layer_updatecosts_benchmark
```

### Custom Benchmarks

Run a custom benchmark with specific parameters:

```bash
./build/nav2_costmap_2d/test/benchmark/inflation_layer_updatecosts_benchmark \
  --custom \
  --width=2000 \
  --height=2000 \
  --occupancy=15 \
  --inflation=0.55
```

**Parameters:**
- `--custom`: Enable custom benchmark mode
- `--width=<N>`: Map width in cells (default: 1000)
- `--height=<N>`: Map height in cells (default: 1000)
- `--occupancy=<N>`: Obstacle occupancy percentage 0-100 (default: 10)
- `--inflation=<N>`: Inflation radius in meters (default: 0.55)

### Filtering Benchmarks

Run only specific benchmark scenarios using regex filtering:

```bash
# Run only small map benchmarks (100x100)
./build/nav2_costmap_2d/test/benchmark/inflation_layer_updatecosts_benchmark \
  --benchmark_filter="100/100"

# Run only benchmarks with 10% occupancy
./build/nav2_costmap_2d/test/benchmark/inflation_layer_updatecosts_benchmark \
  --benchmark_filter="/10"

# Run only medium-sized maps (500x500)
./build/nav2_costmap_2d/test/benchmark/inflation_layer_updatecosts_benchmark \
  --benchmark_filter="500/500"
```

### Output Formats

Save results to a file in various formats:

```bash
# CSV format
./build/nav2_costmap_2d/test/benchmark/inflation_layer_updatecosts_benchmark \
  --benchmark_format=csv \
  --benchmark_out=results.csv

# JSON format
./build/nav2_costmap_2d/test/benchmark/inflation_layer_updatecosts_benchmark \
  --benchmark_format=json \
  --benchmark_out=results.json
```

### Multiple Repetitions

Run benchmarks multiple times for statistical significance:

```bash
./build/nav2_costmap_2d/test/benchmark/inflation_layer_updatecosts_benchmark \
  --benchmark_repetitions=10
```

### Control Benchmark Duration

Specify minimum time to run each benchmark:

```bash
./build/nav2_costmap_2d/test/benchmark/inflation_layer_updatecosts_benchmark \
  --benchmark_min_time=5.0
```

## Predefined Scenarios

The inflation layer benchmark includes the following predefined scenarios:

### Small Maps (100x100 cells)
- 5%, 10%, 20%, 30% occupancy

### Medium Maps (500x500 cells)
- 5%, 10%, 20%, 30% occupancy

### Large Maps (1000x1000 cells)
- 5%, 10%, 20%, 30% occupancy

### Very Large Maps (2000x2000 cells)
- 10%, 20% occupancy

### Rectangular Maps (non-square)
- 1000x500 (wide), 500x1000 (tall) with 10% occupancy

## Understanding the Results

The benchmark reports several metrics:

- **Time**: Time taken to execute `updateCosts` (milliseconds)
- **cells**: Total number of cells in the costmap
- **occupied**: Number of occupied cells (obstacles)
- **occupancy_%**: Percentage of cells that are occupied
- **width/height**: Dimensions of the costmap
- **cells/s**: Processing rate in cells per second

### Example Output

```
Run on (8 X 3600 MHz CPU s)
CPU Caches:
  L1 Data 32 KiB (x4)
  L1 Instruction 32 KiB (x4)
  L2 Unified 256 KiB (x4)
  L3 Unified 8192 KiB (x1)
-------------------------------------------------------------------------------
Benchmark                           Time             CPU   Iterations UserCounters...
-------------------------------------------------------------------------------
UpdateCosts/100/100/5           0.342 ms        0.341 ms         2048 cells=10000 occupied=500 occupancy_%=5 width=100 height=100 cells/s=29.3M/s
UpdateCosts/100/100/10          0.512 ms        0.511 ms         1365 cells=10000 occupied=1000 occupancy_%=10 width=100 height=100 cells/s=19.6M/s
UpdateCosts/500/500/10          12.3 ms         12.3 ms           57 cells=250000 occupied=25000 occupancy_%=10 width=500 height=500 cells/s=20.3M/s
UpdateCosts/1000/1000/10        48.9 ms         48.8 ms           14 cells=1000000 occupied=100000 occupancy_%=10 width=1000 height=1000 cells/s=20.5M/s
```

### Custom Benchmark Output

When running in custom mode (`--custom`), you'll see detailed statistics:

```
========================================
Custom Inflation Layer Benchmark Results
========================================
Configuration:
  Map size: 2000 x 2000 cells
  Total cells: 4000000
  Occupancy: 15.0%
  Inflation radius: 0.55 m
  Iterations: 100

Timing Results:
  Mean: 187.234 ms
  Std Dev: 3.456 ms
  Min: 182.123 ms
  Max: 195.678 ms

Performance Metrics:
  Cells/second: 21358976
  Throughput: 5.34 updates/second
========================================
```

## Interpreting Performance

Factors that affect performance:

1. **Map Size**: Larger maps take longer to process
2. **Occupancy**: More obstacles mean more inflation work
3. **Inflation Radius**: Larger radii increase the affected area per obstacle
4. **Obstacle Distribution**: Clustered obstacles may process differently than scattered ones

## Troubleshooting

### Build Errors

If you encounter build errors, ensure:
- `BUILD_TESTING` is enabled: `-DBUILD_TESTING=ON`
- Google Benchmark is available: `ament_cmake_google_benchmark` package

### Runtime Issues

If the benchmark crashes or hangs:
- Reduce map size for initial testing
- Check available memory (large maps use significant RAM)
- Verify ROS 2 environment is properly sourced

## Contributing

When adding new benchmarks:

1. Follow the existing code structure
2. Use Google Benchmark fixtures for setup/teardown
3. Document parameters and expected behavior
4. Add entries to CMakeLists.txt
5. Update this README with usage instructions
