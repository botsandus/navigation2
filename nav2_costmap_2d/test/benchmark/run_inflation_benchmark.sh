#!/bin/bash
# Helper script to run the inflation layer updateCosts benchmark
# This script provides easy-to-use wrappers for common benchmark scenarios

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Find the benchmark executable
# Allow override with environment variable
if [ -n "$AUTO_WS_ROOT" ]; then
    WORKSPACE_ROOT="$AUTO_WS_ROOT"
else
    # Try to find the workspace root by looking for common workspace indicators
    CURRENT_DIR="$PWD"
    while [ "$CURRENT_DIR" != "/" ]; do
        if [ -f "$CURRENT_DIR/src/auto-sandbox/arri.repos" ]; then
            WORKSPACE_ROOT="$CURRENT_DIR"
            break
        fi
        CURRENT_DIR="$(dirname "$CURRENT_DIR")"
    done
    
    # Fallback to assuming we're in /opt/auto_ws
    if [ -z "$WORKSPACE_ROOT" ]; then
        WORKSPACE_ROOT="/opt/auto_ws"
    fi
fi

BENCHMARK_EXE="$WORKSPACE_ROOT/build/nav2_costmap_2d/test/benchmark/inflation_layer_updatecosts_benchmark"

if [ ! -f "$BENCHMARK_EXE" ]; then
    echo -e "${RED}Error: Benchmark executable not found!${NC}"
    echo "Expected location: $BENCHMARK_EXE"
    echo ""
    echo "Please build the package first:"
    echo "  cd $WORKSPACE_ROOT"
    echo "  colcon build --packages-select nav2_costmap_2d --cmake-args -DBUILD_TESTING=ON"
    exit 1
fi

# Function to print usage
print_usage() {
    echo "Inflation Layer UpdateCosts Benchmark Helper"
    echo "============================================="
    echo ""
    echo "Usage: $0 [OPTION]"
    echo ""
    echo "Quick Run Options:"
    echo "  all               Run all predefined scenarios"
    echo "  small             Run small map scenarios (100x100)"
    echo "  medium            Run medium map scenarios (500x500)"
    echo "  large             Run large map scenarios (1000x1000)"
    echo "  xlarge            Run extra large map scenarios (2000x2000)"
    echo "  custom            Run custom benchmark (prompts for parameters)"
    echo ""
    echo "Custom Parameters:"
    echo "  --width=N         Map width in cells"
    echo "  --height=N        Map height in cells"
    echo "  --occupancy=N     Occupancy percentage (0-100)"
    echo "  --inflation=N     Inflation radius in meters"
    echo ""
    echo "Output Options:"
    echo "  --csv=FILE        Save results to CSV file"
    echo "  --json=FILE       Save results to JSON file"
    echo "  --repetitions=N   Number of times to repeat each benchmark"
    echo ""
    echo "Examples:"
    echo "  $0 all"
    echo "  $0 medium"
    echo "  $0 custom --width=1500 --height=1500 --occupancy=20"
    echo "  $0 all --csv=results.csv --repetitions=10"
    echo ""
}

# Parse arguments
MODE="all"
CUSTOM_ARGS=""
BENCHMARK_ARGS=""

if [ $# -eq 0 ]; then
    print_usage
    exit 0
fi

for arg in "$@"; do
    case $arg in
        -h|--help)
            print_usage
            exit 0
            ;;
        all|small|medium|large|xlarge|custom)
            MODE="$arg"
            ;;
        --width=*|--height=*|--occupancy=*|--inflation=*)
            CUSTOM_ARGS="$CUSTOM_ARGS $arg"
            ;;
        --csv=*)
            FILE="${arg#*=}"
            BENCHMARK_ARGS="$BENCHMARK_ARGS --benchmark_format=csv --benchmark_out=$FILE"
            ;;
        --json=*)
            FILE="${arg#*=}"
            BENCHMARK_ARGS="$BENCHMARK_ARGS --benchmark_format=json --benchmark_out=$FILE"
            ;;
        --repetitions=*)
            BENCHMARK_ARGS="$BENCHMARK_ARGS --benchmark_repetitions=${arg#*=}"
            ;;
        *)
            echo -e "${RED}Unknown argument: $arg${NC}"
            print_usage
            exit 1
            ;;
    esac
done

# Source ROS 2 environment if not already sourced
if [ -z "$ROS_DISTRO" ]; then
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        echo -e "${YELLOW}Sourcing ROS 2 Humble...${NC}"
        source /opt/ros/humble/setup.bash
    else
        echo -e "${RED}Error: ROS 2 environment not sourced and not found at /opt/ros/humble${NC}"
        exit 1
    fi
fi

# Source workspace overlay
if [ -f "$WORKSPACE_ROOT/install/setup.bash" ]; then
    source "$WORKSPACE_ROOT/install/setup.bash"
fi

echo -e "${GREEN}Running Inflation Layer UpdateCosts Benchmark${NC}"
echo "================================================"
echo ""

# Run the appropriate benchmark mode
case $MODE in
    all)
        echo "Running all predefined scenarios..."
        "$BENCHMARK_EXE" $BENCHMARK_ARGS
        ;;
    small)
        echo "Running small map scenarios (100x100)..."
        "$BENCHMARK_EXE" --benchmark_filter="100/100" $BENCHMARK_ARGS
        ;;
    medium)
        echo "Running medium map scenarios (500x500)..."
        "$BENCHMARK_EXE" --benchmark_filter="500/500" $BENCHMARK_ARGS
        ;;
    large)
        echo "Running large map scenarios (1000x1000)..."
        "$BENCHMARK_EXE" --benchmark_filter="1000/1000" $BENCHMARK_ARGS
        ;;
    xlarge)
        echo "Running extra large map scenarios (2000x2000)..."
        "$BENCHMARK_EXE" --benchmark_filter="2000/2000" $BENCHMARK_ARGS
        ;;
    custom)
        if [ -z "$CUSTOM_ARGS" ]; then
            # Interactive mode
            echo "Custom Benchmark Configuration"
            echo "------------------------------"
            read -p "Map width (cells) [1000]: " width
            width=${width:-1000}
            
            read -p "Map height (cells) [1000]: " height
            height=${height:-1000}
            
            read -p "Occupancy percentage (0-100) [10]: " occupancy
            occupancy=${occupancy:-10}
            
            read -p "Inflation radius (meters) [0.55]: " inflation
            inflation=${inflation:-0.55}
            
            CUSTOM_ARGS="--width=$width --height=$height --occupancy=$occupancy --inflation=$inflation"
        fi
        
        echo "Running custom benchmark with parameters:$CUSTOM_ARGS"
        "$BENCHMARK_EXE" --custom $CUSTOM_ARGS
        ;;
esac

echo ""
echo -e "${GREEN}Benchmark complete!${NC}"

# Show output file location if specified
if echo "$BENCHMARK_ARGS" | grep -q "benchmark_out"; then
    OUTPUT_FILE=$(echo "$BENCHMARK_ARGS" | grep -o "benchmark_out=[^ ]*" | cut -d= -f2)
    echo -e "${GREEN}Results saved to: $OUTPUT_FILE${NC}"
fi
