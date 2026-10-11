#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "reference_system/system/systems.hpp"
#include "reference_system/sample_management.hpp"

#include "autoware_reference_system/autoware_system_builder.hpp"
#include "autoware_reference_system/system/timing/default.hpp"

namespace
{
const std::vector<std::string> kNodeNames{
  "FrontLidarDriver",
  "RearLidarDriver",
  "PointCloudMap",
  "Visualizer",
  "Lanelet2Map",
  "EuclideanClusterSettings",
  "PointsTransformerFront",
  "PointsTransformerRear",
  "VoxelGridDownsampler",
  "PointCloudMapLoader",
  "RayGroundFilter",
  "ObjectCollisionEstimator",
  "MPCController",
  "ParkingPlanner",
  "LanePlanner",
  "PointCloudFusion",
  "NDTLocalizer",
  "VehicleInterface",
  "Lanelet2GlobalPlanner",
  "Lanelet2MapLoader",
  "BehaviorPlanner",
  "EuclideanClusterDetector",
  "VehicleDBWSystem",
  "IntersectionOutput",
};

void print_usage(const char * program)
{
  std::cerr << "usage: " << program << " <node_name>\nallowed nodes:\n";
  for (const auto & node_name : kNodeNames) {
    std::cerr << "  " << node_name << "\n";
  }
}

double input_period_scale()
{
  const char * value = std::getenv("LAME_INPUT_PERIOD_SCALE");
  if (value == nullptr) {
    return 1.0;
  }
  char * end = nullptr;
  errno = 0;
  const double scale = std::strtod(value, &end);
  if (errno != 0 || end == value || *end != '\0' || !std::isfinite(scale) ||
    scale < 0.1 || scale > 10.0)
  {
    throw std::invalid_argument{"LAME_INPUT_PERIOD_SCALE must be in [0.1, 10.0]"};
  }
  return scale;
}

bool is_source_node(const std::string & node_name)
{
  return node_name == "FrontLidarDriver" || node_name == "RearLidarDriver" ||
         node_name == "PointCloudMap" || node_name == "Visualizer" ||
         node_name == "Lanelet2Map" || node_name == "EuclideanClusterSettings";
}

std::optional<std::chrono::nanoseconds> source_period(const std::string & node_name)
{
  const char * value = std::getenv("LAME_SOURCE_PERIOD_NS");
  if (value == nullptr) {
    return std::nullopt;
  }
  if (std::getenv("LAME_INPUT_PERIOD_SCALE") != nullptr) {
    throw std::invalid_argument{
            "LAME_SOURCE_PERIOD_NS conflicts with LAME_INPUT_PERIOD_SCALE"};
  }
  if (!is_source_node(node_name)) {
    throw std::invalid_argument{"LAME_SOURCE_PERIOD_NS is valid only for source nodes"};
  }
  char * end = nullptr;
  errno = 0;
  const long long period_ns = std::strtoll(value, &end, 10);
  if (errno != 0 || end == value || *end != '\0' || period_ns <= 0) {
    throw std::invalid_argument{"LAME_SOURCE_PERIOD_NS must be a positive integer"};
  }
  return std::chrono::nanoseconds{period_ns};
}

bool structured_output_enabled()
{
  const char * value = std::getenv("LAME_STRUCTURED_OUTPUT");
  if (value == nullptr || std::string{value} == "1") {
    return true;
  }
  if (std::string{value} == "0") {
    return false;
  }
  throw std::invalid_argument{"LAME_STRUCTURED_OUTPUT must be 0 or 1"};
}
}  // namespace

int main(int argc, char * argv[])
{
  if (argc != 2 || std::string(argv[1]) == "--help") {
    print_usage(argv[0]);
    return argc == 2 ? 0 : 1;
  }

  const std::string node_name{argv[1]};
  if (std::find(kNodeNames.begin(), kNodeNames.end(), node_name) == kNodeNames.end()) {
    std::cerr << "unknown node: " << node_name << "\n";
    print_usage(argv[0]);
    return 1;
  }

  double period_scale;
  std::optional<std::chrono::nanoseconds> source_period_ns;
  try {
    source_period_ns = source_period(node_name);
    period_scale = input_period_scale();
    set_structured_output_enabled(structured_output_enabled());
  } catch (const std::invalid_argument & error) {
    std::cerr << error.what() << "\n";
    return 1;
  }

  rclcpp::init(argc, argv);

  using TimeConfig = nodes::timing::Default;
  auto nodes = create_autoware_nodes<RclcppSystem, TimeConfig>(
    {node_name}, period_scale, source_period_ns);

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  for (auto & node : nodes) {
    executor.add_node(node);
  }
  executor.spin();

  nodes.clear();
  rclcpp::shutdown();
  return 0;
}
