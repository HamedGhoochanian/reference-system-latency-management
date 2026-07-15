#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <iostream>
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
  try {
    period_scale = input_period_scale();
    set_structured_output_enabled(structured_output_enabled());
  } catch (const std::invalid_argument & error) {
    std::cerr << error.what() << "\n";
    return 1;
  }

  rclcpp::init(argc, argv);

  using TimeConfig = nodes::timing::Default;
  auto nodes = create_autoware_nodes<RclcppSystem, TimeConfig>({node_name}, period_scale);

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  for (auto & node : nodes) {
    executor.add_node(node);
  }
  executor.spin();

  nodes.clear();
  rclcpp::shutdown();
  return 0;
}
