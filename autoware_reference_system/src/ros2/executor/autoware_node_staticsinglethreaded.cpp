#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "reference_system/system/systems.hpp"

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

  rclcpp::init(argc, argv);

  using TimeConfig = nodes::timing::Default;
  auto nodes = create_autoware_nodes<RclcppSystem, TimeConfig>({node_name});

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  for (auto & node : nodes) {
    executor.add_node(node);
  }
  executor.spin();

  nodes.clear();
  rclcpp::shutdown();
  return 0;
}
