// Copyright 2021 Apex.AI, Inc.
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

#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "reference_system/system/systems.hpp"

#include "autoware_reference_system/autoware_system_builder.hpp"
#include "autoware_reference_system/system/timing/default.hpp"

int main(int argc, char * argv[])
{
  const std::map<std::string, std::vector<std::string>> groups{
    {"planning_control_high", {"VehicleDBWSystem", "VehicleInterface", "MPCController",
      "BehaviorPlanner"}},
    {"perception_collision_high", {"FrontLidarDriver", "RearLidarDriver",
      "PointsTransformerFront", "PointsTransformerRear", "PointCloudFusion", "RayGroundFilter",
      "ObjectCollisionEstimator"}},
    {"intersection_medium", {"EuclideanClusterDetector", "EuclideanClusterSettings",
      "IntersectionOutput"}},
    {"map_localization_medium", {"PointCloudMap", "PointCloudMapLoader",
      "VoxelGridDownsampler", "NDTLocalizer", "Visualizer", "Lanelet2Map", "LanePlanner",
      "ParkingPlanner", "Lanelet2MapLoader", "Lanelet2GlobalPlanner"}},
  };

  if (argc != 2 || std::string(argv[1]) == "--help") {
    std::cerr << "usage: " << argv[0] << " <group>\nallowed groups:\n";
    for (const auto & group : groups) {
      std::cerr << "  " << group.first << "\n";
    }
    return argc == 2 ? 0 : 1;
  }

  const auto group = groups.find(argv[1]);
  if (group == groups.end()) {
    std::cerr << "unknown group: " << argv[1] << "\nallowed groups:\n";
    for (const auto & allowed_group : groups) {
      std::cerr << "  " << allowed_group.first << "\n";
    }
    return 1;
  }

  rclcpp::init(argc, argv);

  using TimeConfig = nodes::timing::Default;
  auto nodes = create_autoware_nodes<RclcppSystem, TimeConfig>(group->second);

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  for (auto & node : nodes) {
    executor.add_node(node);
  }
  executor.spin();

  nodes.clear();
  rclcpp::shutdown();

  return 0;
}
