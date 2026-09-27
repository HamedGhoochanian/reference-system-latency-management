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
#ifndef AUTOWARE_REFERENCE_SYSTEM__AUTOWARE_SYSTEM_BUILDER_HPP_
#define AUTOWARE_REFERENCE_SYSTEM__AUTOWARE_SYSTEM_BUILDER_HPP_
#include <algorithm>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "reference_system/nodes/settings.hpp"
#include "reference_system/system/systems.hpp"

using namespace std::chrono_literals;  // NOLINT

#ifdef PICAS
#include "autoware_reference_system/system/priority/default.hpp"
using CallbackPriority = callback::priority::Default;
#endif

template<typename SystemType, typename TimingConfig>
auto create_autoware_nodes(
  const std::vector<std::string> & selected_node_names = {},
  const double input_period_scale = 1.0)
->std::vector<std::shared_ptr<typename SystemType::NodeBaseType>>
{
  std::vector<std::shared_ptr<typename SystemType::NodeBaseType>> nodes;
  const auto is_selected = [&selected_node_names](const char * node_name) {
    return selected_node_names.empty() || std::find(
      selected_node_names.begin(), selected_node_names.end(), node_name) != selected_node_names.end();
  };
  const auto input_period = [input_period_scale](const auto period) {
    return std::chrono::nanoseconds{
      static_cast<std::chrono::nanoseconds::rep>(period.count() * input_period_scale)};
  };

// ignore the warning about designated initializers - they make the code much
// more readable
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"

  // setup communication graph
  // sensor nodes
  if (is_selected("FrontLidarDriver")) {
    nodes.emplace_back(
      std::make_shared<typename SystemType::Sensor>(
        nodes::SensorSettings{.node_name = "FrontLidarDriver",
          .topic_name = "FrontLidarDriver",
          .cycle_time = input_period(TimingConfig::FRONT_LIDAR_DRIVER),
          #ifdef PICAS
          .callback_priority = CallbackPriority::FRONT_LIDAR_DRIVER_CALLBACK
          #endif
        }));
  }

  if (is_selected("RearLidarDriver")) {
    nodes.emplace_back(
      std::make_shared<typename SystemType::Sensor>(
        nodes::SensorSettings{.node_name = "RearLidarDriver",
          .topic_name = "RearLidarDriver",
          .cycle_time = input_period(TimingConfig::REAR_LIDAR_DRIVER),
          #ifdef PICAS
          .callback_priority = CallbackPriority::REAR_LIDAR_DRIVER_CALLBACK
          #endif
        }));
  }

  if (is_selected("PointCloudMap")) {
    nodes.emplace_back(
      std::make_shared<typename SystemType::Sensor>(
        nodes::SensorSettings{.node_name = "PointCloudMap",
          .topic_name = "PointCloudMap",
          .cycle_time = input_period(TimingConfig::POINT_CLOUD_MAP),
          #ifdef PICAS
          .callback_priority = CallbackPriority::POINT_CLOUD_MAP_CALLBACK
          #endif
        }));
  }

  if (is_selected("Visualizer")) {
    nodes.emplace_back(
      std::make_shared<typename SystemType::Sensor>(
        nodes::SensorSettings{.node_name = "Visualizer",
          .topic_name = "Visualizer",
          .cycle_time = input_period(TimingConfig::VISUALIZER),
          #ifdef PICAS
          .callback_priority = CallbackPriority::VISUALIZER_CALLBACK
          #endif
        }));
  }

  if (is_selected("Lanelet2Map")) {
    nodes.emplace_back(
      std::make_shared<typename SystemType::Sensor>(
        nodes::SensorSettings{.node_name = "Lanelet2Map",
          .topic_name = "Lanelet2Map",
          .cycle_time = input_period(TimingConfig::LANELET2MAP),
          #ifdef PICAS
          .callback_priority = CallbackPriority::LANELET_2_MAP_CALLBACK
          #endif
        }));
  }

  if (is_selected("EuclideanClusterSettings")) {
    nodes.emplace_back(
      std::make_shared<typename SystemType::Sensor>(
        nodes::SensorSettings{.node_name = "EuclideanClusterSettings",
          .topic_name = "EuclideanClusterSettings",
          .cycle_time = input_period(TimingConfig::EUCLIDEAN_CLUSTER_SETTINGS),
          #ifdef PICAS
          .callback_priority = CallbackPriority::EUCLIDEAN_CLUSTER_SETTINGS_CALLBACK
          #endif
        }));
  }

  // transform nodes
  if (is_selected("PointsTransformerFront")) {
    nodes.emplace_back(
      std::make_shared<typename SystemType::Transform>(
        nodes::TransformSettings{
      .node_name = "PointsTransformerFront",
      .input_topic = "FrontLidarDriver",
      .output_topic = "PointsTransformerFront",
      .number_crunch_limit = TimingConfig::POINTS_TRANSFORMER_FRONT,
      #ifdef PICAS
      .callback_priority = CallbackPriority::POINTS_TRANSFORMER_FRONT_CALLBACK
      #endif
    }));
  }

  if (is_selected("PointsTransformerRear")) {
    nodes.emplace_back(
      std::make_shared<typename SystemType::Transform>(
        nodes::TransformSettings{
      .node_name = "PointsTransformerRear",
      .input_topic = "RearLidarDriver",
      .output_topic = "PointsTransformerRear",
      .number_crunch_limit = TimingConfig::POINTS_TRANSFORMER_REAR,
      #ifdef PICAS
      .callback_priority = CallbackPriority::POINTS_TRANSFORMER_REAR_CALLBACK
      #endif
    }));
  }

  if (is_selected("VoxelGridDownsampler")) {
    nodes.emplace_back(
      std::make_shared<typename SystemType::Transform>(
        nodes::TransformSettings{
      .node_name = "VoxelGridDownsampler",
      .input_topic = "PointCloudFusion",
      .output_topic = "VoxelGridDownsampler",
      .number_crunch_limit = TimingConfig::VOXEL_GRID_DOWNSAMPLER,
      #ifdef PICAS
      .callback_priority = CallbackPriority::VOXEL_GRID_DOWNSAMPLER_CALLBACK
      #endif
    }));
  }

  if (is_selected("PointCloudMapLoader")) {
    nodes.emplace_back(
      std::make_shared<typename SystemType::Transform>(
        nodes::TransformSettings{
      .node_name = "PointCloudMapLoader",
      .input_topic = "PointCloudMap",
      .output_topic = "PointCloudMapLoader",
      .number_crunch_limit = TimingConfig::POINT_CLOUD_MAP_LOADER,
      #ifdef PICAS
      .callback_priority = CallbackPriority::POINT_CLOUD_MAP_LOADER_CALLBACK
      #endif
    }));
  }

  if (is_selected("RayGroundFilter")) {
    nodes.emplace_back(
      std::make_shared<typename SystemType::Transform>(
        nodes::TransformSettings{
      .node_name = "RayGroundFilter",
      .input_topic = "PointCloudFusion",
      .output_topic = "RayGroundFilter",
      .number_crunch_limit = TimingConfig::RAY_GROUND_FILTER,
      #ifdef PICAS
      .callback_priority = CallbackPriority::RAY_GROUND_FILTER_CALLBACK
      #endif
    }));
  }

  if (is_selected("ObjectCollisionEstimator")) {
    nodes.emplace_back(
      std::make_shared<typename SystemType::Transform>(
        nodes::TransformSettings{
      .node_name = "ObjectCollisionEstimator",
      .input_topic = "EuclideanClusterDetector",
      .output_topic = "ObjectCollisionEstimator",
      .number_crunch_limit = TimingConfig::OBJECT_COLLISION_ESTIMATOR,
      #ifdef PICAS
      .callback_priority = CallbackPriority::OBJECT_COLLISION_ESTIMATOR_CALLBACK
      #endif
    }));
  }

  if (is_selected("MPCController")) {
    nodes.emplace_back(
      std::make_shared<typename SystemType::Transform>(
        nodes::TransformSettings{
      .node_name = "MPCController",
      .input_topic = "BehaviorPlanner",
      .output_topic = "MPCController",
      .number_crunch_limit = TimingConfig::MPC_CONTROLLER,
      #ifdef PICAS
      .callback_priority = CallbackPriority::MPC_CONTROLLER_CALLBACK
      #endif
    }));
  }

  if (is_selected("ParkingPlanner")) {
    nodes.emplace_back(
      std::make_shared<typename SystemType::Transform>(
        nodes::TransformSettings{
      .node_name = "ParkingPlanner",
      .input_topic = "Lanelet2MapLoader",
      .output_topic = "ParkingPlanner",
      .number_crunch_limit = TimingConfig::PARKING_PLANNER,
      #ifdef PICAS
      .callback_priority = CallbackPriority::PARKING_PLANNER_CALLBACK
      #endif
    }));
  }

  if (is_selected("LanePlanner")) {
    nodes.emplace_back(
      std::make_shared<typename SystemType::Transform>(
        nodes::TransformSettings{
      .node_name = "LanePlanner",
      .input_topic = "Lanelet2MapLoader",
      .output_topic = "LanePlanner",
      .number_crunch_limit = TimingConfig::LANE_PLANNER,
      #ifdef PICAS
      .callback_priority = CallbackPriority::LANE_PLANNER_CALLBACK
      #endif
    }));
  }

  // fusion nodes
  if (is_selected("PointCloudFusion")) {
    nodes.emplace_back(
      std::make_shared<typename SystemType::Fusion>(
        nodes::FusionSettings{
      .node_name = "PointCloudFusion",
      .input_0 = "PointsTransformerFront",
      .input_1 = "PointsTransformerRear",
      .output_topic = "PointCloudFusion",
      .number_crunch_limit = TimingConfig::POINT_CLOUD_FUSION,
      #ifdef PICAS
      .callback_priority_1 = CallbackPriority::POINT_CLOUD_FUSION_CALLBACK_1,
      .callback_priority_2 = CallbackPriority::POINT_CLOUD_FUSION_CALLBACK_2
      #endif
    }));
  }

  if (is_selected("NDTLocalizer")) {
    nodes.emplace_back(
      std::make_shared<typename SystemType::Fusion>(
        nodes::FusionSettings{
      .node_name = "NDTLocalizer",
      .input_0 = "PointCloudMapLoader", // modified
      .input_1 = "VoxelGridDownsampler", //modified
      .output_topic = "NDTLocalizer",
      .number_crunch_limit = TimingConfig::NDT_LOCALIZER,
      #ifdef PICAS
      .callback_priority_1 = CallbackPriority::NDT_LOCALIZER_CALLBACK_1,
      .callback_priority_2 = CallbackPriority::NDT_LOCALIZER_CALLBACK_2
      #endif
    }));
  }

  if (is_selected("VehicleInterface")) {
    nodes.emplace_back(
      std::make_shared<typename SystemType::Fusion>(
        nodes::FusionSettings{
      .node_name = "VehicleInterface",
      .input_0 = "MPCController",
      .input_1 = "BehaviorPlanner",
      .output_topic = "VehicleInterface",
      .number_crunch_limit = TimingConfig::VEHICLE_INTERFACE,
      #ifdef PICAS
      .callback_priority_1 = CallbackPriority::VEHICLE_INTERFACE_CALLBACK_1,
      .callback_priority_2 = CallbackPriority::VEHICLE_INTERFACE_CALLBACK_2
      #endif
    }));
  }

  if (is_selected("Lanelet2GlobalPlanner")) {
    nodes.emplace_back(
      std::make_shared<typename SystemType::Fusion>(
        nodes::FusionSettings{
      .node_name = "Lanelet2GlobalPlanner",
      .input_0 = "Visualizer",
      .input_1 = "NDTLocalizer",
      .output_topic = "Lanelet2GlobalPlanner",
      .number_crunch_limit = TimingConfig::LANELET_2_GLOBAL_PLANNER,
      #ifdef PICAS
      .callback_priority_1 = CallbackPriority::LANELET_2_GLOBAL_PLANNER_CALLBACK_1,
      .callback_priority_2 = CallbackPriority::LANELET_2_GLOBAL_PLANNER_CALLBACK_2
      #endif
    }));
  }

  if (is_selected("Lanelet2MapLoader")) {
    nodes.emplace_back(
      std::make_shared<typename SystemType::Fusion>(
        nodes::FusionSettings{
      .node_name = "Lanelet2MapLoader",
      .input_0 = "Lanelet2Map",
      .input_1 = "Lanelet2GlobalPlanner",
      .output_topic = "Lanelet2MapLoader",
      .number_crunch_limit = TimingConfig::LANELET_2_MAP_LOADER,
      #ifdef PICAS
      .callback_priority_1 = CallbackPriority::LANELET_2_MAP_LOADER_CALLBACK_1,
      .callback_priority_2 = CallbackPriority::LANELET_2_MAP_LOADER_CALLBACK_2
      #endif
    }));
  }

  // cyclic node
  if (is_selected("BehaviorPlanner")) {
    nodes.emplace_back(
      std::make_shared<typename SystemType::Cyclic>(
        nodes::CyclicSettings{
      .node_name = "BehaviorPlanner",
      .inputs = {"ObjectCollisionEstimator", "NDTLocalizer",
        "Lanelet2GlobalPlanner", "Lanelet2MapLoader",
        "ParkingPlanner", "LanePlanner"},
      .output_topic = "BehaviorPlanner",
      .number_crunch_limit = TimingConfig::BEHAVIOR_PLANNER,
      .cycle_time = TimingConfig::BEHAVIOR_PLANNER_CYCLE,
      #ifdef PICAS
      .callback_priority_1 = CallbackPriority::BEHAVIOR_PLANNER_CALLBACK_1,
      .callback_priority_2 = CallbackPriority::BEHAVIOR_PLANNER_CALLBACK_2,
      .callback_priority_3 = CallbackPriority::BEHAVIOR_PLANNER_CALLBACK_3,
      .callback_priority_4 = CallbackPriority::BEHAVIOR_PLANNER_CALLBACK_4,
      .callback_priority_5 = CallbackPriority::BEHAVIOR_PLANNER_CALLBACK_5,
      .callback_priority_6 = CallbackPriority::BEHAVIOR_PLANNER_CALLBACK_6,
      .callback_priority_7 = CallbackPriority::BEHAVIOR_PLANNER_CALLBACK_7
      #endif
    }));
  }

  // intersection node
  if (is_selected("EuclideanClusterDetector")) {
    nodes.emplace_back(
      std::make_shared<typename SystemType::Intersection>(
        nodes::IntersectionSettings{
      .node_name = "EuclideanClusterDetector",
      .connections = {
        {.input_topic = "RayGroundFilter",
          .output_topic = "EuclideanClusterDetector",
          .number_crunch_limit = TimingConfig::EUCLIDEAN_CLUSTER_DETECTOR,
          #ifdef PICAS
          .callback_priority = CallbackPriority::EUCLIDEAN_CLUSTER_DETECTOR_CALLBACK
          #endif
        },
        {.input_topic = "EuclideanClusterSettings",
          .output_topic = "EuclideanIntersection",
          .number_crunch_limit = TimingConfig::EUCLIDEAN_INTERSECTION,
          #ifdef PICAS
          .callback_priority = CallbackPriority::EUCLIDEAN_INTERSECTION_CALLBACK
          #endif
        }
      }}
    ));
  }

  // command node
  if (is_selected("VehicleDBWSystem")) {
    nodes.emplace_back(
      std::make_shared<typename SystemType::Command>(
        nodes::CommandSettings{
      .node_name = "VehicleDBWSystem", .input_topic = "VehicleInterface",
      #ifdef PICAS
      .callback_priority = CallbackPriority::VEHICLE_DBW_SYSTEM_CALLBACK
      #endif
    }));
  }

  if (is_selected("IntersectionOutput")) {
    nodes.emplace_back(
      std::make_shared<typename SystemType::Command>(
        nodes::CommandSettings{
      .node_name = "IntersectionOutput",
      .input_topic = "EuclideanIntersection",
      #ifdef PICAS
      .callback_priority = CallbackPriority::INTERSECTION_OUTPUT_CALLBACK
      #endif
    }));
  }
#pragma GCC diagnostic pop

  return nodes;
}

#endif  // AUTOWARE_REFERENCE_SYSTEM__AUTOWARE_SYSTEM_BUILDER_HPP_
