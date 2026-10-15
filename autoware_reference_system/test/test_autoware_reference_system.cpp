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

// Regression test: selected nodes must not be constructed then erased.
#include <chrono>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "autoware_reference_system/autoware_system_builder.hpp"
#include "autoware_reference_system/system/timing/default.hpp"

namespace
{
struct TestNode
{
  explicit TestNode(const nodes::CommandSettings & settings) { record(settings.node_name); }
  explicit TestNode(const nodes::CyclicSettings & settings) { record(settings.node_name); }
  explicit TestNode(const nodes::FusionSettings & settings) { record(settings.node_name); }
  explicit TestNode(const nodes::IntersectionSettings & settings) { record(settings.node_name); }
  explicit TestNode(const nodes::SensorSettings & settings)
  {
    record(settings.node_name);
    sensor_period = settings.cycle_time;
    source_schedule_file = settings.source_schedule_file;
  }
  explicit TestNode(const nodes::TransformSettings & settings) { record(settings.node_name); }

  const std::string & get_name() const { return name; }

  static std::vector<std::string> constructed_names;
  std::chrono::nanoseconds sensor_period{0};
  std::string source_schedule_file;

private:
  void record(const std::string & node_name)
  {
    name = node_name;
    constructed_names.emplace_back(node_name);
  }

  std::string name;
};

std::vector<std::string> TestNode::constructed_names;

struct TestSystem
{
  using NodeBaseType = TestNode;
  using Command = TestNode;
  using Cyclic = TestNode;
  using Fusion = TestNode;
  using Intersection = TestNode;
  using Sensor = TestNode;
  using Transform = TestNode;
};

class AutowareSystemBuilderTest : public ::testing::Test
{
protected:
  void SetUp() override { TestNode::constructed_names.clear(); }
};

TEST_F(AutowareSystemBuilderTest, EmptySelectionConstructsAllNodes)
{
  const auto built_nodes = create_autoware_nodes<TestSystem, nodes::timing::Default>();

  EXPECT_EQ(built_nodes.size(), 24U);
  EXPECT_EQ(TestNode::constructed_names.size(), built_nodes.size());
}

TEST_F(AutowareSystemBuilderTest, GroupSelectionConstructsOnlyThatGroup)
{
  const std::vector<std::string> selected{
    "VehicleDBWSystem", "VehicleInterface", "MPCController", "BehaviorPlanner"};
  const std::vector<std::string> expected{
    "MPCController", "VehicleInterface", "BehaviorPlanner", "VehicleDBWSystem"};

  const auto built_nodes = create_autoware_nodes<TestSystem, nodes::timing::Default>(selected);

  EXPECT_EQ(built_nodes.size(), expected.size());
  EXPECT_EQ(TestNode::constructed_names, expected);
}

TEST_F(AutowareSystemBuilderTest, SingleSelectionConstructsOnlyThatNode)
{
  const auto built_nodes = create_autoware_nodes<TestSystem, nodes::timing::Default>(
    {"IntersectionOutput"});

  ASSERT_EQ(built_nodes.size(), 1U);
  EXPECT_EQ(built_nodes.front()->get_name(), "IntersectionOutput");
  EXPECT_EQ(TestNode::constructed_names, std::vector<std::string>{"IntersectionOutput"});
}

TEST_F(AutowareSystemBuilderTest, InputPeriodScaleChangesSensorPeriod)
{
  const auto built_nodes = create_autoware_nodes<TestSystem, nodes::timing::Default>(
    {"FrontLidarDriver"}, 0.5);

  ASSERT_EQ(built_nodes.size(), 1U);
  EXPECT_EQ(built_nodes.front()->sensor_period, std::chrono::milliseconds{50});
}

TEST_F(AutowareSystemBuilderTest, ExplicitSourcePeriodOverridesScale)
{
  const auto built_nodes = create_autoware_nodes<TestSystem, nodes::timing::Default>(
    {"FrontLidarDriver"}, 0.5, std::chrono::nanoseconds{50000001});

  ASSERT_EQ(built_nodes.size(), 1U);
  EXPECT_EQ(built_nodes.front()->sensor_period, std::chrono::nanoseconds{50000001});
}

TEST_F(AutowareSystemBuilderTest, SourceScheduleReachesEverySelectedSource)
{
  const std::vector<std::string> selected{
    "FrontLidarDriver", "RearLidarDriver", "PointCloudMap", "Visualizer",
    "Lanelet2Map", "EuclideanClusterSettings"};
  const auto built_nodes = create_autoware_nodes<TestSystem, nodes::timing::Default>(
    selected, 1.0, std::chrono::milliseconds{100}, "/artifacts/control/synthetic.schedule");
  ASSERT_EQ(built_nodes.size(), 6U);
  for (const auto & node : built_nodes) {
    EXPECT_EQ(node->source_schedule_file, "/artifacts/control/synthetic.schedule");
    EXPECT_EQ(node->sensor_period, std::chrono::milliseconds{100});
  }
}
}  // namespace
