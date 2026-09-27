// Copyright 2026 Eindhoven University of Technology
// Licensed under the Apache License 2.0

#include <gtest/gtest.h>

#include <map>
#include <string>
#include <vector>

#include "reference_system/sample_management.hpp"

TEST(SampleManagement, RejectsBackwardTimestamps)
{
  uint64_t elapsed = 0;
  const auto before = structured_record_error_count().load();
  EXPECT_FALSE(elapsed_ns(20, 10, elapsed));
  EXPECT_EQ(before + 1, structured_record_error_count().load());
  EXPECT_TRUE(elapsed_ns(10, 20, elapsed));
  EXPECT_EQ(10U, elapsed);
}

TEST(SampleManagement, DeadlineStatusUsesStrictBoundary)
{
  EXPECT_STREQ("completed", deadline_status(499999999U, 500000000U));
  EXPECT_STREQ("completed", deadline_status(500000000U, 500000000U));
  EXPECT_STREQ("violated", deadline_status(500000001U, 500000000U));

  EXPECT_STREQ("completed", deadline_status(999999999U, 1000000000U));
  EXPECT_STREQ("completed", deadline_status(1000000000U, 1000000000U));
  EXPECT_STREQ("violated", deadline_status(1000000001U, 1000000000U));

  EXPECT_STREQ("completed", deadline_status(249999999U, 250000000U));
  EXPECT_STREQ("completed", deadline_status(250000000U, 250000000U));
  EXPECT_STREQ("violated", deadline_status(250000001U, 250000000U));
}

TEST(SampleManagement, KeepsNewestNodeSnapshotRegardlessOfMergeOrder)
{
  // Test gap: uint32_t sequence wrap is out of scope for short benchmark runs.
  message_t older{};
  message_t newer{};
  message_t first{};
  message_t second{};
  set_sample("FrontLidarDriver", 4, 1, 900, older);
  set_sample("FrontLidarDriver", 5, 2, 100, newer);

  merge_history_into_sample(first, &older);
  merge_history_into_sample(first, &newer);
  merge_history_into_sample(second, &newer);
  merge_history_into_sample(second, &older);

  ASSERT_EQ(1U, first.size);
  ASSERT_EQ(1U, second.size);
  EXPECT_EQ(5U, first.stats[0].sequence_number);
  EXPECT_EQ(100U, first.stats[0].timestamp);
  EXPECT_EQ(2U, first.stats[0].dropped_samples);
  EXPECT_EQ(first.stats[0].sequence_number, second.stats[0].sequence_number);
  EXPECT_EQ(first.stats[0].timestamp, second.stats[0].timestamp);
}

TEST(SampleManagement, KeepsMaximumDropsOnExactTieRegardlessOfMergeOrder)
{
  message_t fewer_drops{};
  message_t more_drops{};
  message_t first{};
  message_t second{};
  set_sample("Node", 7, 2, 101, fewer_drops);
  set_sample("Node", 7, 99, 101, more_drops);

  merge_history_into_sample(first, &fewer_drops);
  merge_history_into_sample(first, &more_drops);
  merge_history_into_sample(second, &more_drops);
  merge_history_into_sample(second, &fewer_drops);

  ASSERT_EQ(1U, first.size);
  ASSERT_EQ(1U, second.size);
  EXPECT_EQ(99U, first.stats[0].dropped_samples);
  EXPECT_EQ(99U, second.stats[0].dropped_samples);
}

TEST(SampleManagement, SetSampleDoesNotDuplicateCurrentNode)
{
  message_t sample{};
  set_sample("Current", 1, 0, 100, sample);
  set_sample("Current", 2, 3, 200, sample);
  set_sample("Current", 2, 9, 199, sample);

  ASSERT_EQ(1U, sample.size);
  EXPECT_EQ(2U, sample.stats[0].sequence_number);
  EXPECT_EQ(200U, sample.stats[0].timestamp);
  EXPECT_EQ(3U, sample.stats[0].dropped_samples);
}

TEST(SampleManagement, StructuredOutputDisabledMetadataStaysBoundedWithoutOverflow)
{
  const std::vector<std::string> fixed_topology{
    "FrontLidarDriver", "RearLidarDriver", "PointsTransformerFront",
    "PointsTransformerRear", "PointCloudFusion", "VoxelGridDownsampler",
    "RayGroundFilter", "EuclideanClusterDetector", "ObjectCollisionEstimator",
    "PointCloudMap", "PointCloudMapLoader", "NDTLocalizer", "Visualizer",
    "Lanelet2GlobalPlanner", "Lanelet2Map", "Lanelet2MapLoader", "ParkingPlanner",
    "LanePlanner", "BehaviorPlanner", "MPCController", "VehicleInterface",
    "EuclideanClusterSettings", "VehicleDBWSystem", "IntersectionOutput"};
  message_t source{};
  message_t destination{};
  const auto overflow_before = metadata_overflow_count().load();
  EXPECT_EQ(0U, overflow_before);
  set_structured_output_enabled(false);
  for (uint32_t sequence = 1; sequence <= 100; ++sequence) {
    for (size_t i = 0; i < fixed_topology.size(); ++i) {
      set_sample(
        fixed_topology[i], sequence, 0, sequence * 100 + i, source);
    }
    merge_history_into_sample(destination, &source);
  }
  set_structured_output_enabled(true);

  EXPECT_LT(destination.size, message_t::STATS_CAPACITY);
  EXPECT_EQ(fixed_topology.size(), source.size);
  EXPECT_EQ(fixed_topology.size(), destination.size);
  EXPECT_EQ(100U, destination.stats[0].sequence_number);
  EXPECT_EQ(overflow_before, metadata_overflow_count().load());
}

TEST(SampleManagement, ExtractsAndOrdersSourceRootsWithMinMaxTies)
{
  message_t sample{};
  set_sample("RearLidarDriver", 3, 0, 100, sample);
  set_sample("FrontLidarDriver", 7, 0, 100, sample);

  const auto roots = extract_source_roots(
    &sample, std::vector<std::string>{"FrontLidarDriver", "RearLidarDriver"});
  ASSERT_EQ(2U, roots.size());
  EXPECT_EQ("FrontLidarDriver", roots[0].node_name);
  EXPECT_EQ("RearLidarDriver", roots[1].node_name);
  EXPECT_EQ("FrontLidarDriver", select_source_reference(roots, true).node_name);
  EXPECT_EQ("RearLidarDriver", select_source_reference(roots, false).node_name);

  const std::vector<source_identity_t> sequence_tie{
    {"SameNode", 1, 100}, {"SameNode", 2, 100}};
  EXPECT_EQ(1U, select_source_reference(sequence_tie, true).sequence_number);
  EXPECT_EQ(2U, select_source_reference(sequence_tie, false).sequence_number);
}

TEST(SampleManagement, HotPathValidatorRequiresExactKnownTopology)
{
  std::map<std::string, node_map_t> front{
    {"FrontLidarDriver", {1, 0, 0}},
    {"PointsTransformerFront", {2, 0, 0}},
    {"PointCloudFusion", {3, 0, 0}},
    {"RayGroundFilter", {4, 0, 0}},
    {"EuclideanClusterDetector", {5, 0, 0}},
    {"ObjectCollisionEstimator", {6, 0, 0}}};
  EXPECT_TRUE(validate_hot_path_lineage(front));

  auto both = front;
  both["RearLidarDriver"] = {1, 0, 0};
  both["PointsTransformerRear"] = {2, 0, 0};
  EXPECT_TRUE(validate_hot_path_lineage(both));

  front["ForeignNode"] = {7, 0, 0};
  EXPECT_FALSE(validate_hot_path_lineage(front));
}

TEST(SampleManagement, DbwValidatorRequiresExactTwentyOneNodeInput)
{
  const std::vector<std::string> names{
    "FrontLidarDriver", "RearLidarDriver", "PointsTransformerFront",
    "PointsTransformerRear", "PointCloudFusion", "VoxelGridDownsampler",
    "RayGroundFilter", "EuclideanClusterDetector", "ObjectCollisionEstimator",
    "PointCloudMap", "PointCloudMapLoader", "NDTLocalizer", "Visualizer",
    "Lanelet2GlobalPlanner", "Lanelet2Map", "Lanelet2MapLoader", "ParkingPlanner",
    "LanePlanner", "BehaviorPlanner", "MPCController", "VehicleInterface"};
  std::map<std::string, node_map_t> nodes;
  uint64_t timestamp = 0;
  for (const auto & name : names) {
    nodes[name] = {++timestamp, 0, 0};
  }
  EXPECT_TRUE(validate_dbw_lineage(nodes));

  nodes["ForeignNode"] = {++timestamp, 0, 0};
  EXPECT_FALSE(validate_dbw_lineage(nodes));
}

TEST(SampleManagement, IntersectionValidatorRequiresExactTwoNodeInput)
{
  std::map<std::string, node_map_t> nodes{
    {"EuclideanClusterSettings", {1, 0, 0}},
    {"EuclideanClusterDetector", {2, 0, 0}}};
  EXPECT_TRUE(validate_intersection_lineage(nodes));

  nodes["ForeignNode"] = {3, 0, 0};
  EXPECT_FALSE(validate_intersection_lineage(nodes));
}

TEST(SampleManagement, SerializesSchemaVersionTwoSourceAndCompletionRecords)
{
  const auto completion = make_structured_chain_record(
    "chain", "FrontLidarDriver", 7, 100, "sink", 8, 150, 50,
    std::vector<std::string>{"FrontLidarDriver", "middle", "FrontLidarDriver", "sink"},
    std::vector<source_identity_t>{{"RearLidarDriver", 3, 100},
      {"FrontLidarDriver", 7, 100}, {"RearLidarDriver", 4, 101}},
    "completed", 2);
  const auto source = make_structured_source_record("FrontLidarDriver", 7, 100);

  EXPECT_EQ('\n', completion.back());
  EXPECT_NE(std::string::npos, completion.find("\"schema_version\":2"));
  EXPECT_NE(std::string::npos, completion.find("\"record_type\":\"completion\""));
  EXPECT_NE(std::string::npos, completion.find("\"source_sequence\":7"));
  EXPECT_NE(std::string::npos, completion.find("\"drop_count\":2"));
  EXPECT_NE(std::string::npos, completion.find("\"metadata_overflow_count\":"));
  EXPECT_NE(
    std::string::npos,
    completion.find(
      "\"contributing_source_roots\":[{\"source_node\":\"FrontLidarDriver\","
      "\"source_sequence\":7,\"source_timestamp_ns\":100},{\"source_node\":"
      "\"RearLidarDriver\",\"source_sequence\":4,\"source_timestamp_ns\":101}]"));
  EXPECT_NE(std::string::npos, completion.find("\"lineage\":[\"FrontLidarDriver\",\"middle\",\"sink\"]"));
  EXPECT_NE(std::string::npos, source.find("\"schema_version\":2,\"record_type\":\"source\""));
  EXPECT_NE(std::string::npos, source.find("\"source_node\":\"FrontLidarDriver\""));
  EXPECT_NE(std::string::npos, source.find("\"instrumentation_error_count\":"));
}

TEST(SampleManagement, EscapesJsonAndKeepsBoundedCompletionWithinPipeBuf)
{
  EXPECT_EQ("node\\\"\\\\\\n", json_escape("node\"\\\n"));

  std::vector<std::string> lineage;
  lineage.reserve(message_t::STATS_CAPACITY);
  for (uint64_t i = 0; i < message_t::STATS_CAPACITY; ++i) {
    lineage.push_back(std::string(44, 'n') + std::to_string(i));
  }
  const auto record = make_structured_chain_record(
    "chain", lineage.front(), 1, 1, "sink", 1, 2, 1, lineage,
    std::vector<source_identity_t>{{lineage.front(), 1, 1}, {lineage.back(), 2, 2}},
    "completed", 0);
  const auto errors_before = structured_record_error_count().load();

  EXPECT_LE(record.size(), static_cast<size_t>(PIPE_BUF));
  EXPECT_FALSE(write_structured_record(std::string(PIPE_BUF + 1, 'x')));
  EXPECT_EQ(errors_before + 1, structured_record_error_count().load());
}

TEST(SampleManagement, StopsAtMetadataCapacity)
{
  message_t destination{};
  for (uint32_t sequence = 0; sequence < message_t::STATS_CAPACITY; ++sequence) {
    set_sample("Existing" + std::to_string(sequence), sequence, 0, sequence, destination);
  }
  message_t source{};
  set_sample("FrontLidarDriver", 1, 0, 100, source);
  const auto before = metadata_overflow_count().load();

  merge_history_into_sample(destination, &source);

  EXPECT_EQ(message_t::STATS_CAPACITY, destination.size);
  EXPECT_EQ(before + 1, metadata_overflow_count().load());
}
