// Copyright 2026 Eindhoven University of Technology
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cerrno>
#include <unistd.h>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include "reference_system/sample_management.hpp"

namespace
{
std::filesystem::path temporary_event_directory()
{
  const auto name = "rslm-events-" + std::to_string(getpid()) + "-" +
    std::to_string(reference_system::events::monotonic_now_ns());
  const auto path = std::filesystem::temp_directory_path() / name;
  std::filesystem::create_directories(path);
  return path;
}

std::vector<std::string> read_lines(const std::filesystem::path & path)
{
  std::ifstream input(path);
  std::vector<std::string> lines;
  std::string line;
  while (std::getline(input, line)) {lines.push_back(line);}
  return lines;
}

uint64_t json_integer(const std::string & line, const std::string & key)
{
  const auto key_position = line.find("\"" + key + "\":");
  if (key_position == std::string::npos) {throw std::runtime_error("missing JSON integer");}
  const auto first_digit = key_position + key.size() + 3;
  return std::stoull(line.substr(first_digit));
}

std::string host_boot_id()
{
  std::ifstream input("/proc/sys/kernel/random/boot_id");
  std::string value;
  std::getline(input, value);
  return value;
}

std::atomic<bool> slow_write_started{false};

ssize_t slow_write(int fd, const void * data, size_t size)
{
  slow_write_started.store(true);
  std::this_thread::sleep_for(std::chrono::milliseconds{100});
  return ::write(fd, data, size);
}

ssize_t failing_write(int, const void *, size_t)
{
  errno = EIO;
  return -1;
}
}  // namespace

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

TEST(SampleManagement, CallbackClockUsesClockMonotonic)
{
  timespec before{};
  timespec after{};
  ASSERT_EQ(0, clock_gettime(CLOCK_MONOTONIC, &before));
  const uint64_t measured = now_as_int();
  ASSERT_EQ(0, clock_gettime(CLOCK_MONOTONIC, &after));

  const uint64_t before_ns = static_cast<uint64_t>(before.tv_sec) * 1000000000ULL +
    static_cast<uint64_t>(before.tv_nsec);
  const uint64_t after_ns = static_cast<uint64_t>(after.tv_sec) * 1000000000ULL +
    static_cast<uint64_t>(after.tv_nsec);
  EXPECT_LE(before_ns, measured);
  EXPECT_LE(measured, after_ns);
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

TEST(SampleManagement, ConfiguredTriggerIdentityDoesNotFollowNewestRoot)
{
  const std::map<std::string, node_map_t> nodes{
    {"FrontLidarDriver", {100, 7, 0}},
    {"RearLidarDriver", {200, 4, 0}}};

  const auto trigger = find_source_identity(nodes, "FrontLidarDriver");

  ASSERT_TRUE(trigger);
  EXPECT_EQ("FrontLidarDriver", trigger->node_name);
  EXPECT_EQ(7U, trigger->sequence_number);
  EXPECT_EQ(100U, trigger->timestamp);
  EXPECT_FALSE(find_source_identity(nodes, "MissingTrigger"));
}

TEST(SampleManagement, FusionInputZeroTriggerSurvivesANewerCachedOtherInput)
{
  message_t input_zero{};
  message_t cached_input_one{};
  message_t fused{};
  set_sample("FrontLidarDriver", 7U, 0U, 100U, input_zero);
  set_sample("RearLidarDriver", 4U, 0U, 200U, cached_input_one);

  merge_history_into_sample(fused, &input_zero);
  merge_history_into_sample(fused, &cached_input_one);

  const auto nodes = build_node_map(&fused);
  const auto triggering_source = find_source_identity(nodes, "FrontLidarDriver");
  ASSERT_TRUE(triggering_source);
  EXPECT_EQ(7U, triggering_source->sequence_number);
  EXPECT_EQ(100U, triggering_source->timestamp);
}

TEST(EventLogging, WritesMonotonicSourceAndSinkEventsWithProgress)
{
  const auto directory = temporary_event_directory();
  uint64_t source_time = 0;
  {
    reference_system::events::ComponentEventLogger logger(
      "FrontLidarDriver", directory.string(), "run-test", std::chrono::milliseconds{5});
    const auto source = logger.source_entry(
      7U, std::vector<std::string>{"perception_collision_hot_path"});
    ASSERT_TRUE(source.queued);
    source_time = source.timestamp_ns;

    std::this_thread::sleep_for(std::chrono::milliseconds{2});
    const auto sink = logger.sink_finish(
      "perception_collision_hot_path",
      reference_system::events::SourceExecutionId{"FrontLidarDriver", 7U, source_time},
      3U,
      std::vector<reference_system::events::SourceExecutionId>{
        {"FrontLidarDriver", 7U, source_time}, {"RearLidarDriver", 4U, source_time + 1}},
      std::vector<reference_system::events::SourceExecutionId>{
        {"FrontLidarDriver", 7U, source_time}, {"ObjectCollisionEstimator", 3U, 0}});
    ASSERT_TRUE(sink.queued);
    EXPECT_GT(sink.timestamp_ns, source_time);
  }

  const auto lines = read_lines(directory / "FrontLidarDriver.jsonl");
  ASSERT_GE(lines.size(), 4U);
  uint64_t previous_sequence = 0;
  uint64_t previous_watermark = 0;
  bool found_source = false;
  bool found_sink = false;
  bool found_progress = false;
  for (size_t i = 0; i < lines.size(); ++i) {
    EXPECT_NE(std::string::npos, lines[i].find("\"host_boot_id\":\"" + host_boot_id() + "\""));
    EXPECT_NE(std::string::npos, lines[i].find("\"run_id\":\"run-test\""));
    const uint64_t event_sequence = json_integer(lines[i], "event_seq");
    EXPECT_EQ(previous_sequence + 1, event_sequence);
    previous_sequence = event_sequence;

    if (lines[i].find("\"record_type\":\"source_entry\"") != std::string::npos) {
      found_source = true;
      EXPECT_NE(
        std::string::npos,
        lines[i].find("\"source_execution_id\":{\"component\":\"FrontLidarDriver\",\"sequence\":7,\"timestamp_ns\":" +
          std::to_string(source_time) + "}"));
      EXPECT_NE(std::string::npos, lines[i].find("perception_collision_hot_path"));
    }
    if (lines[i].find("\"record_type\":\"sink_finish\"") != std::string::npos) {
      found_sink = true;
      EXPECT_NE(std::string::npos, lines[i].find("\"chain_id\":\"perception_collision_hot_path\""));
      EXPECT_NE(
        std::string::npos,
        lines[i].find("\"source_execution_id\":{\"component\":\"FrontLidarDriver\",\"sequence\":7"));
    }
    if (lines[i].find("\"record_type\":\"progress\"") != std::string::npos) {
      found_progress = true;
      const uint64_t watermark = json_integer(lines[i], "delivery_through_ns");
      EXPECT_GE(watermark, previous_watermark);
      previous_watermark = watermark;
      for (size_t later = i + 1; later < lines.size(); ++later) {
        if (lines[later].find("\"record_type\":\"progress\"") == std::string::npos &&
          lines[later].find("\"record_type\":\"producer_error\"") == std::string::npos)
        {
          EXPECT_GT(json_integer(lines[later], "timestamp_ns"), watermark);
        }
      }
    }
  }
  EXPECT_TRUE(found_source);
  EXPECT_TRUE(found_sink);
  EXPECT_TRUE(found_progress);
  std::filesystem::remove_all(directory);
}

TEST(EventLogging, ConcurrentEventsStayBeforeAnyWatermarkThatCoversThem)
{
  const auto directory = temporary_event_directory();
  {
    reference_system::events::ComponentEventLogger logger(
      "ConcurrentSource", directory.string(), "run-concurrent", std::chrono::milliseconds{2});
    std::vector<std::thread> threads;
    std::atomic<bool> all_queued{true};
    for (uint32_t worker = 0; worker < 4; ++worker) {
      threads.emplace_back([&logger, &all_queued, worker] {
        for (uint32_t i = 0; i < 25; ++i) {
          if (!logger.source_entry(worker * 25 + i, {}).queued) {all_queued.store(false);}
        }
      });
    }
    for (auto & thread : threads) {thread.join();}
    EXPECT_TRUE(all_queued.load());
  }

  const auto lines = read_lines(directory / "ConcurrentSource.jsonl");
  ASSERT_GT(lines.size(), 100U);
  uint64_t previous_sequence = 0;
  for (size_t i = 0; i < lines.size(); ++i) {
    const uint64_t sequence = json_integer(lines[i], "event_seq");
    EXPECT_EQ(previous_sequence + 1, sequence);
    previous_sequence = sequence;
    if (lines[i].find("\"record_type\":\"progress\"") == std::string::npos) {continue;}

    const uint64_t watermark = json_integer(lines[i], "delivery_through_ns");
    for (size_t later = i + 1; later < lines.size(); ++later) {
      if (lines[later].find("\"record_type\":\"progress\"") == std::string::npos &&
        lines[later].find("\"record_type\":\"producer_error\"") == std::string::npos)
      {
        EXPECT_GT(json_integer(lines[later], "timestamp_ns"), watermark);
      }
    }
  }
  std::filesystem::remove_all(directory);
}

TEST(EventLogging, ReportsDeliveryWhileProducerIsRunning)
{
  const auto directory = temporary_event_directory();
  {
    reference_system::events::ComponentEventLogger logger(
      "LiveSource", directory.string(), "run-live", std::chrono::milliseconds{2});
    const auto source = logger.source_entry(1U, {"chain"});
    bool delivered = false;
    for (int attempt = 0; attempt < 100 && !delivered; ++attempt) {
      std::this_thread::sleep_for(std::chrono::milliseconds{1});
      for (const auto & line : read_lines(directory / "LiveSource.jsonl")) {
        if (line.find("\"record_type\":\"progress\"") != std::string::npos &&
          json_integer(line, "delivery_through_ns") >= source.timestamp_ns)
        {
          delivered = true;
        }
      }
    }
    EXPECT_TRUE(delivered);
    EXPECT_TRUE(logger.healthy());
  }
  std::filesystem::remove_all(directory);
}

TEST(EventLogging, MissingSourceCannotClaimCompleteLineage)
{
  const auto directory = temporary_event_directory();
  {
    reference_system::events::ComponentEventLogger logger(
      "MissingSourceSink", directory.string(), "run-missing");
    logger.sink_finish("chain", std::nullopt, 1U, {}, {});
  }
  const auto lines = read_lines(directory / "MissingSourceSink.jsonl");
  ASSERT_EQ(3U, lines.size());
  EXPECT_NE(std::string::npos, lines[1].find("\"lineage_complete\":false"));
  EXPECT_NE(std::string::npos, lines[1].find("\"lineage_error\":\"source_match_unavailable\""));
  std::filesystem::remove_all(directory);
}

TEST(EventLogging, UnconfiguredDBWSinkIsNotGivenASourceMatch)
{
  const auto directory = temporary_event_directory();
  {
    reference_system::events::ComponentEventLogger logger(
      "VehicleDBWSystem", directory.string(), "run-dbw");
    const auto sink = logger.sink_finish(
      "perception_localization_planning_control_to_dbw",
      std::nullopt,
      9U,
      std::vector<reference_system::events::SourceExecutionId>{
        {"FrontLidarDriver", 7U, 100U}, {"EuclideanClusterSettings", 3U, 200U}},
      std::vector<reference_system::events::SourceExecutionId>{
        {"FrontLidarDriver", 7U, 100U}, {"VehicleInterface", 8U, 300U}},
      "chain_source_unconfigured",
      "unconfigured");
    EXPECT_TRUE(sink.queued);
  }

  const auto lines = read_lines(directory / "VehicleDBWSystem.jsonl");
  ASSERT_EQ(3U, lines.size());
  EXPECT_NE(std::string::npos, lines[1].find("\"record_type\":\"sink_finish\""));
  EXPECT_NE(std::string::npos, lines[1].find("\"source_execution_id\":null"));
  EXPECT_NE(std::string::npos, lines[1].find("\"source_selection\":\"unconfigured\""));
  EXPECT_NE(std::string::npos, lines[1].find("\"lineage_complete\":false"));
  EXPECT_NE(std::string::npos, lines[1].find("FrontLidarDriver"));
  EXPECT_NE(std::string::npos, lines[1].find("EuclideanClusterSettings"));
  std::filesystem::remove_all(directory);
}

TEST(EventLogging, QueueOverflowWritesErrorAndStopsProgress)
{
  const auto directory = temporary_event_directory();
  slow_write_started.store(false);
  {
    reference_system::events::ComponentEventLogger logger(
      "SlowWriter", directory.string(), "run-overflow", std::chrono::milliseconds{2}, 1,
      slow_write);
    for (int attempt = 0; attempt < 100 && !slow_write_started.load(); ++attempt) {
      std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
    ASSERT_TRUE(slow_write_started.load());
    EXPECT_TRUE(logger.source_entry(1U, {}).queued);
    EXPECT_THROW(logger.source_entry(2U, {}), std::runtime_error);
    EXPECT_FALSE(logger.healthy());
  }
  const auto lines = read_lines(directory / "SlowWriter.jsonl");
  ASSERT_FALSE(lines.empty());
  EXPECT_NE(std::string::npos, lines.back().find("\"record_type\":\"producer_error\""));
  EXPECT_NE(std::string::npos, lines.back().find("event_queue_full"));
  for (const auto & line : lines) {
    EXPECT_EQ(std::string::npos, line.find("\"record_type\":\"progress\""));
  }
  std::filesystem::remove_all(directory);
}

TEST(EventLogging, WriteFailureCannotProduceDeliveryProgress)
{
  const auto directory = temporary_event_directory();
  {
    reference_system::events::ComponentEventLogger logger(
      "BrokenWriter", directory.string(), "run-write-error", std::chrono::milliseconds{2}, 8,
      failing_write);
    for (int attempt = 0; attempt < 100 && logger.healthy(); ++attempt) {
      std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
    EXPECT_FALSE(logger.healthy());
  }
  const auto lines = read_lines(directory / "BrokenWriter.jsonl");
  for (const auto & line : lines) {
    EXPECT_EQ(std::string::npos, line.find("\"record_type\":\"progress\""));
  }
  std::filesystem::remove_all(directory);
}

TEST(EventLogging, RejectsMissingRunDirectoryAndDuplicateProducerFile)
{
  const auto directory = temporary_event_directory();
  EXPECT_THROW(
    reference_system::events::ComponentEventLogger(
      "MissingDirectory", (directory / "missing").string(), "run"),
    std::runtime_error);
  {
    reference_system::events::ComponentEventLogger first(
      "OneProducer", directory.string(), "run");
    EXPECT_THROW(
      reference_system::events::ComponentEventLogger second(
        "OneProducer", directory.string(), "run"),
      std::runtime_error);
  }
  std::filesystem::remove_all(directory);
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
