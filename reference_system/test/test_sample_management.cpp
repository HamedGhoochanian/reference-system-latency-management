// Copyright 2026 Eindhoven University of Technology
// Licensed under the Apache License, Version 2.0

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "reference_system/nodes/rclcpp/command.hpp"
#include "reference_system/nodes/rclcpp/cyclic.hpp"
#include "reference_system/nodes/rclcpp/sensor.hpp"
#include "reference_system/nodes/rclcpp/transform.hpp"
#include "reference_system/sample_management.hpp"

TEST(SampleManagement, RejectsBackwardTimestamps)
{
  uint64_t elapsed = 0;
  auto before = structured_record_error_count().load();
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

TEST(SampleManagement, SerializesOneCompleteJsonLine)
{
  auto record = make_structured_chain_record(
    "chain", "source", 7, 100, "sink", 8, 150, 50,
    std::vector<std::string>{"source", "sink"}, "completed", 2);

  EXPECT_EQ('\n', record.back());
  EXPECT_EQ(std::string::npos, record.substr(0, record.size() - 1).find('\n'));
  EXPECT_NE(std::string::npos, record.find("\"source_sequence\":7"));
  EXPECT_NE(std::string::npos, record.find("\"drop_count\":2"));
  EXPECT_NE(std::string::npos, record.find("\"metadata_overflow_count\":"));
}

TEST(SampleManagement, PreservesDifferentInstancesFromSameNode)
{
  message_t destination{};
  destination.size = 0;
  message_t source{};
  source.size = 0;
  set_sample("FrontLidarDriver", 1, 0, 100, source);
  set_sample("FrontLidarDriver", 2, 0, 200, source);

  merge_history_into_sample(destination, &source);

  ASSERT_EQ(2U, destination.size);
  EXPECT_EQ(1U, destination.stats[0].sequence_number);
  EXPECT_EQ(2U, destination.stats[1].sequence_number);
  EXPECT_EQ(1U, extract_source_identity(&destination, true).sequence_number);
  EXPECT_EQ(2U, extract_source_identity(&destination, false).sequence_number);
}

TEST(SampleManagement, StopsAtMetadataCapacity)
{
  message_t destination{};
  for (uint32_t sequence = 0; sequence < message_t::STATS_CAPACITY; ++sequence) {
    set_sample("Existing", sequence, 0, sequence, destination);
  }
  message_t source{};
  source.size = 0;
  set_sample("FrontLidarDriver", 1, 0, 100, source);
  auto before = metadata_overflow_count().load();

  merge_history_into_sample(destination, &source);

  EXPECT_EQ(message_t::STATS_CAPACITY, destination.size);
  EXPECT_EQ(before + 1, metadata_overflow_count().load());
}
