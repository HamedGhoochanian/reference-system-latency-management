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
#ifndef REFERENCE_SYSTEM__NODES__RCLCPP__COMMAND_HPP_
#define REFERENCE_SYSTEM__NODES__RCLCPP__COMMAND_HPP_

#include <chrono>
#include <string>
#include <iostream>
#include <sys/time.h>
#include "rclcpp/rclcpp.hpp"
#include "reference_system/nodes/settings.hpp"
#include "reference_system/sample_management.hpp"
#include "reference_system/msg_types.hpp"

namespace nodes
{
namespace rclcpp_system
{

class Command : public rclcpp::Node
{
public:
  explicit Command(const CommandSettings & settings)
  : Node(settings.node_name)
  {
    subscription_ = this->create_subscription<message_t>(
      settings.input_topic, 10,
      [this](const message_t::SharedPtr msg) {input_callback(msg);});
#ifdef PICAS
    subscription_->callback_priority = settings.callback_priority;
#endif
  }

private:
  struct timeval c1, c2;
  void input_callback(const message_t::SharedPtr input_message)
  {
    gettimeofday(&c1, NULL);
    uint32_t missed_samples = get_missed_samples_and_update_seq_nr(input_message, sequence_number_);
    uint32_t sink_sequence = sink_sequence_number_++;

    uint64_t sink_timestamp = now_as_int();

    if (is_structured_output_enabled()) {
      std::string node_name = this->get_name();
      auto nodes = build_node_map(input_message);
      std::vector<std::string> lineage = extract_lineage(input_message);
      lineage.push_back(node_name);

      if (node_name == "VehicleDBWSystem" && validate_dbw_lineage(nodes)) {
        auto src_id = extract_source_identity(input_message, true);
        if (!src_id.node_name.empty()) {
          uint64_t latency;
          if (elapsed_ns(src_id.timestamp, sink_timestamp, latency)) {
            uint32_t drops = sum_drops(input_message, nodes) + missed_samples;
            emit_structured_chain_record(
              "perception_localization_planning_control_to_dbw",
              src_id.node_name, src_id.sequence_number, src_id.timestamp,
              node_name, sink_sequence, sink_timestamp,
              latency, lineage, deadline_status(latency, 1000000000ULL), drops);
          }
        }
      } else if (node_name == "IntersectionOutput" && validate_intersection_lineage(nodes)) {
        auto it = nodes.find("EuclideanClusterSettings");
        if (it != nodes.end()) {
          uint64_t src_ts = it->second.timestamp;
          uint64_t latency;
          if (elapsed_ns(src_ts, sink_timestamp, latency)) {
            uint32_t drops = sum_drops(input_message, nodes) + missed_samples;
            emit_structured_chain_record(
              "euclidean_settings_to_intersection_output",
              "EuclideanClusterSettings", it->second.sequence_number, src_ts,
              node_name, sink_sequence, sink_timestamp,
              latency, lineage, deadline_status(latency, 250000000ULL), drops);
          }
        }
      }
    }

    if (is_legacy_verbose_output_enabled()) {
      print_sample_path(this->get_name(), missed_samples, input_message);
    }

    gettimeofday(&c2, NULL);
    print_execution_time(
      "Command", this->get_name(),
      (c2.tv_sec - c1.tv_sec) * 1000000 + (c2.tv_usec - c1.tv_usec));

  }

private:
  rclcpp::Subscription<message_t>::SharedPtr subscription_;
  uint32_t sequence_number_ = 0;
  uint32_t sink_sequence_number_ = 0;
};
}  // namespace rclcpp_system
}  // namespace nodes
#endif  // REFERENCE_SYSTEM__NODES__RCLCPP__COMMAND_HPP_
