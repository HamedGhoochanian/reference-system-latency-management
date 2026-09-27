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

#include <sys/time.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>
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
  : Node(settings.node_name),
    chain_id_(settings.chain_id),
    source_candidate_names_(settings.source_candidate_names),
    configured_source_name_(settings.configured_source_name)
  {
    if (!chain_id_.empty()) {
      event_logger_ = reference_system::events::make_component_event_logger(settings.node_name);
    }
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

    const std::string node_name = this->get_name();
    const auto nodes = build_node_map(input_message);
    const auto lineage_names = extract_lineage(input_message);
    const auto lineage_ids = event_lineage(nodes);
    const auto source_candidates = find_source_identities(nodes, source_candidate_names_);

    const std::optional<source_identity_t> intersection_source =
      find_source_identity(nodes, configured_source_name_);
    const bool intersection_lineage_valid = validate_intersection_lineage(nodes);

    if (is_legacy_verbose_output_enabled()) {
      print_sample_path(this->get_name(), missed_samples, input_message);
    }

    gettimeofday(&c2, NULL);
    print_execution_time(
      "Command", this->get_name(),
      (c2.tv_sec - c1.tv_sec) * 1000000 + (c2.tv_usec - c1.tv_usec));

    if (node_name == "VehicleDBWSystem") {
      if (event_logger_) {
        const auto earliest = oldest_source(input_message, source_candidate_names_);
        event_logger_->sink_finish(
          chain_id_,
          earliest ? std::optional<reference_system::events::SourceExecutionId>{
            to_event_source_id(*earliest)} : std::nullopt,
          sink_sequence,
          to_event_source_ids(source_candidates),
          lineage_ids,
          earliest ? "" : "earliest_source_missing",
          "earliest_contributor",
          event_input_lineage(input_message));
      }
    } else if (node_name == "IntersectionOutput") {
      const uint64_t legacy_sink_timestamp = now_as_int();
      if (is_structured_output_enabled() && intersection_source && intersection_lineage_valid)
      {
        uint64_t latency = 0;
        if (elapsed_ns(intersection_source->timestamp, legacy_sink_timestamp, latency)) {
          const std::vector<source_identity_t> roots{*intersection_source};
          const uint32_t drops = sum_drops(input_message, nodes) + missed_samples;
          std::vector<std::string> lineage = lineage_names;
          lineage.push_back(node_name);
          emit_structured_chain_record(
            "euclidean_settings_to_intersection_output",
            intersection_source->node_name, intersection_source->sequence_number,
            intersection_source->timestamp,
            node_name, sink_sequence, legacy_sink_timestamp,
            latency, lineage, roots, deadline_status(latency, 250000000ULL), drops);
        }
      }

      if (event_logger_) {
        const std::optional<reference_system::events::SourceExecutionId> source_id =
          intersection_source && intersection_lineage_valid ?
          std::optional<reference_system::events::SourceExecutionId>{
            to_event_source_id(*intersection_source)} : std::nullopt;
        const std::string lineage_error = !intersection_source ?
          "configured_source_missing:EuclideanClusterSettings" :
          !intersection_lineage_valid ? "chain_lineage_invalid" : "";
        event_logger_->sink_finish(
          chain_id_,
          source_id,
          sink_sequence,
          intersection_source ? to_event_source_ids({*intersection_source}) :
          std::vector<reference_system::events::SourceExecutionId>{},
          lineage_ids,
          lineage_error);
      }
    }
  }

private:
  rclcpp::Subscription<message_t>::SharedPtr subscription_;
  std::string chain_id_;
  std::vector<std::string> source_candidate_names_;
  std::string configured_source_name_;
  uint32_t sequence_number_ = 0;
  uint32_t sink_sequence_number_ = 0;
  std::unique_ptr<reference_system::events::ComponentEventLogger> event_logger_;
};
}  // namespace rclcpp_system
}  // namespace nodes
#endif  // REFERENCE_SYSTEM__NODES__RCLCPP__COMMAND_HPP_
