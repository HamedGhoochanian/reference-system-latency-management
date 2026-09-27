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
#ifndef REFERENCE_SYSTEM__NODES__RCLCPP__TRANSFORM_HPP_
#define REFERENCE_SYSTEM__NODES__RCLCPP__TRANSFORM_HPP_
#include <sys/time.h>

#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "reference_system/nodes/settings.hpp"
#include "reference_system/number_cruncher.hpp"
#include "reference_system/sample_management.hpp"
#include "reference_system/msg_types.hpp"

namespace nodes
{
namespace rclcpp_system
{

class Transform : public rclcpp::Node
{
public:
  explicit Transform(const TransformSettings & settings)
  : Node(settings.node_name),
    number_crunch_limit_(settings.number_crunch_limit),
    chain_id_(settings.chain_id),
    source_candidate_names_(settings.source_candidate_names),
    configured_source_name_(settings.configured_source_name)
  {
    event_logger_ = reference_system::events::make_component_event_logger(settings.node_name);
    subscription_ = this->create_subscription<message_t>(
      settings.input_topic, 1,
      [this](const message_t::SharedPtr msg) {input_callback(msg);});
    publisher_ = this->create_publisher<message_t>(settings.output_topic, 1);
#ifdef PICAS
    subscription_->callback_priority = settings.callback_priority;
#endif
  }

private:
  struct timeval c1, c2;
  void input_callback(const message_t::SharedPtr input_message)
  {
    uint64_t timestamp = now_as_int();
    const uint32_t output_sequence = sequence_number_++;
    reference_system::events::SourceExecutionId callback_id{};
    if (event_logger_) {
      const auto entry = event_logger_->input_entry(
        output_sequence, 0U, event_input_lineage(input_message));
      timestamp = entry.timestamp_ns;
      callback_id = {this->get_name(), output_sequence, timestamp};
    }
    auto number_cruncher_result = number_cruncher(number_crunch_limit_);
    gettimeofday(&c1, NULL);
    auto output_message = publisher_->borrow_loaned_message();
    output_message.get().size = 0;
    merge_history_into_sample(output_message.get(), input_message);

    uint32_t missed_samples = get_missed_samples_and_update_seq_nr(
      input_message,
      input_sequence_number_);

    set_sample(this->get_name(), output_sequence, missed_samples, timestamp, output_message.get());

    if (event_logger_) {
      event_logger_->output_dependencies(output_sequence, timestamp, {{0U, callback_id}});
    }

    std::string node_name = this->get_name();
    std::map<std::string, node_map_t> nodes;
    std::vector<source_identity_t> roots;
    std::vector<std::string> lineage;
    std::optional<source_identity_t> configured_source;
    bool chain_lineage_valid = false;
    if (!chain_id_.empty()) {
      nodes = build_node_map(&output_message.get());
      roots = find_source_identities(nodes, source_candidate_names_);
      lineage = extract_lineage(&output_message.get());
      configured_source = find_source_identity(nodes, configured_source_name_);
      chain_lineage_valid = validate_hot_path_lineage(nodes);
    }

    // use result so that it is not optimizied away by some clever compiler
    output_message.get().data[0] = number_cruncher_result;
    publisher_->publish(std::move(output_message));
    gettimeofday(&c2, NULL);
    print_execution_time(
      "Transform", this->get_name(),
      (c2.tv_sec - c1.tv_sec) * 1000000 + (c2.tv_usec - c1.tv_usec));

    if (!chain_id_.empty()) {
      const uint64_t legacy_sink_timestamp = now_as_int();
      if (is_structured_output_enabled() && configured_source && chain_lineage_valid)
      {
        uint64_t latency = 0;
        if (elapsed_ns(configured_source->timestamp, legacy_sink_timestamp, latency)) {
          const uint32_t drops = sum_drops(input_message, nodes);
          emit_structured_chain_record(
            chain_id_,
            configured_source->node_name, configured_source->sequence_number,
            configured_source->timestamp,
            node_name, output_sequence, legacy_sink_timestamp,
            latency, lineage, roots, deadline_status(latency, 500000000ULL), drops);
        }
      }

      if (event_logger_) {
        const std::optional<reference_system::events::SourceExecutionId> source_id =
          configured_source && chain_lineage_valid ?
          std::optional<reference_system::events::SourceExecutionId>{
            to_event_source_id(*configured_source)} : std::nullopt;
        const std::string lineage_error = !configured_source ?
          "configured_source_missing:" + configured_source_name_ :
          !chain_lineage_valid ? "chain_lineage_invalid" : "";
        event_logger_->sink_finish(
          chain_id_,
          source_id,
          output_sequence,
          to_event_source_ids(roots),
          event_lineage(nodes),
          lineage_error);
      }
    }
  }

private:
  rclcpp::Publisher<message_t>::SharedPtr publisher_;
  rclcpp::Subscription<message_t>::SharedPtr subscription_;
  uint64_t number_crunch_limit_;
  std::string chain_id_;
  std::vector<std::string> source_candidate_names_;
  std::string configured_source_name_;
  std::unique_ptr<reference_system::events::ComponentEventLogger> event_logger_;
  uint32_t sequence_number_ = 0;
  uint32_t input_sequence_number_ = 0;
};
}  // namespace rclcpp_system
}  // namespace nodes
#endif  // REFERENCE_SYSTEM__NODES__RCLCPP__TRANSFORM_HPP_
