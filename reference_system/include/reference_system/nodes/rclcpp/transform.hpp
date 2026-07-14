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
#include <chrono>
#include <string>
#include <utility>
#include <iostream>
#include <sys/time.h>
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
    number_crunch_limit_(settings.number_crunch_limit)
  {
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
    auto number_cruncher_result = number_cruncher(number_crunch_limit_);
    gettimeofday(&c1, NULL);
    auto output_message = publisher_->borrow_loaned_message();
    output_message.get().size = 0;
    merge_history_into_sample(output_message.get(), input_message);

    uint32_t missed_samples = get_missed_samples_and_update_seq_nr(
      input_message,
      input_sequence_number_);

    set_sample(
      this->get_name(), sequence_number_++, missed_samples, timestamp,
      output_message.get());

    uint64_t sink_timestamp = now_as_int();

    std::string node_name = this->get_name();
    if (is_structured_output_enabled() && node_name == "ObjectCollisionEstimator") {
      auto nodes = build_node_map(&output_message.get());
      if (validate_hot_path_lineage(nodes)) {
        static const std::vector<std::string> lidar_sources{
          "FrontLidarDriver", "RearLidarDriver"};
        const auto roots = extract_source_roots(&output_message.get(), lidar_sources);
        auto src_id = select_source_reference(roots, false);
        if (!src_id.node_name.empty()) {
          uint64_t latency;
          if (elapsed_ns(src_id.timestamp, sink_timestamp, latency)) {
            uint32_t drops = sum_drops(&output_message.get(), nodes);
            std::vector<std::string> lineage = extract_lineage(&output_message.get());
            emit_structured_chain_record(
              "perception_collision_hot_path",
              src_id.node_name, src_id.sequence_number, src_id.timestamp,
              node_name, sequence_number_ - 1, sink_timestamp,
              latency, lineage, roots, deadline_status(latency, 500000000ULL), drops);
          }
        }
      }
    }

    // use result so that it is not optimizied away by some clever compiler
    output_message.get().data[0] = number_cruncher_result;
    publisher_->publish(std::move(output_message));
    gettimeofday(&c2, NULL);
    print_execution_time(
      "Transform", this->get_name(),
      (c2.tv_sec - c1.tv_sec) * 1000000 + (c2.tv_usec - c1.tv_usec));
  }

private:
  rclcpp::Publisher<message_t>::SharedPtr publisher_;
  rclcpp::Subscription<message_t>::SharedPtr subscription_;
  uint64_t number_crunch_limit_;
  uint32_t sequence_number_ = 0;
  uint32_t input_sequence_number_ = 0;
};
}  // namespace rclcpp_system
}  // namespace nodes
#endif  // REFERENCE_SYSTEM__NODES__RCLCPP__TRANSFORM_HPP_
