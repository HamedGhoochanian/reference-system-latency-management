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
#ifndef REFERENCE_SYSTEM__NODES__RCLCPP__INTERSECTION_HPP_
#define REFERENCE_SYSTEM__NODES__RCLCPP__INTERSECTION_HPP_
#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>
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

class Intersection : public rclcpp::Node
{
public:
  explicit Intersection(const IntersectionSettings & settings)
  : Node(settings.node_name)
  {
    event_logger_ = reference_system::events::make_component_event_logger(settings.node_name);
    for (auto & connection : settings.connections) {
      connections_.emplace_back(
        Connection{
            this->create_publisher<message_t>(connection.output_topic, 1),
            this->create_subscription<message_t>(
              connection.input_topic, 1,
              [this, id = connections_.size()](const message_t::SharedPtr msg) {
                input_callback(msg, id);
              }),
            connection.number_crunch_limit
          });
    }
#ifdef PICAS
    connections_[0].subscription->callback_priority = settings.connections[0].callback_priority;
    connections_[1].subscription->callback_priority = settings.connections[1].callback_priority;
#endif
  }

private:
  struct timeval c1, c2;
  void input_callback(const message_t::SharedPtr input_message, const uint64_t id)
  {
    uint64_t timestamp = now_as_int();
    const uint32_t output_sequence = connections_[id].sequence_number++;
    reference_system::events::SourceExecutionId callback_id{};
    if (event_logger_) {
      const uint32_t callback_sequence = input_callback_sequence_++;
      const auto entry = event_logger_->input_entry(
        callback_sequence, id, event_input_lineage(input_message));
      timestamp = entry.timestamp_ns;
      callback_id = {this->get_name(), callback_sequence, timestamp};
    }
    auto number_cruncher_result = number_cruncher(connections_[id].number_crunch_limit);
    gettimeofday(&c1, NULL);
    auto output_message = connections_[id].publisher->borrow_loaned_message();
    output_message.get().size = 0;
    merge_history_into_sample(output_message.get(), input_message);

    uint32_t missed_samples = get_missed_samples_and_update_seq_nr(
      input_message,
      connections_[id].input_sequence_number);

    set_sample(this->get_name(), output_sequence, missed_samples, timestamp, output_message.get());

    if (event_logger_) {
      event_logger_->output_dependencies(output_sequence, timestamp, {{id, callback_id}});
    }

    // use result so that it is not optimizied away by some clever compiler
    output_message.get().data[0] = number_cruncher_result;
    connections_[id].publisher->publish(std::move(output_message));
    gettimeofday(&c2, NULL);
    print_execution_time(
      "Intersection", this->get_name(),
      (c2.tv_sec - c1.tv_sec) * 1000000 + (c2.tv_usec - c1.tv_usec));
  }

private:
  struct Connection
  {
    rclcpp::Publisher<message_t>::SharedPtr publisher;
    rclcpp::Subscription<message_t>::SharedPtr subscription;
    uint64_t number_crunch_limit;
    uint32_t sequence_number = 0;
    uint32_t input_sequence_number = 0;
  };
  std::vector<Connection> connections_;
  std::unique_ptr<reference_system::events::ComponentEventLogger> event_logger_;
  uint32_t input_callback_sequence_ = 0;
};
}  // namespace rclcpp_system
}  // namespace nodes
#endif  // REFERENCE_SYSTEM__NODES__RCLCPP__INTERSECTION_HPP_
