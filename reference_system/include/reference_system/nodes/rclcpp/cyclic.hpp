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
#ifndef REFERENCE_SYSTEM__NODES__RCLCPP__CYCLIC_HPP_
#define REFERENCE_SYSTEM__NODES__RCLCPP__CYCLIC_HPP_
#include <chrono>
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
#include "reference_system/provenance.hpp"

namespace nodes
{
namespace rclcpp_system
{

class Cyclic : public rclcpp::Node
{
public:
  explicit Cyclic(const CyclicSettings & settings)
  : Node(settings.node_name),
    number_crunch_limit_(settings.number_crunch_limit)
  {
    register_provenance_producer(this->get_name());
    uint64_t input_number = 0U;
    for (const auto & input_topic : settings.inputs) {
      subscriptions_.emplace_back(
        subscription_t{
            this->create_subscription<message_t>(
              input_topic, 1,
              [this, input_number](const message_t::SharedPtr msg) {
                input_callback(input_number, msg);
              }), 0, message_t::SharedPtr()});
      ++input_number;
    }
    publisher_ = this->create_publisher<message_t>(settings.output_topic, 1);
    timer_ = this->create_wall_timer(
      settings.cycle_time,
      [this] {timer_callback();});
#ifdef PICAS
    subscriptions_[0].subscription->callback_priority = settings.callback_priority_1;
    subscriptions_[1].subscription->callback_priority = settings.callback_priority_2;
    subscriptions_[2].subscription->callback_priority = settings.callback_priority_3;
    subscriptions_[3].subscription->callback_priority = settings.callback_priority_4;
    subscriptions_[4].subscription->callback_priority = settings.callback_priority_5;
    subscriptions_[5].subscription->callback_priority = settings.callback_priority_6;
    timer_->callback_priority = settings.callback_priority_7;
#endif
  }

private:
  struct timeval c1, c2;
  void input_callback(
    const uint64_t input_number,
    const message_t::SharedPtr input_message)
  {
    gettimeofday(&c1, NULL);
    subscriptions_[input_number].cache = input_message;
    gettimeofday(&c2, NULL);
    double time_diff = (c2.tv_sec - c1.tv_sec) * 1000000 + c2.tv_usec - c1.tv_usec;
    print_execution_time("Cyclic", this->get_name(), time_diff);
  }

  void timer_callback()
  {
    auto tracking_callback = provenance_callback_guard(this->get_name());
    uint64_t timestamp = provenance_now_ns();
    auto number_cruncher_result = number_cruncher(number_crunch_limit_);
    gettimeofday(&c1, NULL);
    auto output_message = publisher_->borrow_loaned_message();
    output_message.get().size = 0;

    uint32_t missed_samples = 0;
    std::vector<std::pair<uint32_t, provenance_message_t>> inputs;
    std::vector<provenance_source_t> sources;
    for (size_t i = 0; i < subscriptions_.size(); ++i) {
      if (!subscriptions_[i].cache) continue;
      inputs.emplace_back(static_cast<uint32_t>(i), provenance_identity(*subscriptions_[i].cache));
      sources.push_back(provenance_source(*subscriptions_[i].cache));
    }
    for (auto & s : subscriptions_) {
      if (!s.cache) {continue;}

      missed_samples += get_missed_samples_and_update_seq_nr(s.cache, s.sequence_number);

      merge_history_into_sample(output_message.get(), s.cache);
      s.cache.reset();
    }
    set_sample(
      this->get_name(), sequence_number_++, missed_samples, timestamp,
      output_message.get());

    output_message.get().data[0] = number_cruncher_result;
    const uint32_t output_sequence = sequence_number_ - 1;
    const auto source = oldest_provenance_source(sources);
    set_provenance(output_message.get(), this->get_name(), output_sequence, 0, source);
    publisher_->publish(std::move(output_message));
    emit_provenance_link(
      this->get_name(), output_sequence, 0, provenance_now_ns(), inputs, source, true);
    gettimeofday(&c2, NULL);
    print_execution_time(
      "Cyclic", std::string(this->get_name()) + "Timer",
      (c2.tv_sec - c1.tv_sec) * 1000000 + (c2.tv_usec - c1.tv_usec));
  }

private:
  rclcpp::Publisher<message_t>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  struct subscription_t
  {
    rclcpp::Subscription<message_t>::SharedPtr subscription;
    uint32_t sequence_number = 0;
    message_t::SharedPtr cache;
  };

  std::vector<subscription_t> subscriptions_;
  uint64_t number_crunch_limit_;
  uint32_t sequence_number_ = 0;
};
}  // namespace rclcpp_system
}  // namespace nodes
#endif  // REFERENCE_SYSTEM__NODES__RCLCPP__CYCLIC_HPP_
