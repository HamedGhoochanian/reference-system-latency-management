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
    uint64_t timestamp = now_as_int();
    auto number_cruncher_result = number_cruncher(number_crunch_limit_);
    gettimeofday(&c1, NULL);
    auto output_message = publisher_->borrow_loaned_message();
    output_message.get().size = 0;

    uint32_t missed_samples = 0;
    for (auto & s : subscriptions_) {
      if (!s.cache) {continue;}

      missed_samples += get_missed_samples_and_update_seq_nr(s.cache, s.sequence_number);

      merge_history_into_sample(output_message.get(), s.cache);
      s.cache.reset();
    }
    set_sample(
      this->get_name(), sequence_number_++, missed_samples, timestamp,
      output_message.get());

    bool has_period = false;
    uint64_t period_ns = 0;
    bool violated = false;
    if (is_structured_output_enabled() && previous_timestamp_ != 0) {
      uint64_t elapsed;
      if (elapsed_ns(previous_timestamp_, timestamp, elapsed)) {
        period_ns = elapsed / std::max(sequence_number_ - 1 - previous_sequence_, 1U);
        double period_ms = static_cast<double>(period_ns) / 1000000.0;
        violated = std::abs(period_ms - 100.0) > 10.0;
        has_period = true;
      }
    }
    previous_timestamp_ = timestamp;
    previous_sequence_ = sequence_number_ - 1;

    output_message.get().data[0] = number_cruncher_result;
    publisher_->publish(std::move(output_message));
    uint64_t sink_timestamp = now_as_int();
    if (is_structured_output_enabled()) {
      emit_structured_source_record(this->get_name(), sequence_number_ - 1, timestamp);
      if (has_period) {
        std::vector<std::string> lineage{this->get_name()};
        emit_structured_chain_record(
          "behavior_planner_cyclic_jitter",
          this->get_name(), sequence_number_ - 1, timestamp,
          this->get_name(), sequence_number_ - 1, sink_timestamp,
          period_ns, lineage,
          violated ? "violated" : "completed", 0);
      }
    }
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
  uint64_t previous_timestamp_ = 0;
  uint32_t previous_sequence_ = 0;
};
}  // namespace rclcpp_system
}  // namespace nodes
#endif  // REFERENCE_SYSTEM__NODES__RCLCPP__CYCLIC_HPP_
