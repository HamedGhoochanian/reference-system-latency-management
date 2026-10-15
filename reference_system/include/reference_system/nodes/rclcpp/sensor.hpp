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
#ifndef REFERENCE_SYSTEM__NODES__RCLCPP__SENSOR_HPP_
#define REFERENCE_SYSTEM__NODES__RCLCPP__SENSOR_HPP_
#include <chrono>
#include <string>
#include <utility>
#include <iostream>
#include <sys/time.h>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <rcl/timer.h>
#include "reference_system/source_schedule.hpp"
#include "rclcpp/rclcpp.hpp"
#include "reference_system/nodes/settings.hpp"
#include "reference_system/sample_management.hpp"
#include "reference_system/msg_types.hpp"
#include "reference_system/provenance.hpp"

namespace nodes
{
namespace rclcpp_system
{

class Sensor : public rclcpp::Node
{
public:
  explicit Sensor(const SensorSettings & settings)
  : Node(settings.node_name), source_kind_(settings.source_kind)
  {
    register_provenance_producer(this->get_name());
    if (!settings.source_schedule_file.empty()) {
      const char * boot = std::getenv("REFERENCE_SYSTEM_BOOT_ID");
      const char * instance = std::getenv("REFERENCE_SYSTEM_INSTANCE_ID");
      if (!boot || !instance) {throw std::runtime_error("source schedule requires explicit producer identity");}
      boot_ = boot;
      instance_ = instance;
      schedule_ = std::make_unique<nodes::SourceSchedule>(settings.source_schedule_file,
          this->get_name(), boot_, instance_, settings.cycle_time.count());
      const auto output = std::string("/artifacts/raw/") + this->get_name() + ".source-applications.jsonl";
      if (std::filesystem::exists(output)) {throw std::runtime_error("source application evidence already exists");}
      applications_.open(output);
      if (!applications_) {throw std::runtime_error("cannot open source application file");}
    }
    publisher_ = this->create_publisher<message_t>(settings.topic_name, 1);
    timer_ = this->create_wall_timer(
      settings.cycle_time,
      [this] {timer_callback();});
#ifdef PICAS
    timer_->callback_priority = settings.callback_priority;
#endif
  }

private:
  struct timeval c1, c2;
  void timer_callback()
  {
    auto tracking_callback = provenance_callback_guard(this->get_name());
    gettimeofday(&c1, NULL);
    uint64_t timestamp = provenance_now_ns();
    apply_source_schedule();
    auto message = publisher_->borrow_loaned_message();
    message.get().size = 0;

    uint32_t sequence = sequence_number_++;
    set_sample(this->get_name(), sequence, 0, timestamp, message.get());
    const auto kind = source_kind_;
    const provenance_source_t source{kind, sequence, timestamp};
    set_provenance(message.get(), this->get_name(), sequence, 0, source);
    if (kind != 0) emit_provenance_source(this->get_name(), sequence, kind, timestamp);

    publisher_->publish(std::move(message));
    gettimeofday(&c2, NULL);
    print_execution_time(
      "Sensor", this->get_name(),
      (c2.tv_sec - c1.tv_sec) * 1000000 + (c2.tv_usec - c1.tv_usec));
  }

private:
  void apply_source_schedule()
  {
    if (!schedule_) {return;}
    const auto due = schedule_->due(static_cast<int64_t>(provenance_now_ns()));
    if (due.empty()) {return;}
    int64_t current = 0;
    if (rcl_timer_get_period(timer_->get_timer_handle().get(), &current) != RCL_RET_OK) {
      throw std::runtime_error("cannot read source timer period");
    }
    for (size_t i=0; i+1<due.size(); ++i) {
      schedule_->record(applications_, due[i], "skipped", this->get_name(), boot_, instance_,
        current, current, static_cast<int64_t>(provenance_now_ns()));
    }
    const auto index = due.back();
    int64_t old_period = 0;
    if (rcl_timer_exchange_period(timer_->get_timer_handle().get(),
      schedule_->phase(index).period, &old_period) != RCL_RET_OK) {
      throw std::runtime_error("source timer period exchange failed");
    }
    timer_->reset(); // Serialized immediately after exchange, in the existing callback.
    schedule_->record(applications_, index, "applied", this->get_name(), boot_, instance_,
      old_period, schedule_->phase(index).period, static_cast<int64_t>(provenance_now_ns()));
    schedule_->applied(index);
  }

  std::unique_ptr<nodes::SourceSchedule> schedule_;
  std::ofstream applications_;
  std::string boot_, instance_;
  uint32_t source_kind_;
  rclcpp::Publisher<message_t>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  uint32_t sequence_number_ = 0;
};
}  // namespace rclcpp_system
}  // namespace nodes
#endif  // REFERENCE_SYSTEM__NODES__RCLCPP__SENSOR_HPP_
