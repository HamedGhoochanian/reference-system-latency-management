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
#ifndef REFERENCE_SYSTEM__PROVENANCE_HPP_
#define REFERENCE_SYSTEM__PROVENANCE_HPP_

#include <cstdint>
#include <ctime>
#include <iostream>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include "reference_system/msg_types.hpp"
#include "reference_system/sample_management.hpp"

struct provenance_source_t
{
  uint32_t kind = 0;
  uint32_t sequence = 0;
  uint64_t timestamp_ns = 0;
  bool valid() const {return kind != 0;}
};

struct provenance_message_t
{
  std::string name;
  uint32_t sequence = 0;
  uint32_t channel = 0;
};

inline uint64_t provenance_now_ns()
{
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return static_cast<uint64_t>(now.tv_sec) * 1000000000ULL + now.tv_nsec;
}

inline provenance_source_t provenance_source(const message_t & message)
{
  return {message.source_kind, message.source_sequence, message.source_timestamp_ns};
}

inline provenance_source_t oldest_provenance_source(
  const std::vector<provenance_source_t> & sources)
{
  provenance_source_t oldest;
  for (const auto & source : sources) {
    if (source.valid() && (!oldest.valid() || source.timestamp_ns < oldest.timestamp_ns)) {
      oldest = source;
    }
  }
  return oldest;
}

inline void set_provenance(
  message_t & message, const std::string & node, uint32_t sequence, uint32_t channel,
  const provenance_source_t & source)
{
  message.provenance_node = node;
  message.provenance_sequence = sequence;
  message.provenance_channel = channel;
  message.source_kind = source.kind;
  message.source_sequence = source.sequence;
  message.source_timestamp_ns = source.timestamp_ns;
}

inline provenance_message_t provenance_identity(const message_t & message)
{
  return {message.provenance_node, message.provenance_sequence, message.provenance_channel};
}

inline void write_provenance_record(const std::string & record)
{
  if (!is_structured_output_enabled()) return;
  static std::mutex mutex;
  std::lock_guard<std::mutex> lock(mutex);
  std::cout << record << std::endl;
}

inline std::string provenance_source_json(const provenance_source_t & source)
{
  if (!source.valid()) return "null";
  return "{\"kind\":" + std::to_string(source.kind) +
         ",\"seq\":" + std::to_string(source.sequence) +
         ",\"t_ns\":" + std::to_string(source.timestamp_ns) + "}";
}

inline void emit_provenance_source(
  const std::string & node, uint32_t sequence, uint32_t kind, uint64_t timestamp_ns)
{
  write_provenance_record(
    "{\"schema\":1,\"event\":\"source\",\"t_ns\":" +
    std::to_string(timestamp_ns) + ",\"node\":\"" + node + "\",\"seq\":" +
    std::to_string(sequence) + ",\"kind\":" + std::to_string(kind) + "}");
}

inline void emit_provenance_link(
  const std::string & node, uint32_t sequence, uint32_t channel, uint64_t timestamp_ns,
  const std::vector<std::pair<uint32_t, provenance_message_t>> & inputs,
  const provenance_source_t & source, bool timer)
{
  std::string record = "{\"schema\":1,\"event\":\"link\",\"t_ns\":" +
    std::to_string(timestamp_ns) + ",\"node\":\"" + node + "\",\"msg\":{\"name\":\"" +
    node + "\",\"seq\":" + std::to_string(sequence) + ",\"channel\":" +
    std::to_string(channel) + "},\"inputs\":[";
  for (size_t i = 0; i < inputs.size(); ++i) {
    if (i != 0) record += ",";
    const auto & input = inputs[i];
    record += "{\"index\":" + std::to_string(input.first) + ",\"msg\":{\"name\":\"" +
      input.second.name + "\",\"seq\":" + std::to_string(input.second.sequence) +
      ",\"channel\":" + std::to_string(input.second.channel) + "}}";
  }
  record += "],\"source\":" + provenance_source_json(source) +
    ",\"timer\":" + std::string(timer ? "true" : "false") + "}";
  write_provenance_record(record);
}

inline void emit_provenance_sink(
  const std::string & node, uint32_t sequence, uint64_t timestamp_ns,
  const provenance_message_t & input, const provenance_source_t & source)
{
  write_provenance_record(
    "{\"schema\":1,\"event\":\"sink\",\"t_ns\":" +
    std::to_string(timestamp_ns) + ",\"node\":\"" + node + "\",\"msg\":{\"name\":\"" +
    node + "\",\"seq\":" + std::to_string(sequence) +
    ",\"channel\":0},\"input\":{\"name\":\"" + input.name + "\",\"seq\":" +
    std::to_string(input.sequence) + ",\"channel\":" + std::to_string(input.channel) +
    "},\"source\":" + provenance_source_json(source) +
    ",\"sensor_started\":" + std::string(source.valid() ? "true" : "false") + "}");
}

#endif  // REFERENCE_SYSTEM__PROVENANCE_HPP_
